/*
 * ============================================================================
 *  ecg_hr.c —— ECG 心率检测算法实现
 *
 *  功能：从 100 Hz 的滤波 ECG 样本流中实时检测 R 峰位置，计算心率和
 *        RR 间隔。核心是一个基于自适应阈值和候选峰跟踪的状态机。
 *
 *  算法概述：
 *    ① 每 2 秒（200 样本）用滑动窗口计算窗内信号幅度（max - min），
 *       阈值为 window_min + 幅度 × 80%。
 *    ② 信号从低于阈值 → 高于阈值时，启动候选峰跟踪模式。
 *    ③ 在候选峰模式下，持续跟踪局部最高点，最长跟踪 250 ms。
 *    ④ 信号开始下降 或 跟踪超时后，确认候选峰为有效 R 峰。
 *    ⑤ 峰顶必须高出阈值至少一个突出度（峰峰值 × 10%，最少 8 ADC码）。
 *    ⑥ 有效的 RR 间隔必须在 33~150 样本之间，且偏离当前代表间隔
 *      不超过 ±35%，才更新心率。
 *    ⑦ 心率用最近 3 个 RR 间隔的中值/平均计算：HR = 6000 / 代表间隔。
 *
 *  设计原则：
 *    - 所有内部函数均为 static，不暴露给外部调用。
 *    - 使用 uint32_t 无符号减法计算间隔，无需担心短期溢出。
 *    - 环形缓存管理 RR 间隔历史，抗单次毛刺。
 * ============================================================================
 */

#include "ecg_hr.h"
#include <stdbool.h>

/*
 * ECG_HR_UINT16_MAX：用于初始化窗口最小值为最大可能值，
 * 确保第一个样本进入窗口时能正确更新 window_min。
 */
#define ECG_HR_UINT16_MAX 65535U

/*
 * ============================================================================
 *  内部辅助函数
 * ============================================================================
 */

/*
 * ECG_HR_ClearIntervals —— 清空 RR 间隔环形历史
 *
 * 作用：将所有 RR 间隔槽位置零，重置计数和写入指针。
 * 不影响阈值、窗口和武装状态。在节律失效和初始化时调用。
 *
 * 参数 detector：非空检测器指针。
 * 返回值：无。
 */
static void ECG_HR_ClearIntervals(ECG_HRDetector *detector)
{
    /*
     * index：遍历三个 RR 历史槽位的循环变量，取值 0、1、2。
     */
    uint8_t index;

    /* 重置环形缓存的计数器和写入指针 */
    detector->interval_count = 0U;
    detector->interval_write_index = 0U;

    /*
     * 循环逐个清空三个 RR 间隔历史槽位。
     * ECG_HR_INTERVAL_HISTORY_SIZE 固定为 3，编译器会自动展开此循环。
     */
    for (index = 0U; index < ECG_HR_INTERVAL_HISTORY_SIZE; ++index)
    {
        detector->intervals[index] = 0U;
    }
}

/*
 * ECG_HR_InvalidateRhythm —— 使当前心率失效
 *
 * 作用：将心率置零，清除 RR 计时基准和候选参考点，
 * 要求下一个 R 峰重新建立 RR 计时基准（首个 R 峰只设基准不计算间隔）。
 * 在信号饱和、超时或窗口无有效幅度时调用。
 *
 * 参数 detector：非空检测器指针。
 * 返回值：无。
 */
static void ECG_HR_InvalidateRhythm(ECG_HRDetector *detector)
{
    /* 心率输出置零，上位机应显示占位符 */
    detector->heart_rate = 0U;

    /*
     * 清除 RR 计时基准标志：
     * has_peak_reference = 0 → 下一个通过候选确认的 R 峰
     *   将只建立 last_peak_sample 基准，不输出 RR 间隔。
     * has_candidate_reference = 0 → 下一个候选峰将建立
     *   last_candidate_sample 参考点，跳过候选间隔检查。
     */
    detector->has_peak_reference = 0U;
    detector->has_candidate_reference = 0U;

    /* 清空 RR 间隔历史，从零开始累积 */
    ECG_HR_ClearIntervals(detector);
}

/*
 * ECG_HR_RearmLevel —— 计算重武装电平
 *
 * 作用：R 峰检测完毕后，信号必须回落到重武装电平以下，才能允许检测
 * 下一个 R 峰。迟滞量同时受窗口幅度比例（REARM_PERCENT）和最小ADC码
 * 限制：
 *   迟滞 = max(峰峰值 × 20%, 30 ADC码)
 *   重武装电平 = threshold - 迟滞
 *
 * 大信号使用更宽迟滞，避免 R 峰顶部的小凹陷引起多次重复触发；
 * 小信号使用最小迟滞（30 ADC码），保证小幅度信号也能正常回落重武装。
 *
 * 参数 detector：非空检测器指针。
 * 返回值：重武装电平的 ADC 值。如果 threshold ≤ 迟滞，返回 0 防止下溢。
 */
static uint16_t ECG_HR_RearmLevel(const ECG_HRDetector *detector)
{
    /*
     * hysteresis：按峰峰值比例计算的回落迟滞量。
     * 使用 uint32_t 中间计算避免 16 位乘法溢出。
     * +50U 用于实现四舍五入。
     */
    uint16_t hysteresis =
        (uint16_t)(((uint32_t)detector->window_peak_to_peak *
                    ECG_HR_REARM_PERCENT +
                    50U) /
                   100U);

    /* 如果比例迟滞小于最小迟滞，使用最小迟滞保障 */
    if (hysteresis < ECG_HR_REARM_MIN)
    {
        hysteresis = ECG_HR_REARM_MIN;
    }

    /* 阈值大于迟滞时正常相减；否则返回 0 防下溢 */
    if (detector->threshold > hysteresis)
    {
        return (uint16_t)(detector->threshold - hysteresis);
    }

    return 0U;
}

/*
 * ECG_HR_PeakProminence —— 计算候选峰的最小突出度
 *
 * 作用：候选峰确认时，候选峰顶值必须高出阈值至少此值，防止噪声波动
 * 被误认为 R 峰。
 *   突出度 = max(峰峰值 × 10%, 8 ADC码)
 *
 * 参数 detector：非空检测器指针。
 * 返回值：最小突出度的 ADC 码数。
 */
static uint16_t ECG_HR_PeakProminence(const ECG_HRDetector *detector)
{
    /*
     * prominence：按峰峰值比例计算的最小突出度。
     * +50U 用于四舍五入。
     */
    uint16_t prominence =
        (uint16_t)(((uint32_t)detector->window_peak_to_peak *
                    ECG_HR_PEAK_PROMINENCE_PERCENT +
                    50U) /
                   100U);

    /*
     * 如果比例突出度小于最小值，返回最小值；
     * 确保极小信号也有基本的抗噪能力。
     */
    return prominence < ECG_HR_PEAK_PROMINENCE_MIN
               ? ECG_HR_PEAK_PROMINENCE_MIN
               : prominence;
}

/*
 * ECG_HR_RepresentativeInterval —— 计算抗毛刺的 RR 代表间隔
 *
 * 作用：根据当前 RR 间隔历史数量，选择不同的抗毛刺策略：
 *   - 1 个间隔：直接使用（无其他数据可参考）
 *   - 2 个间隔：求算术平均（平滑短时波动）
 *   - 3 个间隔：取中值（有效抑制单次异常长/短间隔）
 *
 * 这样单次过长或过短的 RR 不会立刻造成网页心率大幅跳动。
 * 例如 [83, 83, 166] 的中值为 83，HR 保持 72 BPM 不受漏检影响。
 *
 * 参数 detector：非空检测器指针。
 * 返回值：代表 RR 间隔（样本数）。
 */
static uint16_t ECG_HR_RepresentativeInterval(const ECG_HRDetector *detector)
{
    /*
     * first / second / third：
     *   三个 RR 间隔的排序副本。first 保存最小值，second 保存中值，
     *   third 保存最大值。使用简单的冒泡式三值排序网络完成。
     */
    uint16_t first;
    uint16_t second;
    uint16_t third;

    /* 情况1：只有一个 RR 间隔，直接返回它 */
    if (detector->interval_count == 1U)
    {
        return detector->intervals[0];
    }

    /* 情况2：两个 RR 间隔，返回它们的平均值（整数除法自动截断） */
    if (detector->interval_count == 2U)
    {
        return (uint16_t)((detector->intervals[0] + detector->intervals[1]) /
                          2U);
    }

    /* 情况3：三个 RR 间隔，取中值 */

    /* 将三个间隔值复制到局部变量 */
    first = detector->intervals[0];
    second = detector->intervals[1];
    third = detector->intervals[2];

    /*
     * 三值排序网络：共 3 次比较交换，保证 second 最终是中值。
     * 第1步：确保 first ≤ second
     */
    if (first > second)
    {
        /*
         * temporary：交换 first 和 second 时的临时存储。
         * 比声明临时 uint16_t 更高效，避免值反复加载/存储。
         */
        uint16_t temporary = first;

        first = second;
        second = temporary;
    }

    /* 第2步：确保 second ≤ third */
    if (second > third)
    {
        /*
         * temporary：交换 second 和 third 时的临时存储。
         */
        uint16_t temporary = second;

        second = third;
        third = temporary;
    }

    /* 第3步：再次确保 first ≤ second（第2步可能破坏了第1步的顺序） */
    if (first > second)
    {
        second = first;
    }

    /* second 现在就是三个值的中值 */
    return second;
}

/*
 * ECG_HR_IsIntervalConsistent —— 检查 RR 间隔一致性
 *
 * 作用：当算法已经锁定至少一个 RR 间隔后，新间隔必须接近当前代表间隔，
 * 偏离不超过 RR_TOLERANCE_PERCENT（±35%）。
 * 例如当前代表间隔 = 100 样本，新间隔必须在 65~135 样本范围内。
 *
 * 该保护主要拦截：
 *   - 真实 R 峰之间的高 T 波误检（产生过短间隔，如 30~50 样本）
 *   - 漏检后形成的倍长间隔（如 166 样本，偏离 100 样本基准 66%）
 *
 * 参数 detector：非空检测器指针。
 *     interval：待检查的新 RR 间隔（样本数）。
 * 返回值：true 表示间隔在容忍范围内，应接受；false 表示应丢弃。
 */
static bool ECG_HR_IsIntervalConsistent(const ECG_HRDetector *detector,
                                        uint16_t interval)
{
    /*
     * representative：当前 RR 代表间隔。
     * lower / upper：容忍范围的上下限（闭区间）。
     * 使用 uint32_t 中间计算避免 16 位乘法溢出。
     */
    uint16_t representative;
    uint32_t lower;
    uint32_t upper;

    /* 尚无 RR 基准时，任何有效间隔都接受 */
    if (detector->interval_count == 0U)
    {
        return true;
    }

    /* 获取当前代表间隔 */
    representative = ECG_HR_RepresentativeInterval(detector);

    /*
     * 计算下限：representative × (100% - 35%)
     * 整数除法自动截断，保证结果不超过精确下限。
     */
    lower = (uint32_t)representative *
            (100U - ECG_HR_RR_TOLERANCE_PERCENT) / 100U;

    /*
     * 计算上限：representative × (100% + 35%)
     * +99U 实现除法向上取整，保证结果不低于精确上限。
     */
    upper = ((uint32_t)representative *
                 (100U + ECG_HR_RR_TOLERANCE_PERCENT) +
             99U) /
            100U;

    /* 返回是否在闭区间 [lower, upper] 内 */
    return interval >= lower && interval <= upper;
}

/*
 * ECG_HR_RecordInterval —— 保存 RR 间隔并更新心率
 *
 * 作用：将一个通过所有检查的有效 RR 间隔存入环形历史，
 * 然后用代表间隔计算心率（四舍五入到整数 BPM）：
 *   HR = (100 × 60 + 代表间隔/2) / 代表间隔
 *      = (6000 + 代表间隔/2) / 代表间隔
 *
 * 四舍五入逻辑：+代表间隔/2 在整数除法前将余数推到 ≥ 一半的位置，
 * 等效于 floor((6000 + 代表间隔/2) / 代表间隔)。
 *
 * 参数 detector：非空检测器指针。
 *     interval：有效的 RR 间隔（样本数）。
 * 返回值：无。
 */
static void ECG_HR_RecordInterval(ECG_HRDetector *detector, uint16_t interval)
{
    /*
     * representative：用于本次 BPM 换算的抗毛刺 RR 代表间隔。
     * 由 ECG_HR_RepresentativeInterval 根据历史数量选择策略。
     */
    uint16_t representative;

    /*
     * 将新间隔写入环形缓存的当前写入位置。
     * interval_write_index 在 Init 和 ClearIntervals 中重置为 0。
     */
    detector->intervals[detector->interval_write_index] = interval;

    /*
     * 更新环形缓存写入指针（循环向前移动一个位置）。
     * 当达到 HISTORY_SIZE 时回绕到 0。
     */
    detector->interval_write_index =
        (uint8_t)((detector->interval_write_index + 1U) %
                  ECG_HR_INTERVAL_HISTORY_SIZE);

    /*
     * 如果历史中有效间隔数尚未达到最大容量，递增计数。
     * 达到容量后不再增加，新间隔覆盖最旧的间隔。
     */
    if (detector->interval_count < ECG_HR_INTERVAL_HISTORY_SIZE)
    {
        ++detector->interval_count;
    }

    /* 获取抗毛刺后的代表间隔 */
    representative = ECG_HR_RepresentativeInterval(detector);

    /*
     * 计算心率：BPM = 采样率 × 60 秒 / 代表间隔（样本数）。
     * 适用于任何采样率下的心率换算。
     * 当代表间隔 = 0 时不会发生（interval 非零），因此安全。
     */
    detector->heart_rate =
        (uint16_t)((ECG_HR_SAMPLE_RATE_HZ * 60U + representative / 2U) /
                   representative);
}

/*
 * ECG_HR_ConfirmCandidate —— 确认候选峰为有效 R 峰
 *
 * 调用时机：候选峰跟踪结束时（信号开始下降或跟踪超时）调用一次。
 *
 * 工作步骤：
 *   ① 峰顶突出度检查：candidate_peak_value 必须 ≥ threshold + 突出度
 *      → 不通过则认为是噪声或 T 波，直接丢弃，不更新任何基准。
 *   ② 候选间隔检查：相邻候选峰之间的样本数 ≥ MIN_INTERVAL_SAMPLES
 *      → 不通过则认为高频噪声，不更新 RR 计时基准。
 *       注意！即使被拒绝，last_candidate_sample 也会更新，确保后续
 *       高频噪声不会每隔一个就漏网（即不会产生倍频别名效应）。
 *   ③ 首个 R 峰检查：has_peak_reference 为 0 时，只建立参考基准，
 *      不输出 RR 间隔。确保第一个 RR 测量总是从完整的间隔开始。
 *   ④ RR 间隔计算：elapsed = 当前候选峰 - 上一个有效 R 峰
 *      → 必须在 [MIN, MAX] 范围内。
 *      → 必须通过一致性检查（与当前代表间隔偏差 ≤ 35%）。
 *      → 通过后保存间隔并更新心率。
 *
 * 参数 detector：非空检测器指针。
 * 返回值：无（结果通过更新 detector 内部状态体现）。
 */
static void ECG_HR_ConfirmCandidate(ECG_HRDetector *detector)
{
    /*
     * elapsed：当前候选峰与上一个有效 R 峰之间的样本数。
     * candidate_interval：当前候选峰与上一个候选峰之间的样本数，
     *   用于高频噪声去抖。
     * interval：elapsed 的 16 位版本，传入 RecordInterval。
     * prominence：本次确认所需的最小峰顶突出度。
     */
    uint32_t elapsed;
    uint32_t candidate_interval;
    uint16_t interval;
    uint16_t prominence = ECG_HR_PeakProminence(detector);

    /*
     * ========== 步骤①：峰顶突出度检查 ==========
     * 候选峰的局部最高点必须显著高出阈值。
     * 使用 uint32_t 比较防止 16 位加法溢出。
     * 不通过则直接返回，不更新任何状态。
     */
    if ((uint32_t)detector->candidate_peak_value <
        (uint32_t)detector->threshold + prominence)
    {
        return;
    }

    /*
     * ========== 步骤②：候选间隔检查 ==========
     * 计算与上一个候选峰的时间差，用于拒绝高频连续噪声。
     * 注意：即使本步拒绝，last_candidate_sample 也已经更新，
     * 所以下一波噪声会被更严格地拦截。
     */
    candidate_interval =
        detector->candidate_peak_sample - detector->last_candidate_sample;
    detector->last_candidate_sample = detector->candidate_peak_sample;
    if (detector->has_candidate_reference != 0U &&
        candidate_interval < ECG_HR_MIN_INTERVAL_SAMPLES)
    {
        /*
         * 每个局部候选峰都会移动候选参考点。连续高频噪声因此始终
         * 保持在最短RR以内，不会每隔一个噪声峰产生倍频别名。
         * 返回时不更新 last_peak_sample，RR 计时基准不受影响。
         */
        return;
    }
    /* 首个候选峰已建立参考点，后续候选峰执行间隔检查 */
    detector->has_candidate_reference = 1U;

    /*
     * ========== 步骤③：首个 R 峰检查 ==========
     * has_peak_reference = 0 表示这是节律失效后的第一个 R 峰，
     * 只建立 RR 计时基准（last_peak_sample），不输出心率。
     */
    if (detector->has_peak_reference == 0U)
    {
        detector->last_peak_sample = detector->candidate_peak_sample;
        detector->has_peak_reference = 1U;
        return;
    }

    /*
     * ========== 步骤④：RR 间隔计算与验证 ==========
     * 计算当前候选峰与上一个有效 R 峰的间隔样本数。
     * 使用 uint32_t 无符号减法，即使溢出也能正确计算短间隔。
     */
    elapsed = detector->candidate_peak_sample - detector->last_peak_sample;

    /*
     * 间隔必须在 [MIN_INTERVAL_SAMPLES, MAX_INTERVAL_SAMPLES] 范围内。
     * 小于 MIN：两峰太近，可能是 T 波误检或噪声。
     * 大于 MAX：可能漏检了中间的 R 峰，需要丢弃。
     */
    if (elapsed >= ECG_HR_MIN_INTERVAL_SAMPLES &&
        elapsed <= ECG_HR_MAX_INTERVAL_SAMPLES)
    {
        interval = (uint16_t)elapsed;

        /*
         * 一致性检查：已锁定节律后，新间隔不能偏离当前代表间隔
         * 超过 ±35%。T 波误检产生的过短间隔和漏检产生的倍长间隔
         * 在此被过滤。
         */
        if (ECG_HR_IsIntervalConsistent(detector, interval))
        {
            /* 通过所有检查：更新 R 峰基准并记录间隔 */
            detector->last_peak_sample = detector->candidate_peak_sample;
            ECG_HR_RecordInterval(detector, interval);
        }
        /*
         * 未通过一致性检查的间隔被静默丢弃。
         * last_peak_sample 不更新，RR 计时基准保持原值。
         * 下一个 R 峰将以原基准计算间隔，不会引入累计误差。
         */
    }
    /*
     * 超出 [MIN, MAX] 范围的间隔被静默丢弃。
     * 例如漏检产生的 166 样本间隔（> 150）会被丢弃。
     * 后续超时检查（ECG_HR_MAX_INTERVAL_SAMPLES）可能会触发节律失效。
     */
}

/*
 * ECG_HR_UpdateWindow —— 将新样本纳入 2 秒自适应阈值窗口
 *
 * 作用：将当前样本纳入滑动窗口，更新 window_max 和 window_min。
 * 每收集 200 个样本（2 秒）后：
 *   - 计算峰峰值 peak_to_peak = max - min
 *   - 如果峰峰值 < MIN_PEAK_TO_PEAK（信号过弱），清空阈值使检测暂停
 *   - 否则重新计算阈值 = window_min + 峰峰值 × 80%（带四舍五入）
 *   - 清除候选跟踪状态，根据当前信号位置设置武装状态
 *
 * 返回 true 时，调用者（ECG_HR_Process）应跳过本样本的点上穿判断，
 * 避免阈值跳变本身被当作一次向上穿越事件。
 *
 * 参数 detector：非空检测器指针。
 *     sample：当前 100 Hz ECG 样本值。
 * 返回值：true 表示窗口刚结束且阈值已更新；false 表示窗口仍在收集。
 */
static bool ECG_HR_UpdateWindow(ECG_HRDetector *detector, uint16_t sample)
{
    /*
     * peak_to_peak：当前完整窗口内信号最大值与最小值之差。
     * 用于判断信号幅度有效性并计算百分比阈值。
     */
    uint16_t peak_to_peak;

    /*
     * ========== 更新窗口极值 ==========
     * 窗口的第一个样本（window_count == 0）时，直接初始化 max 和 min。
     * 后续样本与当前极值比较，动态更新。
     */
    if (detector->window_count == 0U)
    {
        /* 首个样本同时设为最大值和最小值 */
        detector->window_max = sample;
        detector->window_min = sample;
    }
    else
    {
        /* 更新最大值 */
        if (sample > detector->window_max)
        {
            detector->window_max = sample;
        }
        /* 更新最小值 */
        if (sample < detector->window_min)
        {
            detector->window_min = sample;
        }
    }

    /* 窗口样本计数递增 */
    ++detector->window_count;

    /*
     * 窗口尚未满（不足 200 样本），返回 false。
     * 调用者继续正常的检峰流程。
     */
    if (detector->window_count < ECG_HR_WINDOW_SAMPLES)
    {
        return false;
    }

    /*
     * ========== 窗口结束：计算峰峰值 ==========
     * 整数减法，max ≥ min 由更新逻辑保证。
     */
    peak_to_peak = (uint16_t)(detector->window_max - detector->window_min);

    /* 重置窗口计数器，准备下一个 2 秒窗口 */
    detector->window_count = 0U;

    /*
     * ========== 信号幅度检查 ==========
     * 峰峰值小于 MIN_PEAK_TO_PEAK（100 ADC码）时，认为信号过弱
     * （传感器脱落、导联脱落或静音）。
     * 清除阈值就绪标志和武装标志，使心率检测暂停。
     * 同时清除候选跟踪状态，防止残留跟踪污染新窗口。
     * 调用 InvalidateRhythm 清空节律和历史。
     */
    if (peak_to_peak < ECG_HR_MIN_PEAK_TO_PEAK)
    {
        detector->threshold_ready = 0U;
        detector->armed = 0U;
        detector->candidate_active = 0U;
        ECG_HR_InvalidateRhythm(detector);
        return true;
    }

    /*
     * ========== 信号有效：更新阈值 ==========
     * 保存峰峰值供重武装和突出度等百分比计算使用。
     * 阈值 = window_min + 峰峰值 × 80%（带四舍五入，+50U 后 /100）。
     * 使用 uint32_t 中间计算防止 16 位乘法溢出。
     */
    detector->window_peak_to_peak = peak_to_peak;
    detector->threshold =
        (uint16_t)(detector->window_min +
                   ((uint32_t)peak_to_peak * ECG_HR_THRESHOLD_PERCENT + 50U) /
                       100U);

    /* 阈值就绪，允许峰值检测 */
    detector->threshold_ready = 1U;

    /*
     * 窗口边界处清除候选跟踪状态，避免窗口切换前的跟踪残留
     * 错误地进入新窗口的第一个样本。
     * 同时根据当前样本位置设置武装状态：
     *   样本 ≤ 重武装电平 → 允许检测下一个上升沿
     *   样本 > 重武装电平 → 继续等待信号回落
     */
    detector->candidate_active = 0U;
    detector->armed = sample <= ECG_HR_RearmLevel(detector) ? 1U : 0U;

    return true;
}

/*
 * ECG_HR_Init —— 完整复位心率检测器
 *
 * 作用：将所有状态变量恢复为初始值，包括：
 *   - 样本序号归零
 *   - 窗口极值、阈值清空
 *   - RR 计时基准和候选参考点清除
 *   - 心率输出置零
 *   - 所有状态标志复位
 *   - RR 间隔历史清空
 *
 * 在系统上电或 ECG 来源切换时调用，确保从干净的 2 秒窗口重新开始。
 *
 * 特别注意：
 *   window_min 初始化为 65535（uint16_t 最大值），而不是 0。
 *   这样第一个样本进入窗口时（window_count == 0），会同时将
 *   window_max 和 window_min 设为该样本的值，而不是停留在 0。
 *   如果初始化为 0，窗口收集期间的 window_min 可能被锁定为 0，
 *   导致峰峰值被夸大，阈值位置偏高。
 *
 * 参数 detector：检测器指针。NULL 时直接返回，保证调用安全。
 * 返回值：无。
 */
void ECG_HR_Init(ECG_HRDetector *detector)
{
    /* 空指针保护：防止未初始化指针导致 hardfault */
    if (detector == 0)
    {
        return;
    }

    /*
     * ---- 时间轴重置 ----
     * sample_index 从 0 开始单调递增。
     * 所有基准样本序号清零。
     */
    detector->sample_index = 0U;
    detector->last_peak_sample = 0U;
    detector->last_candidate_sample = 0U;
    detector->candidate_peak_sample = 0U;

    /*
     * ---- 窗口和阈值重置 ----
     * window_min 初始化为最大值，确保首个样本正确初始化。
     * threshold 为 0（无效值），threshold_ready 为 0。
     */
    detector->window_max = 0U;
    detector->window_min = ECG_HR_UINT16_MAX;
    detector->window_peak_to_peak = 0U;
    detector->window_count = 0U;
    detector->threshold = 0U;

    /*
     * ---- 样本历史重置 ----
     * previous_sample 和 candidate_peak_value 清零。
     * previous_ready 为 0，表示尚不能进行上升沿比较。
     */
    detector->previous_sample = 0U;
    detector->candidate_peak_value = 0U;

    /*
     * ---- 输出和状态重置 ----
     * heart_rate 置 0，上位机应显示占位符（如 "-- BPM"）。
     * 所有标志位清零，候选年龄归零。
     */
    detector->heart_rate = 0U;
    detector->candidate_age = 0U;
    detector->threshold_ready = 0U;
    detector->previous_ready = 0U;
    detector->armed = 0U;
    detector->candidate_active = 0U;
    detector->has_peak_reference = 0U;
    detector->has_candidate_reference = 0U;

    /* ---- RR 间隔历史清空 ---- */
    ECG_HR_ClearIntervals(detector);
}

/*
 * ============================================================================
 *  ECG_HR_Process —— 核心心率检测状态机
 *
 *  每个 100 Hz 滤波 ECG 样本调用一次。内部状态机按以下顺序执行：
 *
 *  步骤 ① 空指针保护：detector 为 NULL 时直接返回 0
 *  步骤 ② 获取当前样本序号并递增
 *  步骤 ③ ADC 饱和检测：信号接近电源轨 → 复位窗口和节律
 *  步骤 ④ RR 超时检测：超过 1.5 秒无有效 R 峰 → 节律失效
 *  步骤 ⑤ 窗口更新：将样本纳入 2 秒窗口，窗口结束则重新计算阈值
 *         窗口边界样本跳过后续检峰，避免阈值跳变误触发
 *  步骤 ⑥ 阈值就绪检查：前 2 秒阈值未建立 → 不检峰，返回 0
 *  步骤 ⑦ 候选峰跟踪中：
 *         a) 更新局部最高点
 *         b) 更新候选年龄
 *         c) 检测到信号下降或跟踪超时 → 确认候选峰 + 检查重武装
 *  步骤 ⑧ 重武装：信号已回落到重武装电平以下 → 允许检测下一个峰
 *  步骤 ⑨ 上升沿检测：信号从低于阈值穿越到高于阈值 → 启动候选跟踪
 *  步骤 ⑩ 保存当前样本到 previous_sample 供下次比较
 *
 *  返回值：detector->heart_rate，即当前保存的心率值（BPM）。
 *          0 表示尚未稳定、信号无效或节律失效。
 * ============================================================================
 */
uint16_t ECG_HR_Process(ECG_HRDetector *detector, uint16_t sample)
{
    /*
     * current_sample：当前样本的单调递增序号，取自 detector->sample_index
     *   并在本函数开始时递增。用于计算 RR 间隔。
     * elapsed：当前样本与上一个有效 R 峰之间的样本数，用于超时检测。
     * window_updated：true 表示本样本刚完成一个 2 秒阈值窗口，
     *   此时跳过上升沿检测，避免阈值跳变本身被当作穿越事件。
     */
    uint32_t current_sample;
    uint32_t elapsed;
    bool window_updated;

    /* ========== 步骤①：空指针保护 ========== */
    if (detector == 0)
    {
        return 0U;
    }

    /* ========== 步骤②：获取并递增样本序号 ========== */
    current_sample = detector->sample_index;
    ++detector->sample_index;

    /*
     * ========== 步骤③：ADC 饱和检测 ==========
     * ADC 触及电源轨（接近 0V 或 3.3V）时，ECG 波形已削波失真，
     * 无法可靠还原信号形状。此时立即使心率失效，丢弃当前窗口数据，
     * 清空候选跟踪和武装状态，从下一个 2 秒窗口重新开始。
     *
     * 上下边界保留约 100 ADC 码的摆幅余量（ECG_HR_ADC_SATURATION_LOW=100,
     * ECG_HR_ADC_SATURATION_HIGH=3995）。
     */
    if (sample <= ECG_HR_ADC_SATURATION_LOW ||
        sample >= ECG_HR_ADC_SATURATION_HIGH)
    {
        /* 清空窗口和状态 */
        detector->window_count = 0U;
        detector->threshold_ready = 0U;
        detector->previous_ready = 0U;
        detector->armed = 0U;
        detector->candidate_active = 0U;

        /* 使节律失效 */
        ECG_HR_InvalidateRhythm(detector);
        return 0U;
    }

    /*
     * ========== 步骤④：RR 超时检测 ==========
     * 如果已有 RR 计时基准（has_peak_reference=1）且当前不在候选峰
     * 跟踪中（candidate_active=0），检查上次 R 峰以来的样本数。
     *
     * 超过 MAX_INTERVAL_SAMPLES（150 样本 = 1.5 秒 = 40 BPM）时，
     * 认为节律丢失（可能传感器脱落或心律失常），使心率失效。
     * 注意：候选峰跟踪期间不检查超时，因为跟踪本身就在寻找峰。
     */
    if (detector->has_peak_reference != 0U &&
        detector->candidate_active == 0U)
    {
        elapsed = current_sample - detector->last_peak_sample;
        if (elapsed > ECG_HR_MAX_INTERVAL_SAMPLES)
        {
            ECG_HR_InvalidateRhythm(detector);
        }
    }

    /*
     * ========== 步骤⑤：窗口更新 ==========
     * 将当前样本纳入 2 秒自适应阈值窗口。
     * window_updated 为 true 时，表示窗口已满并重新计算了阈值。
     * 此时跳过所有后续检峰逻辑，只更新 previous_sample 后返回，
     * 避免阈值跳变被误当作上升沿穿越事件。
     */
    window_updated = ECG_HR_UpdateWindow(detector, sample);
    if (window_updated)
    {
        /*
         * 阈值每 2 秒可能发生跳变。边界样本只用于建立新的比较起点，
         * 避免阈值变化本身被当作一次向上穿越。
         * 同时这确保了窗口边界不会产生虚假的 R 峰检测。
         */
        detector->previous_sample = sample;
        detector->previous_ready = 1U;
        return detector->heart_rate;
    }

    /*
     * ========== 步骤⑥：阈值就绪检查 ==========
     * 前 2 秒（200 样本）阈值尚未建立，无法检测 R 峰。
     * 此时只保存样本历史，不执行任何检峰逻辑，返回 0。
     */
    if (detector->threshold_ready == 0U)
    {
        detector->previous_sample = sample;
        detector->previous_ready = 1U;
        return 0U;
    }

    /*
     * ========== 步骤⑦：候选峰跟踪模式 ==========
     * candidate_active = 1 表示已检测到一次阈值上穿事件，
     * 正在跟踪该事件的局部最高点。在此模式下：
     *
     * ⑦a) 如果当前样本高于已记录的最高点，更新最高点及其样本序号。
     *     这确保我们能找到 R 波的真实顶峰，而不是阈值穿越点。
     *
     * ⑦b) 候选年龄递增（最长跟踪 PEAK_SEARCH_SAMPLES = 25 = 250ms）。
     *     限制跟踪时长防止异常宽大的波形导致算法卡住。
     *
     * ⑦c) 检测信号是否开始下降：
     *     peak_falling = 当前样本 < 上一个样本（previous_ready 有效时）。
     *     一旦检测到下降，或候选年龄超时，立即：
     *     - 退出候选跟踪模式（candidate_active = 0）
     *     - 调用 ConfirmCandidate 确认候选峰
     *     - 如果当前样本已低于重武装电平，允许立即检测下一个峰
     */
    if (detector->candidate_active != 0U)
    {
        /*
         * peak_falling：true 表示信号刚从上升转为下降。
         * 判断依据：当前样本值 < 上一个样本值。
         * 首次下降确认后，R 峰的最高点已经记录在 candidate_peak_value 中。
         */
        bool peak_falling;

        /* ⑦a：更新局部最高点 */
        if (sample > detector->candidate_peak_value)
        {
            detector->candidate_peak_value = sample;
            detector->candidate_peak_sample = current_sample;
        }

        /* ⑦b：候选年龄递增（上限 250ms） */
        if (detector->candidate_age < ECG_HR_PEAK_SEARCH_SAMPLES)
        {
            ++detector->candidate_age;
        }

        /* ⑦c：下降检测或超时 → 确认候选峰 */
        peak_falling = detector->previous_ready != 0U &&
                       sample < detector->previous_sample;
        if (peak_falling ||
            detector->candidate_age >= ECG_HR_PEAK_SEARCH_SAMPLES)
        {
            /* 退出候选跟踪 */
            detector->candidate_active = 0U;

            /* 确认候选峰（通过突出度、间隔等检查） */
            ECG_HR_ConfirmCandidate(detector);

            /*
             * 候选峰确认后立即检查重武装条件。
             * 如果信号已经回落到重武装电平以下，直接允许检测下一个峰。
             * 这加速了连续心跳的检测，减少不必要的延迟。
             */
            if (sample <= ECG_HR_RearmLevel(detector))
            {
                detector->armed = 1U;
            }
        }
    }
    /*
     * ========== 步骤⑧：重武装 ==========
     * armed = 0 且不在候选跟踪中：检查信号是否已回落到重武装电平以下。
     * 重武装电平 = threshold - 迟滞（迟滞 = max(峰峰值×20%, 30)）。
     *
     * 只有重武装后才能检测下一个 R 峰的上升沿。
     * 这防止了 R 峰顶部的小凹陷或 T 波被误检为独立心跳。
     */
    else if (detector->armed == 0U)
    {
        if (sample <= ECG_HR_RearmLevel(detector))
        {
            detector->armed = 1U;
        }
    }
    /*
     * ========== 步骤⑨：上升沿检测 ==========
     * armed = 1，不在候选跟踪中，且 previous_sample 可用时：
     * 检测信号是否从低于阈值（previous_sample < threshold）穿越到
     * 高于或等于阈值（sample >= threshold）。
     *
     * 条件满足时：
     * - armed 置 0，防止重复触发
     * - 启动候选跟踪（candidate_active = 1）
     * - 记录当前样本作为候选峰的初始最高点
     * - 候选年龄归零
     */
    else if (detector->previous_ready != 0U &&
             detector->previous_sample < detector->threshold &&
             sample >= detector->threshold)
    {
        /* 立即闭锁，防止同一次上升沿重复触发 */
        detector->armed = 0U;

        /* 启动候选峰跟踪 */
        detector->candidate_active = 1U;
        detector->candidate_peak_value = sample;
        detector->candidate_peak_sample = current_sample;
        detector->candidate_age = 0U;
    }

    /*
     * ========== 步骤⑩：保存当前样本供下次比较 ==========
     * 更新 previous_sample 为当前样本值，并标记 previous_ready。
     * 这为下一次 ECG_HR_Process 调用的上升沿检测和下降检测
     * 提供了正确的参考值。
     */
    detector->previous_sample = sample;
    detector->previous_ready = 1U;

    /* 返回当前保存的心率值（BPM） */
    return detector->heart_rate;
}

/*
 * ECG_HR_GetValue —— 只读获取当前心率值
 *
 * 作用：不推进状态机，只读取 detector 中保存的最新心率。
 * 在需要显示心率但不希望影响检测逻辑时使用。
 *
 * 参数 detector：检测器指针。NULL 时返回 0。
 * 返回值：当前心率（BPM），0 表示尚无有效结果。
 */
uint16_t ECG_HR_GetValue(const ECG_HRDetector *detector)
{
    return detector == 0 ? 0U : detector->heart_rate;
}
