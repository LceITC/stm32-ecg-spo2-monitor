#ifndef ECG_HR_H
#define ECG_HR_H

#include <stdint.h>

/*
 * ============================================================================
 *  心率检测算法 —— 配置参数
 * ============================================================================
 * 所有时间相关参数基于 100 Hz 采样率。
 * 幅度相关参数基于原始 ADC 值（12位，范围 0~4095）。
 *
 * 算法工作流程：
 *   ① 每 2 秒（200 样本）计算一次自适应阈值，
 *      threshold = window_min + 峰峰值 × THRESHOLD_PERCENT / 100
 *   ② 信号从低于阈值 → 高于阈值时，开始跟踪峰顶
 *   ③ 在最多 PEAK_SEARCH_SAMPLES 个样本内寻找局部最高点
 *   ④ 信号开始下降或超时后，确认候选峰：
 *      - 峰顶必须高出阈值至少一个 PEAK_PROMINENCE 幅度
 *      - 与上一个候选峰间隔必须 ≥ MIN_INTERVAL_SAMPLES
 *   ⑤ 有效 RR 间隔在 [MIN, MAX] 范围内，且偏离当前代表间隔
 *      不超过 RR_TOLERANCE_PERCENT 时，才更新心率
 * ============================================================================
 */

/* ---------- 时间相关参数 ---------- */

/*
 * ECG_HR_SAMPLE_RATE_HZ：采样率，100 Hz。
 * 所有"样本数"单位都基于此值换算。60 BPM 对应 100 样本间隔。
 */
#define ECG_HR_SAMPLE_RATE_HZ 100U

/*
 * ECG_HR_WINDOW_SAMPLES：自适应阈值窗口包含的样本数，200 点 = 2 秒。
 * 窗口收集 200 个连续样本后，根据 window_max - window_min 重新计算阈值。
 * 窗口足够长（2秒）才能保证至少包含一个完整 QRS 波群；
 * 窗口不宜过长（>3秒），否则对信号幅度变化的响应太慢。
 */
#define ECG_HR_WINDOW_SAMPLES 200U

/*
 * ECG_HR_MIN_PEAK_TO_PEAK：有效窗口所需最小峰峰值。
 * 如果 2 秒内信号最大值与最小值之差小于 100 ADC 码，认为信号过弱或
 * 传感器脱落，不启用心率检测。防止静音噪声被放大为虚假心率。
 */
#define ECG_HR_MIN_PEAK_TO_PEAK 100U

/* ---------- 阈值与迟滞参数 ---------- */

/*
 * ECG_HR_THRESHOLD_PERCENT：阈值位于窗口幅度（peak_to_peak）的百分比位置。
 * threshold = window_min + 峰峰值 × 80% 。
 * 例如 window_min = 1500, 峰峰值 = 500，则 threshold = 1500 + 400 = 1900。
 * R 峰必须超过 1900 才会被跟踪。80% 的位置对大多数 ECG 信号能有效
 * 避开基线噪声和 T 波，只捕获 R 波上升沿。
 */
#define ECG_HR_THRESHOLD_PERCENT 80U

/*
 * ECG_HR_REARM_PERCENT / ECG_HR_REARM_MIN：回落迟滞。
 * R 峰检测完成后，信号必须回落到 threshold - 迟滞 以下，才允许检测
 * 下一个 R 峰。迟滞 = max(峰峰值 × 20%, 30 ADC码)。
 * 大信号用比例迟滞（抑制 R 峰顶部小凹陷重复触发），
 * 小信号用最小迟滞（保证能正常回落重武装）。
 */
#define ECG_HR_REARM_PERCENT 20U
#define ECG_HR_REARM_MIN 30U

/* ---------- 候选峰确认参数 ---------- */

/*
 * ECG_HR_PEAK_SEARCH_SAMPLES：候选峰最长跟踪窗口，25 样本 = 250 ms。
 * 当信号上穿阈值后，算法会持续跟踪局部最高点最多 250 ms。
 * 这段时间足够覆盖 QRS 波群的上升支和顶峰（典型 ~100-120 ms）。
 * 超时后无论是否检测到下降沿，都强制确认候选峰。
 */
#define ECG_HR_PEAK_SEARCH_SAMPLES 25U

/*
 * ECG_HR_PEAK_PROMINENCE_PERCENT / ECG_HR_PEAK_PROMINENCE_MIN：
 * 峰顶高出阈值的最小幅度。确认候选峰时，候选峰顶值必须满足：
 *   峰顶值 - threshold ≥ max(峰峰值 × 10%, 8 ADC码)
 * 该条件用于过滤阈值附近的小幅度噪声波动，确保只有显著的 R 波
 * 上升才能被确认为心跳。
 */
#define ECG_HR_PEAK_PROMINENCE_PERCENT 10U
#define ECG_HR_PEAK_PROMINENCE_MIN 8U

/* ---------- RR 间隔有效性参数 ---------- */

/*
 * ECG_HR_MIN_INTERVAL_SAMPLES / ECG_HR_MAX_INTERVAL_SAMPLES：
 * 有效 RR 间隔的上下限，单位样本数。
 * 33 样本 = 330 ms → 上限 ~182 BPM
 * 150 样本 = 1500 ms → 下限 ~40 BPM
 * 超出此范围的间隔被直接丢弃，不更新心率。
 */
#define ECG_HR_MIN_INTERVAL_SAMPLES 33U
#define ECG_HR_MAX_INTERVAL_SAMPLES 150U

/*
 * ECG_HR_RR_TOLERANCE_PERCENT：RR 间隔一致性容忍度。
 * 当算法已锁定节律（interval_count > 0），新间隔必须在当前代表间隔的
 * ±35% 以内。例如当前代表间隔 = 100 样本，新间隔必须在 65~135 之间。
 * 该保护主要拦截真实 R 峰之间的高 T 波误检，以及漏检后形成的倍长间隔。
 */
#define ECG_HR_RR_TOLERANCE_PERCENT 35U

/* ---------- 保护与存储参数 ---------- */

/*
 * ECG_HR_ADC_SATURATION_LOW / ECG_HR_ADC_SATURATION_HIGH：
 * ADC 饱和边界。12位 ADC 满量程 0~4095，但接近电源轨的信号已经失真。
 * 为 R 峰保留约 100 ADC 码的上下摆幅余量。超出此范围时立即丢弃
 * 当前节律并重新收集 2 秒窗口。
 */
#define ECG_HR_ADC_SATURATION_LOW 100U
#define ECG_HR_ADC_SATURATION_HIGH 3995U

/*
 * ECG_HR_INTERVAL_HISTORY_SIZE：RR 间隔环形历史长度，3 个槽位。
 * 用于中值/平均滤波抗毛刺，同时满足 3 点中值计算的需求。
 */
#define ECG_HR_INTERVAL_HISTORY_SIZE 3U

/*
 * ============================================================================
 *  心率检测器 —— 数据结构
 * ============================================================================
 * 保存检峰状态机的全部状态变量。
 * sample_index 使用 uint32_t 无符号减法计算间隔，即使约 497 天后
 * 发生回绕，只要相邻 R 峰间隔远小于 2^32，RR 计算仍然正确。
 */
typedef struct
{
    /*
     * ---- 时间轴相关 ----
     * sample_index：自 ECG_HR_Init 起累计的 100 Hz 样本序号，
     *   每个 ECG_HR_Process 调用递增 1。用于测量 RR 间隔。
     * last_peak_sample：最近一个被确认为有效 R 峰的样本序号。
     * last_candidate_sample：最近一个通过候选检查的候选峰的样本序号，
     *   用于候选间隔去抖（间隔过近的候选峰被拒绝）。
     * candidate_peak_sample：当前正在跟踪的候选峰的最高点样本序号。
     */
    uint32_t sample_index;
    uint32_t last_peak_sample;
    uint32_t last_candidate_sample;
    uint32_t candidate_peak_sample;

    /*
     * ---- 阈值相关 ----
     * window_max / window_min：当前 2 秒窗口内信号的最大/最小值。
     * window_peak_to_peak：最近一个完整窗口的峰峰值（max - min），
     *   用于百分比阈值和百分比迟滞计算。
     * window_count：当前窗口已经收集的样本数，达到 200 时窗口结束。
     * threshold：由 window_min + 峰峰值 × 80% 计算的自适应阈值。
     */
    uint16_t window_max;
    uint16_t window_min;
    uint16_t window_peak_to_peak;
    uint16_t window_count;
    uint16_t threshold;

    /*
     * ---- 样本历史 ----
     * previous_sample：上一次 ECG_HR_Process 的输入，与当前样本
     *   比较以检测信号上升沿穿越阈值。
     * candidate_peak_value：当前候选峰跟踪期间观察到的最高 ADC 值，
     *   用于确认候选峰时检查突出度是否达标。
     */
    uint16_t previous_sample;
    uint16_t candidate_peak_value;

    /*
     * ---- RR 间隔与心率输出 ----
     * intervals[]：最近 ECG_HR_INTERVAL_HISTORY_SIZE 个有效 RR 间隔
     *   的环形缓存。用于中值/平均滤波。
     * interval_count / interval_write_index：环形缓存的状态变量。
     * heart_rate：最近计算的有效心率（BPM），0 表示当前无效。
     */
    uint16_t intervals[ECG_HR_INTERVAL_HISTORY_SIZE];
    uint16_t heart_rate;
    uint8_t interval_count;
    uint8_t interval_write_index;

    /*
     * ---- 状态标志 ----
     * threshold_ready：1 表示已经完成至少一个 2 秒窗口，阈值已就绪。
     * previous_ready：1 表示 previous_sample 已保存有效值，可用于
     *   上升沿比较。首样本或饱和复位后为 0。
     * armed：1 表示信号已从上一个 R 峰充分回落（低于重武装电平），
     *   允许检测下一个 R 峰的上升沿。
     * candidate_active：1 表示正在跟踪一次阈值上穿事件，寻找局部最
     *   高点。此期间忽略新的阈值上穿。
     */
    uint8_t threshold_ready;
    uint8_t previous_ready;
    uint8_t armed;
    uint8_t candidate_active;

    /*
     * ---- 候选跟踪状态 ----
     * candidate_age：当前候选峰已经跟踪的样本数，达到 PEAK_SEARCH_
     *   SAMPLES 时强制结束跟踪。
     */
    uint8_t candidate_age;

    /*
     * ---- 节律锁定标志 ----
     * has_peak_reference：1 表示已有至少一个 RR 计时基准峰。
     *   首个 R 峰只建立基准，不计算 RR 间隔。
     * has_candidate_reference：1 表示已有至少一个候选峰通过了候选
     *   间隔检查。首个候选峰只建立参考点。
     */
    uint8_t has_peak_reference;
    uint8_t has_candidate_reference;
} ECG_HRDetector;

/*
 * ============================================================================
 *  函数声明
 * ============================================================================
 */

/*
 * ECG_HR_Init：将检测器的所有状态变量恢复为初始值。
 * 在系统上电或 ECG 来源切换时调用，确保从干净的 2 秒窗口重新开始。
 * 参数 detector 为 NULL 时直接返回。
 */
void ECG_HR_Init(ECG_HRDetector *detector);

/*
 * ECG_HR_Process：核心处理函数，每个 100 Hz 滤波 ECG 样本调用一次。
 * 内部状态机执行：饱和检查 → 超时检查 → 窗口更新 → 候选峰跟踪 →
 * 重武装 → 上升沿检测 → 候选确认 → RR 间隔验证 → 心率更新。
 * 返回当前保存的 heart_rate（BPM），0 表示尚未稳定或信号无效。
 */
uint16_t ECG_HR_Process(ECG_HRDetector *detector, uint16_t sample);

/*
 * ECG_HR_GetValue：只读接口，返回 detector 中保存的最新心率值。
 * 不推进状态机。空指针或无有效结果时返回 0。
 */
uint16_t ECG_HR_GetValue(const ECG_HRDetector *detector);

#endif
