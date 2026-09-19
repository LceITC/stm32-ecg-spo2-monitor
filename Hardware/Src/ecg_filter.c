/*
 * ============================================================================
 *  ecg_filter.c —— ECG 信号双级滤波器实现
 *
 *  滤波器特性：
 *    第一级：y[n] = (x[n] + y[n-1]) / 2
 *       - 一阶 IIR 低通，alpha = 0.5
 *       - 传递函数：H(z) = 0.5 / (1 - 0.5*z^(-1))
 *       - -3dB 截止频率 ≈ 0.5 * fs / (2π * 0.5) ≈ 16 Hz（@100Hz）
 *       - 对 50 Hz 工频干扰的衰减约 14 dB
 *
 *    第二级：z[n] = (y[n] + y[n-1]) / 2
 *       - 两点滑动平均（FIR）
 *       - 传递函数：H(z) = 0.5 * (1 + z^(-1))
 *       - 零点在 z = -1（奈奎斯特频率），对高频噪声有良好抑制
 *
 *  首样本处理：
 *    滤波器启动时（initialized == 0），第一个样本不经过滤波直接输出，
 *    同时将所有历史值初始化为该样本。这避免了从 0 启动时产生的
 *    大幅瞬态过冲，使滤波器输出从第一个样本开始就处于合理范围。
 *
 *  使用方式：
 *    1. 初始化：ECG_Filter_Init(&filter)
 *    2. 每个 100 Hz 样本：filtered = ECG_Filter_Process(&filter, raw)
 *    3. 来源切换时重新 Init，防止旧通道基线污染新通道
 * ============================================================================
 */

#include "ecg_filter.h"

/*
 * ECG_Filter_Init —— 清空滤波器历史状态
 *
 * 作用：将 lowpass_previous、moving_average_previous 和 filtered
 * 全部清零，initialized 置 0。下次 Process 调用时首样本将直接
 * 输出并建立历史。
 *
 * 注意：清零后第一个 Process 调用输出的值等于 raw_sample（未滤波），
 * 调用者可能需要忽略这个未滤波值，或确保首样本不影响后续判断。
 */
void ECG_Filter_Init(ECG_Filter *filter)
{
    /* 空指针保护 */
    if (filter == 0)
    {
        return;
    }

    /* 所有跨样本历史清零 */
    filter->lowpass_previous = 0U;
    filter->moving_average_previous = 0U;
    filter->filtered = 0U;

    /* initialized = 0 表示下次 Process 调用将执行初始化 */
    filter->initialized = 0U;
}

/*
 * ECG_Filter_Process —— 对单个原始样本执行双级滤波
 *
 * 执行步骤：
 *   ① 空指针检查：filter 为 NULL 时退化为直通返回
 *   ② 首样本处理：initialized == 0 时，直接建立历史后返回 raw_sample
 *   ③ 一级低通：lowpass = (raw_sample + lowpass_previous) / 2
 *      注意：使用整数除法，结果自动截断。
 *      对于 12 位 ADC 值（0~4095），精度损失可忽略。
 *   ④ 两点平均：filtered = (lowpass + moving_average_previous) / 2
 *      注意：moving_average_previous 保存的是上一次的 lowpass 值，
 *      不是上一次的 filtered 值。这确保了两点平均操作的是相邻的
 *      原始低通输出，而不是已经平均过的值。
 *   ⑤ 更新历史：lowpass_previous = lowpass（供下次低通使用）
 *      moving_average_previous = lowpass（供下次两点平均使用）
 *   ⑥ 返回 filter->filtered
 *
 * 参数 filter：滤波器指针。NULL 时直通返回 raw_sample。
 *     raw_sample：ADC 原始 12 位采样值。
 * 返回值：双级滤波后的 16 位整数值。
 */
uint16_t ECG_Filter_Process(ECG_Filter *filter, uint16_t raw_sample)
{
    /*
     * lowpass：当前样本经过一级低通后的中间结果。
     * 首样本处理时设为 raw_sample。
     * 正式滤波时 = (raw_sample + lowpass_previous) / 2。
     */
    uint16_t lowpass;

    /* ========== 步骤①：空指针保护 ========== */
    if (filter == 0)
    {
        return raw_sample;
    }

    /*
     * ========== 步骤②：首样本处理 ==========
     * 首样本直接建立所有历史值，输出 raw_sample。
     * 避免滤波器从 0 启动产生较大瞬态（例如首样本=2000时，
     * 如果不初始化，滤波输出 = (2000 + 0)/2 = 1000，误差巨大）。
     */
    if (filter->initialized == 0U)
    {
        filter->lowpass_previous = raw_sample;
        filter->moving_average_previous = raw_sample;
        filter->filtered = raw_sample;
        filter->initialized = 1U;
        return raw_sample;
    }

    /*
     * ========== 步骤③：一级低通 ==========
     * y[n] = 0.5 * x[n] + 0.5 * y[n-1]
     * 等效于 (x[n] + y[n-1]) >> 1
     */
    lowpass = (uint16_t)((raw_sample + filter->lowpass_previous) / 2U);

    /*
     * ========== 步骤④：两点滑动平均 ==========
     * z[n] = (y[n] + y[n-1]) / 2
     * moving_average_previous 保存的是上一次的 lowpass（即 y[n-1]）
     */
    filter->filtered =
        (uint16_t)((lowpass + filter->moving_average_previous) / 2U);

    /*
     * ========== 步骤⑤：更新历史值 ==========
     * 两个历史值都更新为当前 lowpass，供下一次调用使用。
     */
    filter->lowpass_previous = lowpass;
    filter->moving_average_previous = lowpass;

    /* ========== 步骤⑥：返回双级滤波结果 ========== */
    return filter->filtered;
}
