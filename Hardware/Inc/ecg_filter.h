#ifndef ECG_FILTER_H
#define ECG_FILTER_H

#include <stdint.h>

/*
 * ============================================================================
 *  ECG 滤波器 —— 数据结构与函数声明
 *
 *  滤波器结构：两级串联
 *    第一级：alpha=0.5 的一阶 IIR 低通滤波器
 *       y[n] = (x[n] + y[n-1]) / 2
 *       -3dB 截止频率约 16 Hz（@100Hz 采样率）
 *       用于抑制 50 Hz 工频干扰和高频肌电噪声
 *
 *    第二级：相邻低通输出的两点滑动平均
 *       z[n] = (y[n] + y[n-1]) / 2
 *       进一步抑制单点尖峰噪声
 *
 *  综合效果：
 *    - 群延迟约 1.5 个样本（15ms @100Hz），对心率计算影响可忽略
 *    - QRS 波群幅度衰减约 10-15%，但自适应阈值随之调整，不影响检峰
 *    - 有效滤除 50 Hz 工频及其谐波
 *
 *  滤波器初始化约定：
 *    首样本不滤波，直接输出并建立所有历史值。
 *    从第二个样本开始执行完整的双级滤波。
 * ============================================================================
 */

/*
 * ECG_Filter —— ECG 滤波器状态结构体
 *
 * lowpass_previous：上一次一级低通输出 y[n-1]，用于当前样本的
 *   低通计算。首样本后被初始化为 raw_sample。
 *
 * moving_average_previous：上一次的低通输出值，与当前低通输出
 *   求平均作为最终滤波结果。注意：这个值等于上一次的 lowpass_previous，
 *   而不是 filtered，确保滑动平均操作的是相邻的低通输出。
 *
 * filtered：最近一次完整两级滤波的结果。Init 时清零，首样本后
 *   设置为 raw_sample。用于外部读取当前滤波值。
 *
 * initialized：0 表示尚未收到第一个原始样本。此时 Process 函数
 *   会绕过滤波逻辑，直接输出 raw_sample 并建立历史。
 *   1 表示已经初始化完成，进行正式双级滤波。
 */
typedef struct
{
    uint16_t lowpass_previous;
    uint16_t moving_average_previous;
    uint16_t filtered;
    uint8_t initialized;
} ECG_Filter;

/*
 * ECG_Filter_Init —— 清空滤波器历史状态
 *
 * 作用：将所有跨样本历史清零，initialized 置 0。
 * 在系统上电或 ECG 来源切换时调用，防止旧通道的基线和滤波值
 * 污染新通道的第一个样本。
 *
 * 参数 filter：滤波器指针。NULL 时直接返回。
 * 返回值：无。
 */
void ECG_Filter_Init(ECG_Filter *filter);

/*
 * ECG_Filter_Process —— 对原始 ECG 样本执行双级滤波
 *
 * 作用：对 raw_sample 执行 alpha=0.5 一级低通和两点滑动平均。
 * 首样本直接输出并建立历史值。
 *
 * 参数 filter：滤波器指针。NULL 时退化为原样返回 raw_sample。
 *     raw_sample：ADC 原始 12 位采样值（0~4095）。
 * 返回值：滤波后的 16 位整数值。
 */
uint16_t ECG_Filter_Process(ECG_Filter *filter, uint16_t raw_sample);

#endif
