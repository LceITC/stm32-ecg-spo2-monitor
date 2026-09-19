#ifndef ECG_ACQUISITION_H
#define ECG_ACQUISITION_H

#include "stm32f10x.h"
#include <stdbool.h>

/*
 * ============================================================================
 *  ECG 采集链路 —— 数据类型与函数声明
 *
 *  采集链路构成（按初始化顺序）：
 *    ① DMA1 Channel1：将 ADC1 转换结果自动搬运到内存变量
 *    ② ADC1：单通道、外部触发（TIM2_CC2）、非连续转换
 *    ③ TIM2：100 Hz PWM 输出，通过内部 CC2 事件触发 ADC 转换
 *
 *  数据流：
 *    TIM2(100Hz) → ADC1(单次转换) → DMA1(自动搬运) → 软件队列 → 主循环
 *
 *  故障诊断：
 *    200ms 无样本时按顺序检查：TIM2 是否运行 → DMA 是否挂起 → ADC 触发链路
 * ============================================================================
 */

/*
 * ECG_AcquisitionInitStatus —— 采集链路初始化状态枚举
 *
 * ECG_ACQUISITION_INIT_OK = 0：DMA、ADC 和 TIM2 均启动成功。
 * ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT：ADC 复位校准超时。
 *   可能原因：ADC 未使能、时钟异常或硬件故障。
 * ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT：ADC 正式校准超时。
 *   可能原因：ADC 硬件异常或外部干扰导致校准无法完成。
 */
typedef enum
{
    ECG_ACQUISITION_INIT_OK = 0,
    ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT,
    ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT
} ECG_AcquisitionInitStatus;

/*
 * ECG_AcquisitionFault —— 运行期采样故障枚举
 *
 * 诊断优先级：TIM2 → DMA → ADC/触发链路
 * 每次故障只上报一次，同一次持续故障不重复报告。
 *
 * ECG_ACQUISITION_FAULT_NONE = 0：当前没有新的采样故障。
 * ECG_ACQUISITION_FAULT_TIM2：TIM2 未产生 CC2 事件（定时器未启动或配置错误）。
 * ECG_ACQUISITION_FAULT_DMA：DMA TC 标志未被 ISR 清除（ISR 未触发或挂起）。
 * ECG_ACQUISITION_FAULT_TRIGGER：定时器运行但 ADC/DMA 链路没有产生样本。
 */
typedef enum
{
    ECG_ACQUISITION_FAULT_NONE = 0,
    ECG_ACQUISITION_FAULT_TIM2,
    ECG_ACQUISITION_FAULT_DMA,
    ECG_ACQUISITION_FAULT_TRIGGER
} ECG_AcquisitionFault;

/*
 * ECG_Source —— ECG 输入源选择枚举
 *
 * 与 ADC 模块的底层通道枚举（ADC_ECG_Source）相互独立。
 * 上层代码（main.c）使用此枚举选择来源，采集层内部再转换为
 * ADC 通道配置。
 *
 * ECG_SOURCE_AD8232 = 0：PA0/ADC1_CH0，AD8232 模拟输出。
 * ECG_SOURCE_DIRECT：PA2/ADC1_CH2，信号发生器直测输入。
 */
typedef enum
{
    ECG_SOURCE_AD8232 = 0,
    ECG_SOURCE_DIRECT
} ECG_Source;

/*
 * ECG_Acquisition_Init —— 初始化 100 Hz ECG 采集链路
 *
 * 初始化顺序（严格）：DMA → ADC → TIM2。
 * DMA 必须先于 ADC 启用，避免 ADC 首次转换完成时没有可用的搬运目标。
 * TIM2 最后启动，确保触发开始前 ADC 和 DMA 均已准备好。
 *
 * 返回值：ECG_AcquisitionInitStatus，指示初始化结果。
 */
ECG_AcquisitionInitStatus ECG_Acquisition_Init(void);

/*
 * ECG_Acquisition_SelectSource —— 切换 ECG 输入源
 *
 * 执行步骤：
 *   ① 停止 TIM2 触发
 *   ② 禁用 ADC 外部触发
 *   ③ 等待 20μs 确保当前转换结束
 *   ④ 清空 DMA 软件队列和遗留标志
 *   ⑤ 切换 ADC 规则通道
 *   ⑥ 恢复 ADC 外部触发和 TIM2
 *
 * 参数 source：目标 ECG 源（AD8232 或 DIRECT）。
 * 返回值：true 表示切换成功，false 表示链路未就绪或参数无效。
 */
bool ECG_Acquisition_SelectSource(ECG_Source source);

/*
 * ECG_Acquisition_TakeSample —— 从 DMA 软件队列取出一个 ECG 样本
 *
 * 每个 100 Hz 样本由 DMA ISR 放入软件环形队列。
 * 此函数从队列头部取出一个尚未消费的样本。
 * 同步刷新采样看门狗时间戳，允许后续新故障再次上报。
 *
 * 参数 sample：接收 12 位 ADC 结果的无符号 16 位指针。
 * 返回值：true 表示成功取到一个新样本；false 表示队列为空或指针无效。
 */
bool ECG_Acquisition_TakeSample(uint16_t *sample);

/*
 * ECG_Acquisition_PollFault —— 非阻塞检查 200ms 采样超时
 *
 * 如果上次成功取样本后超过 200ms 仍无新数据，按优先级诊断：
 *   ① TIM2 CC2 标志 → 定时器未运行
 *   ② DMA TC 标志挂起 → DMA 中断链路异常
 *   ③ 以上均正常 → ADC 外部触发或 DMA 请求链路异常
 *
 * 同一次持续故障只向上层报告一次（fault_reported 机制）。
 *
 * 返回值：ECG_AcquisitionFault，ECG_ACQUISITION_FAULT_NONE 表示正常。
 */
ECG_AcquisitionFault ECG_Acquisition_PollFault(void);

#endif
