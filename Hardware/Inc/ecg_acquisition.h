#ifndef ECG_ACQUISITION_H
#define ECG_ACQUISITION_H

#include "stm32f10x.h"
#include <stdbool.h>

/* ECG 采集链路初始化状态，错误来源由 ADC 校准阶段决定。 */
typedef enum
{
    ECG_ACQUISITION_INIT_OK = 0,
    ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT,
    ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT
} ECG_AcquisitionInitStatus;

/* 连续 200 ms 没有样本时，用于定位定时器、DMA 或触发链路。 */
typedef enum
{
    ECG_ACQUISITION_FAULT_NONE = 0,
    ECG_ACQUISITION_FAULT_TIM2,
    ECG_ACQUISITION_FAULT_DMA,
    ECG_ACQUISITION_FAULT_TRIGGER
} ECG_AcquisitionFault;

typedef enum
{
    ECG_SOURCE_AD8232 = 0,
    ECG_SOURCE_DIRECT
} ECG_Source;

/* 按 DMA -> ADC -> TIM2 的顺序启动 100 Hz 采集链路。 */
ECG_AcquisitionInitStatus ECG_Acquisition_Init(void);
bool ECG_Acquisition_SelectSource(ECG_Source source);

/* 从 DMA 单样本缓存中取走一个尚未消费的新样本。 */
bool ECG_Acquisition_TakeSample(uint16_t *sample);

/* 非阻塞检查采样超时；同一次故障只报告一次。 */
ECG_AcquisitionFault ECG_Acquisition_PollFault(void);

#endif
