#ifndef ECG_FILTER_H
#define ECG_FILTER_H

#include "stm32f10x.h"

/* 保存低通和两点滑动平均的历史状态。 */
typedef struct
{
    uint16_t lowpass_previous;
    uint16_t moving_average_previous;
    uint16_t filtered;
    uint8_t initialized;
} ECG_Filter;

/* 清空滤波器状态；第一个样本将直接作为初值。 */
void ECG_Filter_Init(ECG_Filter *filter);

/* 执行 alpha=0.5 一阶低通，再执行两点滑动平均。 */
uint16_t ECG_Filter_Process(ECG_Filter *filter, uint16_t raw_sample);

#endif
