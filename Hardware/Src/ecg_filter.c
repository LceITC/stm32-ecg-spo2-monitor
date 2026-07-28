#include "ecg_filter.h"

/* 显式初始化便于后续重复开始一次新的 ECG 测量。 */
void ECG_Filter_Init(ECG_Filter *filter)
{
    if (filter == 0)
    {
        return;
    }

    filter->lowpass_previous = 0U;
    filter->moving_average_previous = 0U;
    filter->filtered = 0U;
    filter->initialized = 0U;
}

uint16_t ECG_Filter_Process(ECG_Filter *filter, uint16_t raw_sample)
{
    uint16_t lowpass;

    if (filter == 0)
    {
        return raw_sample;
    }

    /* 首样本直接建立历史值，避免滤波器从 0 启动产生较大瞬态。 */
    if (filter->initialized == 0U)
    {
        filter->lowpass_previous = raw_sample;
        filter->moving_average_previous = raw_sample;
        filter->filtered = raw_sample;
        filter->initialized = 1U;
        return raw_sample;
    }

    /* 一阶低通：y[n] = 0.5*x[n] + 0.5*y[n-1]。 */
    lowpass = (uint16_t)((raw_sample + filter->lowpass_previous) / 2U);

    /* 对相邻两个低通结果再求平均，进一步抑制单点抖动。 */
    filter->filtered =
        (uint16_t)((lowpass + filter->moving_average_previous) / 2U);
    filter->lowpass_previous = lowpass;
    filter->moving_average_previous = lowpass;

    return filter->filtered;
}
