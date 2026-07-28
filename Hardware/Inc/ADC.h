#ifndef ECG_ADC_H
#define ECG_ADC_H

/* ADC1 初始化结果，用于区分两种硬件校准超时。 */
typedef enum
{
    ADC_ECG_INIT_OK = 0,
    ADC_ECG_INIT_RESET_CALIBRATION_TIMEOUT,
    ADC_ECG_INIT_CALIBRATION_TIMEOUT
} ADC_ECG_InitStatus;

/* 配置 PA0/ADC1_CH0，由 TIM2_CC2 外部事件触发单次转换。 */
typedef enum
{
    ADC_ECG_SOURCE_AD8232 = 0,
    ADC_ECG_SOURCE_DIRECT
} ADC_ECG_Source;

ADC_ECG_InitStatus ADC_ECG_Init(ADC_ECG_Source source);
void ADC_ECG_SelectSource(ADC_ECG_Source source);

#endif
