#ifndef ECG_ADC_H
#define ECG_ADC_H

/* ADC1初始化结果，用于让上层准确显示校准失败发生在哪个阶段。 */
typedef enum
{
    ADC_ECG_INIT_OK = 0,                     /* ADC初始化和校准全部成功。 */
    ADC_ECG_INIT_RESET_CALIBRATION_TIMEOUT,  /* 复位校准寄存器超时。 */
    ADC_ECG_INIT_CALIBRATION_TIMEOUT         /* 正式校准过程超时。 */
} ADC_ECG_InitStatus;

/*
 * PA0/ADC1_CH0 用于 AD8232，PA2/ADC1_CH2 用于信号发生器直测。
 * 两个来源均由 TIM2_CC2 外部事件触发单次转换。
 */
typedef enum
{
    ADC_ECG_SOURCE_AD8232 = 0, /* 采集PA0上的AD8232输出。 */
    ADC_ECG_SOURCE_DIRECT      /* 采集PA2上的信号发生器输出。 */
} ADC_ECG_Source;

/* 配置PA0/PA2、ADC1、TIM2_CC2外部触发和ADC校准，返回初始化状态。 */
ADC_ECG_InitStatus ADC_ECG_Init(ADC_ECG_Source source);

/* 将ADC1规则序列的第1个通道切换为source指定的ECG输入。 */
void ADC_ECG_SelectSource(ADC_ECG_Source source);

#endif
