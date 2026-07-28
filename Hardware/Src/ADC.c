#include "ADC.h"
#include "delay.h"
#include "stm32f10x.h"

#define ADC_CALIBRATION_TIMEOUT_MS 20U

static uint8_t ADC_ECG_Channel(ADC_ECG_Source source)
{
    return source == ADC_ECG_SOURCE_DIRECT ? ADC_Channel_1 : ADC_Channel_0;
}

void ADC_ECG_SelectSource(ADC_ECG_Source source)
{
    ADC_RegularChannelConfig(ADC1, ADC_ECG_Channel(source), 1U,
                             ADC_SampleTime_55Cycles5);
}

/*
 * AD8232 OUTPUT 接 PA0。ADC 不使用连续模式，每次转换均由
 * TIM2_CC2 的 100 Hz 事件触发，结果由 DMA1 Channel1 搬运。
 */
ADC_ECG_InitStatus ADC_ECG_Init(ADC_ECG_Source source)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;
    uint32_t timeout_start;

    /* ADC 时钟为 72 MHz / 6 = 12 MHz，满足 STM32F103 ADC 规格。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* 模拟输入模式关闭数字输入缓冲，减少 PA0 上的额外干扰。 */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* 单通道、非连续转换，外部触发源固定为 TIM2_CC2。 */
    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1U;
    ADC_Init(ADC1, &adc);
    ADC_ECG_SelectSource(source);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    /* 校准过程增加超时保护，避免硬件异常时程序永久阻塞。 */
    ADC_ResetCalibration(ADC1);
    timeout_start = GetTick();
    while (ADC_GetResetCalibrationStatus(ADC1) == SET)
    {
        if ((uint32_t)(GetTick() - timeout_start) >= ADC_CALIBRATION_TIMEOUT_MS)
        {
            return ADC_ECG_INIT_RESET_CALIBRATION_TIMEOUT;
        }
    }

    ADC_StartCalibration(ADC1);
    timeout_start = GetTick();
    while (ADC_GetCalibrationStatus(ADC1) == SET)
    {
        if ((uint32_t)(GetTick() - timeout_start) >= ADC_CALIBRATION_TIMEOUT_MS)
        {
            return ADC_ECG_INIT_CALIBRATION_TIMEOUT;
        }
    }

    /* 校准完成后才允许外部事件启动正式转换。 */
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    return ADC_ECG_INIT_OK;
}
