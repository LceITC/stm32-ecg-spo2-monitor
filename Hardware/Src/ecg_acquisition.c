#include "ecg_acquisition.h"
#include "ADC.h"
#include "DMA.h"
#include "TIM2.h"
#include "delay.h"

#define ECG_SAMPLE_TIMEOUT_MS 200U

static uint32_t last_sample_tick;
static uint8_t acquisition_available;
static uint8_t fault_reported;
static ECG_Source current_source;

/*
 * DMA 必须先于 ADC 启用，避免 ADC 首次转换完成时没有可用的搬运目标；
 * TIM2 最后启动，确保触发开始前 ADC 和 DMA 均已准备好。
 */
ECG_AcquisitionInitStatus ECG_Acquisition_Init(void)
{
    ADC_ECG_InitStatus adc_status;

    acquisition_available = 0U;
    fault_reported = 0U;

    DMA_ECG_Init();
    current_source = ECG_SOURCE_AD8232;
    adc_status = ADC_ECG_Init(ADC_ECG_SOURCE_AD8232);
    if (adc_status == ADC_ECG_INIT_RESET_CALIBRATION_TIMEOUT)
    {
        return ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT;
    }
    if (adc_status == ADC_ECG_INIT_CALIBRATION_TIMEOUT)
    {
        return ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT;
    }

    TIM2_ECG_Init();
    last_sample_tick = GetTick();
    acquisition_available = 1U;
    TIM2_ECG_Start();

    return ECG_ACQUISITION_INIT_OK;
}

bool ECG_Acquisition_SelectSource(ECG_Source source)
{
    ADC_ECG_Source adc_source;

    if (acquisition_available == 0U ||
        (source != ECG_SOURCE_AD8232 && source != ECG_SOURCE_DIRECT))
    {
        return false;
    }
    if (source == current_source)
    {
        return true;
    }

    adc_source = source == ECG_SOURCE_DIRECT ? ADC_ECG_SOURCE_DIRECT
                                             : ADC_ECG_SOURCE_AD8232;
    TIM2_ECG_Stop();
    ADC_ExternalTrigConvCmd(ADC1, DISABLE);
    DelayUs(20U);
    DMA_ECG_Reset();
    ADC_ECG_SelectSource(adc_source);
    current_source = source;
    last_sample_tick = GetTick();
    fault_reported = 0U;
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    TIM2_ECG_Start();
    return true;
}

/* 成功取样后刷新时间戳，并允许后续新的超时故障再次上报。 */
bool ECG_Acquisition_TakeSample(uint16_t *sample)
{
    if (acquisition_available == 0U || !DMA_ECG_TakeSample(sample))
    {
        return false;
    }

    last_sample_tick = GetTick();
    fault_reported = 0U;
    return true;
}

/*
 * 诊断顺序：
 * 1. TIM2 没有 CC2 标志，说明 100 Hz 时基没有运行；
 * 2. DMA TC 标志未被 ISR 清除，说明 DMA 中断链路异常；
 * 3. 否则多半是 ADC 外部触发或 DMA 请求链路异常。
 */
ECG_AcquisitionFault ECG_Acquisition_PollFault(void)
{
    if (acquisition_available == 0U || fault_reported != 0U ||
        (uint32_t)(GetTick() - last_sample_tick) < ECG_SAMPLE_TIMEOUT_MS)
    {
        return ECG_ACQUISITION_FAULT_NONE;
    }

    fault_reported = 1U;
    if (!TIM2_ECG_CompareEventSeen())
    {
        return ECG_ACQUISITION_FAULT_TIM2;
    }
    if (DMA_ECG_TransferCompletePending())
    {
        return ECG_ACQUISITION_FAULT_DMA;
    }
    return ECG_ACQUISITION_FAULT_TRIGGER;
}
