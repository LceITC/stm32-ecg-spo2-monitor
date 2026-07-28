#include "MAX30102.h"
#include "delay.h"
#include "ecg_acquisition.h"
#include "ecg_filter.h"
#include "finger_detector.h"
#include "max30102_spo2.h"
#include "monitor.h"
#include "serial.h"
#include "stm32f10x.h"

/* MAX30102 上电时允许短暂无应答，因此最多重试三次。 */
#define MAX30102_INIT_RETRIES 3U

int main(void)
{
    FingerDetector detector;
    MAX30102_Sample max_sample = {0U, 0U};
    MAX30102_InitStatus max_status = MAX30102_INIT_I2C_ERROR;
    ECG_AcquisitionInitStatus ecg_status;
    ECG_AcquisitionFault ecg_fault;
    ECG_Filter ecg_filter;
    ECG_Source requested_source;
    Serial_EcgSource serial_source;
    bool max_sample_valid;
    uint8_t part_id = 0U;
    uint8_t max_available;
    uint8_t max_init_attempt;
    uint8_t finger_present;
    uint8_t latest_spo2 = 0U;
    uint16_t ecg_sample;
    uint16_t latest_ecg = 0U;

    /* 先建立显示和调试接口，再启动传感器与定时采集。 */
    Monitor_Init();
    My_Serial_Init();
    Delay_Init();
    FingerDetector_Init(&detector);
    MAX30102_SpO2_Init();
    ECG_Filter_Init(&ecg_filter);

    /* MAX30102 使用 PB10/PB11 软件 I2C，PB12 为低有效中断输入。 */
    MAX30102_GPIO_Init();
    for (max_init_attempt = 0U; max_init_attempt < MAX30102_INIT_RETRIES;
         ++max_init_attempt)
    {
        max_status = MAX30102_Init(&part_id);
        if (max_status != MAX30102_INIT_I2C_ERROR)
        {
            break;
        }
        Delay(20U);
    }

    max_available = max_status == MAX30102_INIT_OK ? 1U : 0U;
    Monitor_ShowMaxInitStatus(max_status);

    ecg_status = ECG_Acquisition_Init();
    Monitor_ShowEcgInitStatus(ecg_status);
    Monitor_ShowEcgSource(ECG_SOURCE_AD8232);

    while (1)
    {
        if (Serial_PollEcgSource(&serial_source))
        {
            requested_source = serial_source == SERIAL_ECG_SOURCE_DIRECT
                                   ? ECG_SOURCE_DIRECT
                                   : ECG_SOURCE_AD8232;
            if (ECG_Acquisition_SelectSource(requested_source))
            {
                ECG_Filter_Init(&ecg_filter);
                latest_ecg = 0U;
                Monitor_ShowEcgSource(requested_source);
            }
        }

        /*
         * 每个 MAX30102 新样本只处理和发送一次，串口节奏因此与
         * 传感器的 100 Hz 采样节奏一致。
         */
        if (max_available != 0U && MAX30102_DataReady())
        {
            /*
             * 将 FIFO 中积压的样本逐个送入检测和血氧算法，避免丢点后
             * 破坏 100 Hz 时间序列。每成功读取一个样本只发送一帧 CSV。
             */
            while (MAX30102_ReadSample(&max_sample))
            {
                finger_present = FingerDetector_Process(&detector, &max_sample,
                                                        &max_sample_valid);
                MAX30102_SpO2_ProcessSample(max_sample.red, max_sample.ir,
                                            finger_present != 0U &&
                                                max_sample_valid);
                latest_spo2 = MAX30102_SpO2_GetValue();
                Monitor_UpdateMaxSample(&max_sample, latest_spo2,
                                        finger_present);

                Serial_SendMonitorFrame(0U, latest_spo2, 0U, max_sample.ir);
                Monitor_RecordTxFrame();
            }
        }

        /* ECG 采集代码保留供后续使用，本阶段不进入串口协议。 */
        if (ECG_Acquisition_TakeSample(&ecg_sample))
        {
            latest_ecg = ECG_Filter_Process(&ecg_filter, ecg_sample);
            Monitor_UpdateEcgSample(latest_ecg);
        }

        /* 采样停顿超过 200 ms 时，将故障类型交给 OLED 显示模块。 */
        ecg_fault = ECG_Acquisition_PollFault();
        if (ecg_fault != ECG_ACQUISITION_FAULT_NONE)
        {
            Monitor_ShowEcgFault(ecg_fault);
        }
    }
}
