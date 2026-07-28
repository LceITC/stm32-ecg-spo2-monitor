#include "monitor.h"
#include "OLED.h"
#include "delay.h"

/* 设为 1 时使用 OLED 观察 RED、IR 和饱和状态，不改变串口协议。 */
#ifndef MAX30102_LOCAL_CALIBRATION
#define MAX30102_LOCAL_CALIBRATION 0
#endif

#define OLED_CAL_REFRESH_MS 250U
#define MAX30102_NEAR_SATURATION 260000U
#define UART_TX_FRAMES_PER_COUNT 100U

static uint8_t previous_finger_state;
static uint8_t previous_spo2;
static uint8_t tx_frame_accumulator;
static uint16_t tx_display_count;
static uint32_t calibration_last_update_tick;
static uint32_t ecg_last_update_tick;

/* 第四行只在状态变化时重写，避免循环刷新导致 OLED 闪烁。 */
static void Monitor_ShowFingerState(uint8_t present)
{
    OLED_ShowString(4U, 1U, present ? "Finger: ON " : "Finger: OFF");
}

#if !MAX30102_LOCAL_CALIBRATION
static void Monitor_ShowSpO2(uint8_t spo2)
{
    if (spo2 == 0U)
    {
        OLED_ShowString(3U, 1U, "SpO2: -- %     ");
    }
    else
    {
        OLED_ShowString(3U, 1U, "SpO2: ");
        if (spo2 < 100U)
        {
            OLED_ShowString(3U, 7U, " ");
            OLED_ShowNum(3U, 8U, spo2, 2U);
        }
        else
        {
            OLED_ShowNum(3U, 7U, spo2, 3U);
        }
        OLED_ShowString(3U, 10U, " %     ");
    }
}
#endif

#if MAX30102_LOCAL_CALIBRATION
static void Monitor_ShowMaxCalibration(const MAX30102_Sample *sample)
{
    uint8_t saturated;

    saturated = sample->red >= MAX30102_NEAR_SATURATION ||
                sample->ir >= MAX30102_NEAR_SATURATION;

    OLED_ShowString(1U, 1U, "RED:");
    OLED_ShowNum(1U, 5U, sample->red, 6U);
    OLED_ShowString(2U, 1U, "IR :");
    OLED_ShowNum(2U, 5U, sample->ir, 6U);
    OLED_ShowString(3U, 1U, saturated ? "SAT: YES" : "SAT: NO ");
}
#endif

void Monitor_Init(void)
{
    previous_finger_state = 0U;
    previous_spo2 = 0xFFU;
    tx_frame_accumulator = 0U;
    tx_display_count = 0U;
    calibration_last_update_tick = 0U;
    ecg_last_update_tick = 0U;

    OLED_Init();
    OLED_ShowString(1U, 1U, "ECG:");
    OLED_ShowNum(1U, 5U, 0U, 4U);
    OLED_ShowString(2U, 1U, "MAX INIT");
#if MAX30102_LOCAL_CALIBRATION
    OLED_ShowString(3U, 1U, "SAT: NO ");
#else
    Monitor_ShowSpO2(0U);
#endif
    Monitor_ShowFingerState(0U);
}

void Monitor_ShowEcgSource(ECG_Source source)
{
    OLED_ShowString(1U, 1U, source == ECG_SOURCE_DIRECT ? "ECG1:" : "ECG0:");
}

void Monitor_UpdateEcgSample(uint16_t sample)
{
    if ((uint32_t)(GetTick() - ecg_last_update_tick) < 100U)
    {
        return;
    }
    ecg_last_update_tick = GetTick();
    OLED_ShowNum(1U, 6U, sample, 4U);
}

/* Part ID 错误和 I2C 无应答都显示 MAX ERR，详细类型保留在状态值中。 */
void Monitor_ShowMaxInitStatus(MAX30102_InitStatus status)
{
    if (status == MAX30102_INIT_OK)
    {
        OLED_ShowString(2U, 1U, "MAX OK TX:0000");
    }
    else
    {
        OLED_ShowString(2U, 1U, "MAX ERR ");
    }
}

void Monitor_ShowEcgInitStatus(ECG_AcquisitionInitStatus status)
{
    if (status == ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT)
    {
        OLED_ShowString(3U, 1U, "ECG RSTCAL ERR ");
    }
    else if (status == ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT)
    {
        OLED_ShowString(3U, 1U, "ECG CAL ERR    ");
    }
}

void Monitor_ShowEcgFault(ECG_AcquisitionFault fault)
{
    if (fault == ECG_ACQUISITION_FAULT_TIM2)
    {
        OLED_ShowString(3U, 1U, "ECG TIM2 ERR   ");
    }
    else if (fault == ECG_ACQUISITION_FAULT_DMA)
    {
        OLED_ShowString(3U, 1U, "ECG DMA ERR    ");
    }
    else if (fault == ECG_ACQUISITION_FAULT_TRIGGER)
    {
        OLED_ShowString(3U, 1U, "ECG TRIG ERR   ");
    }
}

void Monitor_UpdateMaxSample(const MAX30102_Sample *sample, uint8_t spo2,
                             uint8_t finger_present)
{
    if (sample == 0)
    {
        return;
    }

    if (finger_present != previous_finger_state)
    {
        previous_finger_state = finger_present;
        Monitor_ShowFingerState(finger_present);
    }

#if MAX30102_LOCAL_CALIBRATION
    /* 校准画面限制为 4 Hz，避免 OLED 写入占用过多采样时间。 */
    if ((uint32_t)(GetTick() - calibration_last_update_tick) >=
        OLED_CAL_REFRESH_MS)
    {
        calibration_last_update_tick = GetTick();
        Monitor_ShowMaxCalibration(sample);
    }
#else
    (void)sample;
    /* 正式画面仅在数值变化时更新 SpO2。 */
    if (spo2 != previous_spo2)
    {
        previous_spo2 = spo2;
        Monitor_ShowSpO2(spo2);
    }
#endif
}

void Monitor_RecordTxFrame(void)
{
    ++tx_frame_accumulator;
    /* 100 Hz 数据流下计数约每秒增加一次，便于确认发送路径运行。 */
    if (tx_frame_accumulator == UART_TX_FRAMES_PER_COUNT)
    {
        tx_frame_accumulator = 0U;
        tx_display_count = (uint16_t)((tx_display_count + 1U) % 10000U);
        OLED_ShowNum(2U, 11U, tx_display_count, 4U);
    }
}
