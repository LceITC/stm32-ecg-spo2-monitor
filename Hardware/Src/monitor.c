#include "monitor.h"
#include "OLED.h"
#include "delay.h"
#include <stdbool.h>

/* 页面0按10Hz更新数值，页面1按约12.5Hz重绘滚动波形。 */
#define MONITOR_DATA_REFRESH_MS 100U
#define MONITOR_PPG_REFRESH_MS 80U

/*
 * 将MAX30102的100Hz采样重采样为40个显示点/秒。
 * 128列因此覆盖约3.2秒，90~100 BPM时屏幕上约显示5次脉搏。
 */
#define PPG_INPUT_RATE_HZ 100U
#define PPG_DISPLAY_RATE_HZ 40U
#define PPG_POINT_COUNT 128U

/* IR直流基线使用1/64步长的一阶IIR，Q8格式保留小变化。 */
#define PPG_BASELINE_FILTER_SHIFT 6U
#define PPG_BASELINE_Q_SHIFT 8U

/* 波形区域为y=16~63，中线40，峰值最多占中线上下21像素。 */
#define PPG_WAVE_TOP 16
#define PPG_WAVE_BOTTOM 63
#define PPG_WAVE_CENTER 40
#define PPG_WAVE_HALF_HEIGHT 21

/*
 * 自动缩放的最小交流跨度。小于该值的噪声不会被放大为满屏波形，
 * 包络在幅度下降时以1/64的速度缓慢回落。
 */
#define PPG_MIN_SCALE 300L
#define PPG_SCALE_DECAY_SHIFT 6U

typedef enum
{
    MONITOR_PAGE_DATA = 0, /* ECG、HR、SpO2和Finger四行数据页。 */
    MONITOR_PAGE_PPG       /* MAX30102 IR交流分量滚动波形页。 */
} Monitor_Page;

/* 当前用户选择的页面，上电默认数据页。 */
static Monitor_Page selected_page;

/* 上一次实际绘制的页面，用于检测切换并强制清屏。 */
static Monitor_Page rendered_page;

/* true表示页面内容需要无条件重新生成。 */
static bool force_redraw;

/* 页面上次生成显存内容的毫秒时间。 */
static uint32_t last_render_tick;

/* 页面0缓存的当前ECG来源和最新ADC值。 */
static ECG_Source latest_ecg_source;
static uint16_t latest_ecg_sample;

/* 页面0和页面1头部使用的最新HR、SpO2、Finger状态。 */
static uint16_t latest_heart_rate;
static uint8_t latest_spo2;
static uint8_t latest_finger_present;

/* 初始化和运行错误缓存；非正常状态优先显示数据错误页。 */
static MAX30102_InitStatus latest_max_status;
static ECG_AcquisitionInitStatus latest_ecg_init_status;
static ECG_AcquisitionFault latest_ecg_fault;

/* 128个重采样IR交流点组成约3.2秒的滚动波形历史。 */
static int32_t ppg_points[PPG_POINT_COUNT];

/* 环形缓冲下一个写入位置以及当前有效点数量。 */
static uint8_t ppg_write_index;
static uint8_t ppg_valid_count;

/* IR直流基线，使用Q8定点数防止小差值因整数除法完全丢失。 */
static int32_t ppg_baseline_q8;

/*
 * 分数重采样状态：交流样本先累加平均，phase决定何时输出40Hz显示点。
 * 100Hz转40Hz时会交替平均3个、2个输入样本，兼顾密度和抗毛刺能力。
 */
static int32_t ppg_ac_accumulator;
static uint8_t ppg_average_count;
static uint8_t ppg_resample_phase;

/* 当前自动缩放包络，单位与IR原始ADC交流量一致。 */
static int32_t ppg_scale;

/* true表示Finger ON后已经用首个IR样本建立了新基线。 */
static bool ppg_tracking_active;

/* 清除波形历史、基线、降采样和缩放状态。 */
static void Monitor_ResetPpg(void)
{
    uint8_t index;

    for (index = 0U; index < PPG_POINT_COUNT; ++index)
    {
        ppg_points[index] = 0;
    }

    ppg_write_index = 0U;
    ppg_valid_count = 0U;
    ppg_baseline_q8 = 0;
    ppg_ac_accumulator = 0;
    ppg_average_count = 0U;
    ppg_resample_phase = 0U;
    ppg_scale = PPG_MIN_SCALE;
    ppg_tracking_active = false;
}

/* 返回32位有符号数绝对值；MAX30102交流量不会达到INT32_MIN。 */
static int32_t Monitor_Abs32(int32_t value)
{
    return value >= 0 ? value : -value;
}

/*
 * 把一个新的40Hz交流点写入环形缓冲，并更新慢速自动缩放包络。
 * 幅度上升时立即扩展范围，幅度下降时缓慢收缩，减少波形跳动。
 */
static void Monitor_PushPpgPoint(int32_t point)
{
    int32_t magnitude = Monitor_Abs32(point);

    ppg_points[ppg_write_index] = point;
    ppg_write_index = (uint8_t)((ppg_write_index + 1U) % PPG_POINT_COUNT);
    if (ppg_valid_count < PPG_POINT_COUNT)
    {
        ++ppg_valid_count;
    }

    if (magnitude > ppg_scale)
    {
        ppg_scale = magnitude;
    }
    else if (ppg_scale > PPG_MIN_SCALE)
    {
        int32_t decay =
            (ppg_scale - PPG_MIN_SCALE) >> PPG_SCALE_DECAY_SHIFT;
        ppg_scale -= decay > 0 ? decay : 1;
    }
}

/*
 * 对100Hz IR样本进行直流去除，再通过相位累加器重采样到40Hz。
 * Finger重新进入时首点只建立基线，避免旧手指状态和接触瞬态残留。
 */
static void Monitor_ProcessPpgSample(uint32_t ir, uint8_t finger_present)
{
    int32_t ir_q8;
    int32_t ac;

    if (finger_present == 0U)
    {
        if (ppg_tracking_active || ppg_valid_count != 0U)
        {
            Monitor_ResetPpg();
            force_redraw = true;
        }
        return;
    }

    ir_q8 = (int32_t)(ir << PPG_BASELINE_Q_SHIFT);
    if (!ppg_tracking_active)
    {
        Monitor_ResetPpg();
        ppg_tracking_active = true;
        ppg_baseline_q8 = ir_q8;
        force_redraw = true;
        return;
    }

    ppg_baseline_q8 +=
        (ir_q8 - ppg_baseline_q8) >> PPG_BASELINE_FILTER_SHIFT;
    ac = (int32_t)ir - (ppg_baseline_q8 >> PPG_BASELINE_Q_SHIFT);

    ppg_ac_accumulator += ac;
    ++ppg_average_count;
    ppg_resample_phase =
        (uint8_t)(ppg_resample_phase + PPG_DISPLAY_RATE_HZ);
    if (ppg_resample_phase >= PPG_INPUT_RATE_HZ)
    {
        ppg_resample_phase =
            (uint8_t)(ppg_resample_phase - PPG_INPUT_RATE_HZ);
        Monitor_PushPpgPoint(ppg_ac_accumulator /
                             (int32_t)ppg_average_count);
        ppg_ac_accumulator = 0;
        ppg_average_count = 0U;
    }
}

/* true表示存在必须覆盖普通波形页的MAX或ECG故障。 */
static bool Monitor_HasFault(void)
{
    return latest_max_status != MAX30102_INIT_OK ||
           latest_ecg_init_status != ECG_ACQUISITION_INIT_OK ||
           latest_ecg_fault != ECG_ACQUISITION_FAULT_NONE;
}

/* 在页面0第二行显示固定宽度心率；0显示占位符。 */
static void Monitor_DrawHeartRate(void)
{
    OLED_ShowString(2U, 1U, "HR : ");
    if (latest_heart_rate == 0U)
    {
        OLED_ShowString(2U, 6U, "--- BPM    ");
    }
    else if (latest_heart_rate < 100U)
    {
        OLED_ShowString(2U, 6U, " ");
        OLED_ShowNum(2U, 7U, latest_heart_rate, 2U);
        OLED_ShowString(2U, 9U, " BPM    ");
    }
    else
    {
        OLED_ShowNum(2U, 6U, latest_heart_rate, 3U);
        OLED_ShowString(2U, 9U, " BPM    ");
    }
}

/* 在页面0第三行显示SpO2；0表示尚未稳定或当前没有手指。 */
static void Monitor_DrawSpO2(void)
{
    if (latest_spo2 == 0U)
    {
        OLED_ShowString(3U, 1U, "SpO2: -- %     ");
    }
    else if (latest_spo2 < 100U)
    {
        OLED_ShowString(3U, 1U, "SpO2:  ");
        OLED_ShowNum(3U, 8U, latest_spo2, 2U);
        OLED_ShowString(3U, 10U, " %     ");
    }
    else
    {
        OLED_ShowString(3U, 1U, "SpO2: ");
        OLED_ShowNum(3U, 7U, latest_spo2, 3U);
        OLED_ShowString(3U, 10U, " %     ");
    }
}

/* 将最高优先级故障写到第三行；无故障时正常显示SpO2。 */
static void Monitor_DrawStatusLine(void)
{
    if (latest_max_status != MAX30102_INIT_OK)
    {
        OLED_ShowString(3U, 1U, "MAX INIT ERR    ");
    }
    else if (latest_ecg_init_status ==
             ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT)
    {
        OLED_ShowString(3U, 1U, "ECG RSTCAL ERR ");
    }
    else if (latest_ecg_init_status ==
             ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT)
    {
        OLED_ShowString(3U, 1U, "ECG CAL ERR    ");
    }
    else if (latest_ecg_fault == ECG_ACQUISITION_FAULT_TIM2)
    {
        OLED_ShowString(3U, 1U, "ECG TIM2 ERR   ");
    }
    else if (latest_ecg_fault == ECG_ACQUISITION_FAULT_DMA)
    {
        OLED_ShowString(3U, 1U, "ECG DMA ERR    ");
    }
    else if (latest_ecg_fault == ECG_ACQUISITION_FAULT_TRIGGER)
    {
        OLED_ShowString(3U, 1U, "ECG TRIG ERR   ");
    }
    else
    {
        Monitor_DrawSpO2();
    }
}

/* 生成页面0的四行显存内容，不在此函数中直接发送I2C数据。 */
static void Monitor_RenderDataPage(void)
{
    OLED_ShowString(1U, 1U,
                    latest_ecg_source == ECG_SOURCE_DIRECT ? "ECG2:"
                                                           : "ECG0:");
    OLED_ShowNum(1U, 6U, latest_ecg_sample, 4U);
    OLED_ShowString(1U, 10U, "       ");

    Monitor_DrawHeartRate();
    Monitor_DrawStatusLine();
    OLED_ShowString(4U, 1U,
                    latest_finger_present != 0U ? "Finger: ON     "
                                                : "Finger: OFF    ");
}

/* 生成页面1顶部16像素状态栏。 */
static void Monitor_DrawPpgHeader(void)
{
    if (latest_finger_present == 0U)
    {
        OLED_ShowString(1U, 1U, "PPG NO FINGER   ");
    }
    else if (latest_spo2 == 0U)
    {
        OLED_ShowString(1U, 1U, "PPG SpO2: --%   ");
    }
    else if (latest_spo2 < 100U)
    {
        OLED_ShowString(1U, 1U, "PPG SpO2: ");
        OLED_ShowNum(1U, 11U, latest_spo2, 2U);
        OLED_ShowString(1U, 13U, "%   ");
    }
    else
    {
        OLED_ShowString(1U, 1U, "PPG SpO2:");
        OLED_ShowNum(1U, 10U, latest_spo2, 3U);
        OLED_ShowString(1U, 13U, "%   ");
    }
}

/* 将一个IR交流点按当前包络映射到y=19~61，超界值会被裁剪。 */
static int16_t Monitor_MapPpgY(int32_t point)
{
    int32_t scaled;
    int32_t y;
    int32_t scale = ppg_scale >= PPG_MIN_SCALE ? ppg_scale : PPG_MIN_SCALE;

    scaled = point * PPG_WAVE_HALF_HEIGHT / scale;
    if (scaled > PPG_WAVE_HALF_HEIGHT)
    {
        scaled = PPG_WAVE_HALF_HEIGHT;
    }
    else if (scaled < -PPG_WAVE_HALF_HEIGHT)
    {
        scaled = -PPG_WAVE_HALF_HEIGHT;
    }

    y = PPG_WAVE_CENTER - scaled;
    if (y < PPG_WAVE_TOP + 1)
    {
        y = PPG_WAVE_TOP + 1;
    }
    else if (y > PPG_WAVE_BOTTOM - 1)
    {
        y = PPG_WAVE_BOTTOM - 1;
    }

    return (int16_t)y;
}

/*
 * 按“最旧点在左、最新点在右”的顺序重绘128列历史，并用连续线段
 * 连接相邻点。数据不足一屏时从右侧开始显示已有历史。
 */
static void Monitor_DrawPpgWave(void)
{
    uint8_t point_number;
    uint8_t oldest_index;
    int16_t start_x;
    int16_t previous_x = 0;
    int16_t previous_y = PPG_WAVE_CENTER;

    OLED_ClearArea(0U, PPG_WAVE_TOP, 128U,
                   (uint8_t)(PPG_WAVE_BOTTOM - PPG_WAVE_TOP + 1));

    if (latest_finger_present == 0U)
    {
        OLED_ShowString(3U, 3U, "NO FINGER   ");
        return;
    }

    if (ppg_valid_count < 2U)
    {
        OLED_ShowString(3U, 3U, "ACQUIRING   ");
        return;
    }

    oldest_index =
        (uint8_t)((ppg_write_index + PPG_POINT_COUNT - ppg_valid_count) %
                  PPG_POINT_COUNT);
    start_x = (int16_t)(PPG_POINT_COUNT - ppg_valid_count);

    for (point_number = 0U; point_number < ppg_valid_count; ++point_number)
    {
        uint8_t buffer_index =
            (uint8_t)((oldest_index + point_number) % PPG_POINT_COUNT);
        int16_t x = (int16_t)(start_x + point_number);
        int16_t y = Monitor_MapPpgY(ppg_points[buffer_index]);

        if (point_number != 0U)
        {
            OLED_DrawLine(previous_x, previous_y, x, y);
        }
        previous_x = x;
        previous_y = y;
    }
}

/* 生成页面1的状态栏和波形区域显存内容。 */
static void Monitor_RenderPpgPage(void)
{
    Monitor_DrawPpgHeader();
    Monitor_DrawPpgWave();
}

void Monitor_Init(void)
{
    selected_page = MONITOR_PAGE_DATA;
    rendered_page = MONITOR_PAGE_DATA;
    force_redraw = true;
    last_render_tick = 0U;

    latest_ecg_source = ECG_SOURCE_AD8232;
    latest_ecg_sample = 0U;
    latest_heart_rate = 0U;
    latest_spo2 = 0U;
    latest_finger_present = 0U;

    latest_max_status = MAX30102_INIT_OK;
    latest_ecg_init_status = ECG_ACQUISITION_INIT_OK;
    latest_ecg_fault = ECG_ACQUISITION_FAULT_NONE;

    Monitor_ResetPpg();
    OLED_Init();
}

void Monitor_ShowMaxInitStatus(MAX30102_InitStatus status)
{
    if (latest_max_status != status)
    {
        latest_max_status = status;
        force_redraw = true;
    }
}

void Monitor_ShowEcgInitStatus(ECG_AcquisitionInitStatus status)
{
    if (latest_ecg_init_status != status)
    {
        latest_ecg_init_status = status;
        force_redraw = true;
    }
}

void Monitor_ShowEcgFault(ECG_AcquisitionFault fault)
{
    if (fault != ECG_ACQUISITION_FAULT_NONE && latest_ecg_fault != fault)
    {
        latest_ecg_fault = fault;
        force_redraw = true;
    }
}

void Monitor_ShowEcgSource(ECG_Source source)
{
    if (latest_ecg_source != source)
    {
        latest_ecg_source = source;
        force_redraw = true;
    }
}

void Monitor_UpdateEcgSample(uint16_t sample)
{
    latest_ecg_sample = sample;
}

void Monitor_UpdateHeartRate(uint16_t heart_rate)
{
    latest_heart_rate = heart_rate;
}

void Monitor_UpdateMaxSample(const MAX30102_Sample *sample, uint8_t spo2,
                             uint8_t finger_present)
{
    if (sample == 0)
    {
        return;
    }

    if (latest_finger_present != finger_present)
    {
        latest_finger_present = finger_present;
        force_redraw = true;
    }
    latest_spo2 = spo2;
    Monitor_ProcessPpgSample(sample->ir, finger_present);
}

void Monitor_NextPage(void)
{
    selected_page = selected_page == MONITOR_PAGE_DATA ? MONITOR_PAGE_PPG : MONITOR_PAGE_DATA;
    force_redraw = true;
}

void Monitor_Task(void)
{
    Monitor_Page effective_page =
        Monitor_HasFault() ? MONITOR_PAGE_DATA : selected_page;
    uint32_t now = GetTick();
    uint32_t refresh_interval =
        effective_page == MONITOR_PAGE_PPG ? MONITOR_PPG_REFRESH_MS
                                           : MONITOR_DATA_REFRESH_MS;

    if (effective_page != rendered_page)
    {
        rendered_page = effective_page;
        OLED_Clear();
        force_redraw = true;
    }

    if (force_redraw ||
        (uint32_t)(now - last_render_tick) >= refresh_interval)
    {
        if (effective_page == MONITOR_PAGE_PPG)
        {
            Monitor_RenderPpgPage();
        }
        else
        {
            Monitor_RenderDataPage();
        }
        last_render_tick = now;
        force_redraw = false;
    }

    /* 每次主循环最多发送一个32字节片段，避免阻塞100Hz采样处理。 */
    (void)OLED_RefreshStep();
}
