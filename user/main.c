#include "MAX30102.h"
#include "button.h"
#include "delay.h"
#include "ecg_acquisition.h"
#include "ecg_filter.h"
#include "ecg_hr.h"
#include "finger_detector.h"
#include "max30102_spo2.h"
#include "monitor.h"
#include "serial.h"
#include "stm32f10x.h"

/*
 * ============================== 引脚分配总览 ==============================
 * MCU ：STM32F103C8T6（LQFP48），HSE 8MHz 经 PLL×9 = 72MHz SYSCLK。
 * 固件：StdPeriph 标准外设库（非 HAL）。
 *
 * 本工程的 GPIO 全部在 Hardware/Src 和 my_lib/Src 里手写配置，5_1OLED.ioc
 * 只声明了 SWD，对本程序没有参考价值，看引脚请以本表为准。
 *
 * ---- PORTA ----
 * PA0   模拟输入           ADC1_IN0，AD8232 心电模块 OUTPUT 信号  → ADC.c
 * PA2   模拟输入           ADC1_IN2，DIRECT 心电来源（信号源）   → ADC.c
 * PA6   上拉输入           按键（低有效），主循环轮询消抖，切换OLED页面
 *                                                               → button.c
 * PA9   复用推挽输出       USART1_TX，100Hz 四字段CSV上传 115200 8N1
 *                                                               → serial.c
 * PA10  上拉输入           USART1_RX，接收 ECG_SRC 切换命令（中断接收）
 *                                                               → serial.c
 * PA13  SWD                SWDIO 调试数据
 * PA14  SWD                SWCLK 调试时钟
 *
 * ---- PORTB ----
 * PB8   开漏输出           软件I2C SCL（OLED/SSD1306，只写不读ACK）→ OLED.c
 * PB9   开漏输出           软件I2C SDA（OLED/SSD1306，地址0x3C）  → OLED.c
 * PB10  开漏输出           软件I2C SCL（MAX30102，约250kHz）      → MAX30102.c
 * PB11  开漏输出（可回读） 软件I2C SDA（MAX30102，双向读ACK）     → MAX30102.c
 * PB12  上拉输入           MAX30102 INT，低有效数据就绪，主循环轮询
 *                                                              → MAX30102.c
 *
 * ---- 相关但未引出的引脚 ----
 * PB3   未使用             TIM2 CH2 只作内部 OC2REF 触发 ADC1，不输出到引脚
 *                                                               → TIM2.c
 *
 * ---- 容易踩坑的几点 ----
 * 1. 无硬件 I2C：两路 I2C 均为 GPIO 软件位操作，STM32 的 I2C1/I2C2 未使用。
 * 2. OLED 与 MAX30102 是两条独立总线，不共用引脚。
 * 3. 全工程无 EXTI：PA6 和 PB12 都是普通输入，在主循环里轮询。
 * 4. 板上没有 LED；唯一的“LED”标识是 MAX30102 自身的发光管寄存器。
 * 5. 中断只有 3 个：DMA1_Channel1(1/0)、USART1(2/0)、SysTick 1ms。
 *
 * ---- 相关外设（不单独占用引脚）----
 * TIM2  72MHz/7200/100 = 100Hz，CC2 内部事件触发 ADC1          → TIM2.c
 * ADC1  由 T2_CC2 外部触发，55.5 周期采样，运行时切换通道       → ADC.c
 * DMA1  Channel1 把 ADC1->DR 循环搬到 32 槽软件环形队列         → DMA.c
 * =========================================================================
 */

/* MAX30102上电时允许短暂无应答，因此最多重试三次。 */
#define MAX30102_INIT_RETRIES 3U /* 每次失败间隔20ms。 */

/*
 * 程序入口只负责模块初始化和主循环调度。硬件配置、算法、显示格式、
 * 中断和协议编码分别保留在各自模块中。
 */
int main(void)
{
    FingerDetector detector; /* MAX30102手指进入、跟踪和离开状态机。 */
    MAX30102_Sample max_sample = {0U, 0U}; /* 当前读取的RED/IR原始样本。 */
    MAX30102_InitStatus max_status = MAX30102_INIT_I2C_ERROR; /* 初始化结果。 */
    ECG_AcquisitionInitStatus ecg_status; /* ADC/DMA/TIM2启动结果。 */
    ECG_AcquisitionFault ecg_fault; /* 主循环最近一次ECG超时诊断结果。 */
    ECG_Filter ecg_filter; /* 当前ECG来源的两级滤波历史。 */
    ECG_HRDetector ecg_hr_detector; /* 当前ECG来源的R峰和RR状态。 */
    ECG_Source requested_source; /* 网页命令转换后的采集模块来源。 */
    Serial_EcgSource serial_source; /* 串口模块解析出的来源枚举。 */
    bool max_sample_valid; /* 当前光学样本能否进入SpO2窗口。 */
    uint8_t part_id = 0U; /* MAX30102初始化时读取的PART_ID。 */
    uint8_t max_available; /* 1表示MAX30102初始化成功，可以读取FIFO。 */
    uint8_t max_init_attempt; /* MAX30102上电初始化重试序号。 */
    uint8_t finger_present; /* 迟滞处理后的Finger ON/OFF状态。 */
    uint8_t latest_spo2 = 0U; /* 最近稳定SpO2，0表示暂无结果。 */
    uint16_t ecg_sample; /* 从DMA软件队列取出的12位原始ADC值。 */
    uint16_t latest_ecg = 0U; /* 最近两级滤波后的ECG数值。 */
    uint16_t latest_hr = 0U; /* ECG算法最近输出的整数BPM。 */
    uint32_t latest_ir = 0U; /* 最近MAX30102 IR值，作为CSV第四字段。 */

    /* 先建立显示和调试接口，再启动传感器与定时采集。 */
    Monitor_Init();
    My_Serial_Init();
    Delay_Init();
    Button_Init();
    FingerDetector_Init(&detector);
    MAX30102_SpO2_Init();
    ECG_Filter_Init(&ecg_filter);
    ECG_HR_Init(&ecg_hr_detector);

    /* MAX30102 使用 PB10/PB11 软件 I2C，PB12 为低有效中断输入。 */
    MAX30102_GPIO_Init();
    for (max_init_attempt = 0U; max_init_attempt < MAX30102_INIT_RETRIES; ++max_init_attempt)
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
    Serial_SendMaxInitLog(max_status, part_id);

    ecg_status = ECG_Acquisition_Init();
    Monitor_ShowEcgInitStatus(ecg_status);
    Monitor_ShowEcgSource(ECG_SOURCE_AD8232);

    while (1)
    {
        /*
         * PA6按键采用主循环非阻塞轮询。每次稳定按下只切换一次页面，
         * 长按期间不会重复触发，释放后才允许下一次按下。
         */
        if (Button_PollPressed())
        {
            Monitor_NextPage();
        }

        if (Serial_PollEcgSource(&serial_source))
        {
            requested_source = serial_source == SERIAL_ECG_SOURCE_DIRECT ? ECG_SOURCE_DIRECT : ECG_SOURCE_AD8232;
            if (ECG_Acquisition_SelectSource(requested_source))
            {
                ECG_Filter_Init(&ecg_filter);
                ECG_HR_Init(&ecg_hr_detector);
                latest_ecg = 0U;
                latest_hr = 0U;
                Monitor_ShowEcgSource(requested_source);
            }
        }

        /* MAX30102 独立更新最新 IR、SpO2 和手指状态，不控制 ECG 时基。 */
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
                latest_ir = max_sample.ir;
                Monitor_UpdateMaxSample(&max_sample, latest_spo2,
                                        finger_present);
            }
        }

        /*
         * CSV 严格跟随 TIM2 触发并由 DMA 搬运的 ECG 新样本。环形队列
         * 保证短暂阻塞后仍按顺序处理，每个 ECG 样本只发送一次。
         */
        while (ECG_Acquisition_TakeSample(&ecg_sample))
        {
            latest_ecg = ECG_Filter_Process(&ecg_filter, ecg_sample);
            latest_hr = ECG_HR_Process(&ecg_hr_detector, latest_ecg);
            Monitor_UpdateEcgSample(latest_ecg);
            Monitor_UpdateHeartRate(latest_hr);
            Serial_SendMonitorFrame(latest_ecg, latest_spo2, latest_hr,
                                    latest_ir);
        }

        /* 采样停顿超过 200 ms 时，将故障类型交给 OLED 显示模块。 */
        ecg_fault = ECG_Acquisition_PollFault();
        if (ecg_fault != ECG_ACQUISITION_FAULT_NONE)
        {
            Monitor_ShowEcgFault(ecg_fault);
        }

        /*
         * OLED只在统一显示任务中访问。任务每次最多发送32字节，
         * 不会改变100Hz ECG采样、MAX30102 FIFO读取或四字段CSV协议。
         */
        Monitor_Task();
    }
}
