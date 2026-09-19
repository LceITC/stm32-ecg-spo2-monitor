#ifndef MONITOR_H
#define MONITOR_H

#include "MAX30102.h"
#include "ecg_acquisition.h"
#include <stdint.h>

/*
 * 初始化OLED和显示状态。默认选择页面0，所有实际OLED发送均由
 * Monitor_Task()在主循环中分段完成。
 */
void Monitor_Init(void);

/* 缓存MAX30102初始化结果；错误会优先覆盖普通页面。 */
void Monitor_ShowMaxInitStatus(MAX30102_InitStatus status);

/* 缓存ADC校准结果；错误会优先覆盖普通页面。 */
void Monitor_ShowEcgInitStatus(ECG_AcquisitionInitStatus status);

/* 缓存TIM2、DMA或ADC触发链路运行故障。 */
void Monitor_ShowEcgFault(ECG_AcquisitionFault fault);

/* 缓存当前实际ECG来源，页面0显示ECG0或ECG2。 */
void Monitor_ShowEcgSource(ECG_Source source);

/* 缓存最新滤波ECG ADC值，不直接访问OLED。 */
void Monitor_UpdateEcgSample(uint16_t sample);

/* 缓存最新ECG心率，0表示当前心率无效。 */
void Monitor_UpdateHeartRate(uint16_t heart_rate);

/*
 * 缓存最新MAX30102数据、SpO2和Finger状态，同时从IR原始数据提取
 * 交流脉搏分量并按40点/秒写入后台波形缓冲。
 */
void Monitor_UpdateMaxSample(const MAX30102_Sample *sample, uint8_t spo2,
                             uint8_t finger_present);

/* 在页面0数据页和页面1 PPG波形页之间切换。 */
void Monitor_NextPage(void);

/*
 * 主循环显示任务：限速绘图，并且每次最多执行一次32字节OLED分段刷新。
 * 不得从中断服务函数调用。
 */
void Monitor_Task(void);

#endif
