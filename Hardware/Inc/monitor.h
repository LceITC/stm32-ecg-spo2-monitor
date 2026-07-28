#ifndef MONITOR_H
#define MONITOR_H

#include "MAX30102.h"
#include "ecg_acquisition.h"

/* 初始化 OLED 固定页面和显示模块内部状态。 */
void Monitor_Init(void);

/* 以下接口只负责显示，不直接控制对应外设。 */
void Monitor_ShowMaxInitStatus(MAX30102_InitStatus status);
void Monitor_ShowEcgInitStatus(ECG_AcquisitionInitStatus status);
void Monitor_ShowEcgFault(ECG_AcquisitionFault fault);
void Monitor_ShowEcgSource(ECG_Source source);
void Monitor_UpdateEcgSample(uint16_t sample);

/* 按状态变化或限定刷新周期更新 SpO2、Finger 和本地校准数据。 */
void Monitor_UpdateMaxSample(const MAX30102_Sample *sample, uint8_t spo2,
                             uint8_t finger_present);

/* 每累计约 100 个发送帧，更新一次 OLED 上的 TX 计数。 */
void Monitor_RecordTxFrame(void);

#endif
