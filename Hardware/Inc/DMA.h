#ifndef ECG_DMA_H
#define ECG_DMA_H

#include "stm32f10x.h"
#include <stdbool.h>

/* 配置DMA1 Channel1、传输完成中断和软件采样队列。 */
void DMA_ECG_Init(void);

/*
 * 按采样顺序取出一个尚未消费的新样本。
 * sample：接收12位ADC结果的输出指针。
 * 返回true表示成功取到一个新样本，false表示队列为空或指针无效。
 */
bool DMA_ECG_TakeSample(uint16_t *sample);

/* 清空软件队列和DMA1 Channel1遗留标志，用于ECG来源切换。 */
void DMA_ECG_Reset(void);

/* 返回DMA传输完成标志是否仍挂起，用于采样超时故障定位。 */
bool DMA_ECG_TransferCompletePending(void);

#endif
