#ifndef ECG_DMA_H
#define ECG_DMA_H

#include "stm32f10x.h"
#include <stdbool.h>

/* 配置 DMA1 Channel1，将 ADC1->DR 循环搬运到单样本缓存。 */
void DMA_ECG_Init(void);

/* 若有新样本则取出并清除就绪标志，保证每个样本只消费一次。 */
bool DMA_ECG_TakeSample(uint16_t *sample);
void DMA_ECG_Reset(void);

/* 用于区分 DMA 中断未处理与 ADC 未产生转换请求。 */
bool DMA_ECG_TransferCompletePending(void);

#endif
