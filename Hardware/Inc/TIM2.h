#ifndef ECG_TIM2_H
#define ECG_TIM2_H

#include <stdbool.h>

/* 配置TIM2为100Hz，并通过CC2内部事件触发ADC1，不占用外部引脚。 */
void TIM2_ECG_Init(void);

/* 将计数器清零并启动TIM2，开始100Hz ADC触发。 */
void TIM2_ECG_Start(void);

/* 停止TIM2；切换ADC通道时用于阻止新的转换开始。 */
void TIM2_ECG_Stop(void);

/* 返回TIM2是否产生过CC2事件，用于区分定时器故障和ADC/DMA故障。 */
bool TIM2_ECG_CompareEventSeen(void);

#endif
