#ifndef ECG_TIM2_H
#define ECG_TIM2_H

#include <stdbool.h>

/* 配置 TIM2_CC2，使其每 10 ms 产生一次 ADC 内部触发边沿。 */
void TIM2_ECG_Init(void);
void TIM2_ECG_Start(void);
void TIM2_ECG_Stop(void);

/* 检查 TIM2 是否至少产生过一次 CC2 比较事件。 */
bool TIM2_ECG_CompareEventSeen(void);

#endif
