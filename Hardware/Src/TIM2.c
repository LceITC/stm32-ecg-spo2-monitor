#include "TIM2.h"
#include "stm32f10x.h"

void TIM2_ECG_Init(void)
{
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef compare;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /*
     * TIM2 输入时钟为 72 MHz：
     * 72 MHz / (7199 + 1) / (99 + 1) = 100 Hz。
     */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 7199U;
    timer.TIM_Period = 99U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timer);

    TIM_OCStructInit(&compare);
    /*
     * PWM2 在 CCR2 处让内部 OC2REF 产生上升沿，ADC1 使用该内部边沿
     * 启动转换。无需输出到 PB3，因此不会额外占用外部引脚。
     */
    compare.TIM_OCMode = TIM_OCMode_PWM2;
    compare.TIM_OutputState = TIM_OutputState_Enable;
    compare.TIM_Pulse = 1U;
    TIM_OC2Init(TIM2, &compare);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

    TIM_SetCounter(TIM2, 0U);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update | TIM_FLAG_CC2);
}

void TIM2_ECG_Start(void)
{
    TIM_SetCounter(TIM2, 0U);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update | TIM_FLAG_CC2);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_ECG_Stop(void)
{
    TIM_Cmd(TIM2, DISABLE);
}

bool TIM2_ECG_CompareEventSeen(void)
{
    return TIM_GetFlagStatus(TIM2, TIM_FLAG_CC2) == SET;
}
