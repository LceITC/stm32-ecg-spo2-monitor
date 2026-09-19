#include "TIM2.h"
#include "stm32f10x.h"

/*
 * 建立严格100Hz硬件时基。函数只配置定时器，不在配置结束时启动计数，
 * 使上层能够先完成DMA和ADC准备工作。
 */
void TIM2_ECG_Init(void)
{
    TIM_TimeBaseInitTypeDef timer; /* 保存TIM2预分频、周期和计数方向。 */
    TIM_OCInitTypeDef compare; /* 保存CC2内部比较事件的PWM2配置。 */

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

/* 从CNT=0重新启动，保证每次启动后的第一个触发周期完整为10ms。 */
void TIM2_ECG_Start(void)
{
    TIM_SetCounter(TIM2, 0U);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update | TIM_FLAG_CC2);
    TIM_Cmd(TIM2, ENABLE);
}

/* 停止计数器；已经写入的PSC、ARR和CCR2配置保持不变。 */
void TIM2_ECG_Stop(void)
{
    TIM_Cmd(TIM2, DISABLE);
}

/* 读取CC2标志但不清除，用于采样超时时判断TIM2是否实际运行过。 */
bool TIM2_ECG_CompareEventSeen(void)
{
    return TIM_GetFlagStatus(TIM2, TIM_FLAG_CC2) == SET;
}
