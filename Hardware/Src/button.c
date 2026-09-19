#include "button.h"
#include "delay.h"
#include "stm32f10x.h"

#define BUTTON_GPIO GPIOA
#define BUTTON_PIN GPIO_Pin_6
#define BUTTON_DEBOUNCE_MS 25U

/* 最近一次直接读取到的原始电平状态，1表示按下。 */
static uint8_t button_raw_candidate;

/* 已经过消抖确认的稳定状态，1表示按下，0表示释放。 */
static uint8_t button_stable_state;

/* 原始状态最近一次变化时的毫秒时间，用于非阻塞消抖。 */
static uint32_t button_candidate_since;

/* 读取PA6并转换为便于状态机使用的“1表示按下”。 */
static uint8_t Button_ReadPressed(void)
{
    return GPIO_ReadInputDataBit(BUTTON_GPIO, BUTTON_PIN) == Bit_RESET ? 1U : 0U;
}

void Button_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = BUTTON_PIN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BUTTON_GPIO, &gpio);

    button_raw_candidate = Button_ReadPressed();
    button_stable_state = button_raw_candidate;
    button_candidate_since = GetTick();
}

bool Button_PollPressed(void)
{
    uint8_t raw_pressed = Button_ReadPressed();
    uint32_t now = GetTick();

    /*
     * 原始输入变化后重新计时。只有它连续保持25ms，才接受为新的稳定状态。
     */
    if (raw_pressed != button_raw_candidate)
    {
        button_raw_candidate = raw_pressed;
        button_candidate_since = now;
        return false;
    }

    if (button_stable_state != button_raw_candidate &&
        (uint32_t)(now - button_candidate_since) >= BUTTON_DEBOUNCE_MS)
    {
        button_stable_state = button_raw_candidate;
        return button_stable_state != 0U;
    }

    return false;
}
