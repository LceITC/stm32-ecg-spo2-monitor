#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

/*
 * 初始化PA6为内部上拉输入。按键另一端接GND，因此按下时读取为低电平。
 */
void Button_Init(void);

/*
 * 非阻塞轮询按键并完成25ms消抖。
 * 只有稳定的“释放到按下”边沿返回true，长按期间不会重复触发。
 */
bool Button_PollPressed(void);

#endif
