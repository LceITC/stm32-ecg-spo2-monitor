#ifndef MAX30102_SPO2_H
#define MAX30102_SPO2_H

#include "stm32f10x.h"
#include <stdbool.h>

/* 100 Hz 下 500 点对应 5 秒 RED/IR 计算窗口。 */
#define MAX30102_SPO2_WINDOW_SIZE 500U

/* 初始化或清空窗口；无手指时可显式复位算法。 */
void MAX30102_SpO2_Init(void);
void MAX30102_SpO2_Reset(void);

/* 输入每个新 RED/IR 样本；sample_valid 为手指三重判据结果。 */
void MAX30102_SpO2_ProcessSample(uint32_t red, uint32_t ir, bool sample_valid);

/* 窗口未满、结果未稳定或无手指时返回 0。 */
uint8_t MAX30102_SpO2_GetValue(void);

#endif
