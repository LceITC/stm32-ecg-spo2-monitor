#ifndef MAX30102_SPO2_H
#define MAX30102_SPO2_H

#include <stdbool.h>
#include <stdint.h>

/* 100Hz下500点对应5秒RED/IR计算窗口。 */
#define MAX30102_SPO2_WINDOW_SIZE 500U /* RED和IR窗口的固定样本数。 */

/* 清空RED/IR窗口、候选结果、稳定计数和当前输出值。 */
void MAX30102_SpO2_Init(void);

/* 与Init等价，提供语义明确的运行时复位接口。 */
void MAX30102_SpO2_Reset(void);

/*
 * red/ir：最新18位原始样本。
 * sample_valid：true时样本进入计算窗口，false时累计无效时间。
 */
void MAX30102_SpO2_ProcessSample(uint32_t red, uint32_t ir, bool sample_valid);

/* 返回最近稳定SpO2整数百分比；没有可显示结果时返回0。 */
uint8_t MAX30102_SpO2_GetValue(void);

#endif
