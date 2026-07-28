#ifndef FINGER_DETECTOR_H
#define FINGER_DETECTOR_H

#include "MAX30102.h"
#include <stdbool.h>

/* 连续帧计数形成迟滞，避免单个异常样本引起状态跳变。 */
typedef struct
{
    uint16_t valid_frames;
    uint16_t invalid_frames;
    uint8_t present;
    uint32_t reference_red;
    uint32_t reference_ir;
} FingerDetector;

void FingerDetector_Init(FingerDetector *detector);

/*
 * 更新手指状态，并通过 sample_valid 返回当前 RED/IR 是否满足三重判据。
 * 返回值为迟滞处理后的 Finger ON/OFF 状态。
 */
uint8_t FingerDetector_Process(FingerDetector *detector,
                               const MAX30102_Sample *sample,
                               bool *sample_valid);

#endif
