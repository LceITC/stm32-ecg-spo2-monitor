#ifndef FINGER_DETECTOR_H
#define FINGER_DETECTOR_H

#include "MAX30102.h"
#include <stdbool.h>

/* 保存手指进入/离开迟滞计数以及放入后的RED/IR参考直流量。 */
typedef struct
{
    uint16_t valid_frames;   /* 尚未确认放入时连续有效样本数。 */
    uint16_t invalid_frames; /* 已确认放入后连续无效样本数。 */
    uint8_t present;         /* 1表示Finger ON，0表示Finger OFF。 */
    uint32_t reference_red;  /* 手指放入后的RED慢速参考值。 */
    uint32_t reference_ir;   /* 手指放入后的IR慢速参考值。 */
} FingerDetector;

/* 清空detector的连续帧计数、手指状态和跟踪参考值。 */
void FingerDetector_Init(FingerDetector *detector);

/*
 * detector：需要更新的手指检测器状态。
 * sample：本次MAX30102的RED/IR原始样本。
 * sample_valid：输出当前样本能否送入SpO2窗口。
 * 返回值：迟滞处理后的Finger ON/OFF状态。
 */
uint8_t FingerDetector_Process(FingerDetector *detector,
                               const MAX30102_Sample *sample,
                               bool *sample_valid);

#endif
