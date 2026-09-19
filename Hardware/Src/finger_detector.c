#include "finger_detector.h"

#define FINGER_VALID_FRAMES 3U      /* Finger OFF转ON所需连续有效帧数。 */
#define FINGER_INVALID_FRAMES 500U  /* Finger ON转OFF所需连续无效帧数。 */
#define FINGER_IR_MIN 50000U        /* 首次确认手指时的IR下限。 */
#define FINGER_IR_MAX 200000U       /* 首次确认手指时的IR上限。 */
#define FINGER_RED_MIN 40000U       /* 首次确认手指时的RED下限。 */
#define FINGER_RED_MAX 180000U      /* 首次确认手指时的RED上限。 */
#define FINGER_NEAR_SATURATION 260000U /* 接近18位满量程的拒绝阈值。 */
#define FINGER_TRACK_MIN_PERCENT 25U   /* 跟踪值可降至参考值的25%。 */
#define FINGER_TRACK_MAX_PERCENT 240U  /* 跟踪值可升至参考值的240%。 */
#define FINGER_TRACK_RATIO_MIN_TENTHS 3U  /* 跟踪RED/IR最小值0.3。 */
#define FINGER_TRACK_RATIO_MAX_TENTHS 15U /* 跟踪RED/IR最大值1.5。 */
#define FINGER_REFERENCE_FILTER_SHIFT 5U  /* 参考值每点更新差值的1/32。 */

/*
 * 三重有效判据：
 * IR 处于 50k~200k，RED 处于 40k~180k，且 RED/IR 为 0.5~1.2。
 * 使用整数交叉相乘，避免在 Cortex-M3 上引入浮点除法。
 */
static bool FingerDetector_IsValidSample(const MAX30102_Sample *sample)
{
    return sample->ir >= FINGER_IR_MIN && sample->ir <= FINGER_IR_MAX &&
           sample->red >= FINGER_RED_MIN && sample->red <= FINGER_RED_MAX &&
           sample->red * 10U >= sample->ir * 5U &&
           sample->red * 10U <= sample->ir * 12U;
}

/*
 * 手指确认后使用首次有效样本形成参考直流量。只要 RED/IR 没有饱和，
 * 且仍处于参考值附近，就继续认为手指存在。这样正常的接触压力变化
 * 不会被固定阈值误判为离开，而真正移开时的幅值骤降仍会触发计时。
 */
static bool FingerDetector_IsTrackingSample(const FingerDetector *detector,
                                             const MAX30102_Sample *sample)
{
    if (detector->present == 0U || detector->reference_red == 0U ||
        detector->reference_ir == 0U || sample->red >= FINGER_NEAR_SATURATION ||
        sample->ir >= FINGER_NEAR_SATURATION)
    {
        return false;
    }

    return sample->red * 100U >=
               detector->reference_red * FINGER_TRACK_MIN_PERCENT &&
           sample->red * 100U <=
               detector->reference_red * FINGER_TRACK_MAX_PERCENT &&
           sample->ir * 100U >=
               detector->reference_ir * FINGER_TRACK_MIN_PERCENT &&
           sample->ir * 100U <=
               detector->reference_ir * FINGER_TRACK_MAX_PERCENT &&
           sample->red * 10U >= sample->ir * FINGER_TRACK_RATIO_MIN_TENTHS &&
           sample->red * 10U <= sample->ir * FINGER_TRACK_RATIO_MAX_TENTHS;
}

/*
 * 用一阶慢速整数滤波更新手指直流参考值。reference是旧参考值，
 * sample是当前严格有效样本，返回值用于下一帧跟踪范围判断。
 */
static uint32_t FingerDetector_FilterReference(uint32_t reference,
                                               uint32_t sample)
{
    if (sample >= reference)
    {
        return reference +
               ((sample - reference) >> FINGER_REFERENCE_FILTER_SHIFT);
    }

    return reference - ((reference - sample) >> FINGER_REFERENCE_FILTER_SHIFT);
}

/* 将手指状态机恢复为Finger OFF，并清除所有连续帧和参考值。 */
void FingerDetector_Init(FingerDetector *detector)
{
    if (detector == 0)
    {
        return;
    }

    detector->valid_frames = 0U;
    detector->invalid_frames = 0U;
    detector->present = 0U;
    detector->reference_red = 0U;
    detector->reference_ir = 0U;
}

/*
 * 先计算严格判据，再在Finger ON期间计算宽松跟踪判据；最后通过
 * 3帧进入和500帧离开迟滞更新状态。
 */
uint8_t FingerDetector_Process(FingerDetector *detector,
                               const MAX30102_Sample *sample,
                               bool *sample_valid)
{
    bool strict_valid; /* 当前样本是否满足初始三重固定阈值。 */
    bool tracking_valid; /* 当前样本是否满足严格或动态跟踪阈值。 */

    if (detector == 0 || sample == 0)
    {
        if (sample_valid != 0)
        {
            *sample_valid = false;
        }
        return 0U;
    }

    strict_valid = FingerDetector_IsValidSample(sample);
    tracking_valid =
        strict_valid || FingerDetector_IsTrackingSample(detector, sample);
    if (sample_valid != 0)
    {
        *sample_valid = tracking_valid;
    }

    if (tracking_valid)
    {
        /* 连续 3 个有效样本后才确认手指放入。 */
        detector->invalid_frames = 0U;
        if (detector->present == 0U)
        {
            if (detector->valid_frames < FINGER_VALID_FRAMES)
            {
                ++detector->valid_frames;
            }
            if (detector->valid_frames == FINGER_VALID_FRAMES)
            {
                detector->present = 1U;
                detector->reference_red = sample->red;
                detector->reference_ir = sample->ir;
            }
        }
        else if (strict_valid)
        {
            /*
             * 仅用严格有效样本缓慢更新参考值，避免移开手指时参考值
             * 跟随环境光一路下降。
             */
            detector->reference_red = FingerDetector_FilterReference(
                detector->reference_red, sample->red);
            detector->reference_ir = FingerDetector_FilterReference(
                detector->reference_ir, sample->ir);
        }
    }
    else
    {
        /* 100 Hz 下连续 500 个无效样本约为 5 秒，之后确认移开。 */
        detector->valid_frames = 0U;
        if (detector->invalid_frames < FINGER_INVALID_FRAMES)
        {
            ++detector->invalid_frames;
        }
        if (detector->invalid_frames == FINGER_INVALID_FRAMES)
        {
            detector->present = 0U;
            detector->reference_red = 0U;
            detector->reference_ir = 0U;
        }
    }

    if (sample_valid != 0)
    {
        *sample_valid = detector->present != 0U && tracking_valid;
    }

    return detector->present;
}
