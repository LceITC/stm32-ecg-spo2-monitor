/*
 * SpO2 ratio-of-ratios calculation adapted from the Maxim Integrated
 * MAXREFDES117 reference algorithm. The original license permits use,
 * modification, and redistribution with this notice retained.
 *
 * Copyright (C) 2015-2016 Maxim Integrated Products, Inc.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

#include "max30102_spo2.h"

/*
 * 算法流程：去直流 -> 平滑与求导 -> 寻找脉搏谷值 ->
 * 计算 RED/IR 交流直流比 -> 查表得到 SpO2。
 */
#define MA4_SIZE 4
#define HAMMING_SIZE 5
#define MAX_PEAKS 15
#define MAX_RATIOS 5
#define DERIVATIVE_VALID_COUNT                                                 \
    (MAX30102_SPO2_WINDOW_SIZE - HAMMING_SIZE - MA4_SIZE - 2U)
#define SPO2_UPDATE_SAMPLES 100U
#define SPO2_INVALID_CLEAR 25U
#define SPO2_WINDOW_RESET 100U
#define SPO2_STABLE_RESULTS 2U
#define SPO2_MAX_STABLE_DELTA 3U

static const uint16_t hamming[HAMMING_SIZE] = {41U, 276U, 512U, 276U, 41U};

static const uint8_t spo2_table[184] = {
    95,  95,  95,  96,  96,  96,  97,  97,  97,  97,  97,  98,  98,  98,  98,
    98,  99,  99,  99,  99,  99,  99,  99,  99,  100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99,
    99,  99,  99,  99,  99,  99,  99,  98,  98,  98,  98,  98,  98,  97,  97,
    97,  97,  96,  96,  96,  96,  95,  95,  95,  94,  94,  94,  93,  93,  93,
    92,  92,  92,  91,  91,  90,  90,  89,  89,  89,  88,  88,  87,  87,  86,
    86,  85,  85,  84,  84,  83,  82,  82,  81,  81,  80,  80,  79,  78,  78,
    77,  76,  76,  75,  74,  74,  73,  72,  72,  71,  70,  69,  69,  68,  67,
    66,  66,  65,  64,  63,  62,  62,  61,  60,  59,  58,  57,  56,  56,  55,
    54,  53,  52,  51,  50,  49,  48,  47,  46,  45,  44,  43,  42,  41,  40,
    39,  38,  37,  36,  35,  34,  33,  31,  30,  29,  28,  27,  26,  25,  23,
    22,  21,  20,  19,  17,  16,  15,  14,  12,  11,  10,  9,   7,   6,   5,
    3,   2,   1};

static uint32_t red_window[MAX30102_SPO2_WINDOW_SIZE];
static uint32_t ir_window[MAX30102_SPO2_WINDOW_SIZE];
static int32_t ir_work[MAX30102_SPO2_WINDOW_SIZE];
static int32_t red_work[MAX30102_SPO2_WINDOW_SIZE];
static int32_t derivative[MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE];

static uint16_t sample_count;
static uint16_t samples_since_calculation;
static uint8_t invalid_samples;
static uint8_t stable_results;
static uint8_t candidate_spo2;
static uint8_t output_spo2;

/* 以下排序与寻峰函数用于筛选间距合理、幅度较高的脉搏峰。 */
static void sort_ascending(int32_t *values, int32_t count)
{
    int32_t i;
    int32_t j;
    int32_t value;

    for (i = 1; i < count; ++i)
    {
        value = values[i];
        for (j = i; j > 0 && value < values[j - 1]; --j)
        {
            values[j] = values[j - 1];
        }
        values[j] = value;
    }
}

static void sort_indices_descending(const int32_t *values, int32_t *indices,
                                    int32_t count)
{
    int32_t i;
    int32_t j;
    int32_t index;

    for (i = 1; i < count; ++i)
    {
        index = indices[i];
        for (j = i; j > 0 && values[index] > values[indices[j - 1]]; --j)
        {
            indices[j] = indices[j - 1];
        }
        indices[j] = index;
    }
}

static void remove_close_peaks(int32_t *locations, int32_t *peak_count,
                               const int32_t *values, int32_t minimum_distance)
{
    int32_t i;
    int32_t j;
    int32_t old_count;
    int32_t distance;

    sort_indices_descending(values, locations, *peak_count);

    for (i = -1; i < *peak_count; ++i)
    {
        old_count = *peak_count;
        *peak_count = i + 1;

        for (j = i + 1; j < old_count; ++j)
        {
            distance = locations[j] - (i == -1 ? -1 : locations[i]);
            if (distance > minimum_distance || distance < -minimum_distance)
            {
                locations[(*peak_count)++] = locations[j];
            }
        }
    }

    sort_ascending(locations, *peak_count);
}

static void find_peaks_above_height(int32_t *locations, int32_t *peak_count,
                                    const int32_t *values, int32_t count,
                                    int32_t minimum_height)
{
    int32_t i = 1;
    int32_t width;

    *peak_count = 0;
    while (i < count - 1)
    {
        if (values[i] > minimum_height && values[i] > values[i - 1])
        {
            width = 1;
            while (i + width < count && values[i] == values[i + width])
            {
                ++width;
            }

            if (i + width < count && values[i] > values[i + width] &&
                *peak_count < MAX_PEAKS)
            {
                locations[(*peak_count)++] = i;
                i += width + 1;
            }
            else
            {
                i += width;
            }
        }
        else
        {
            ++i;
        }
    }
}

static void find_peaks(int32_t *locations, int32_t *peak_count,
                       const int32_t *values, int32_t count,
                       int32_t minimum_height, int32_t minimum_distance,
                       int32_t maximum_peaks)
{
    find_peaks_above_height(locations, peak_count, values, count,
                            minimum_height);
    remove_close_peaks(locations, peak_count, values, minimum_distance);
    if (*peak_count > maximum_peaks)
    {
        *peak_count = maximum_peaks;
    }
}

static bool calculate_spo2(uint8_t *result)
{
    int32_t derivative_peak_locations[MAX_PEAKS];
    int32_t valley_locations[MAX_PEAKS];
    int32_t exact_valley_locations[MAX_PEAKS];
    int32_t ratios[MAX_RATIOS] = {0, 0, 0, 0, 0};
    uint64_t ir_sum = 0U;
    uint32_t ir_mean;
    int32_t peak_count;
    int32_t exact_valley_count = 0;
    int32_t threshold = 0;
    int32_t ratio_count = 0;
    int32_t i;
    int32_t k;

    /* 计算 IR 直流均值，并得到以零为中心的交流波形。 */
    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE; ++k)
    {
        ir_sum += ir_window[k];
    }
    ir_mean = (uint32_t)(ir_sum / MAX30102_SPO2_WINDOW_SIZE);

    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE; ++k)
    {
        ir_work[k] = (int32_t)ir_window[k] - (int32_t)ir_mean;
    }

    /* 四点平均、差分和 Hamming 平滑用于抑制噪声并突出脉搏边沿。 */
    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE; ++k)
    {
        ir_work[k] =
            (ir_work[k] + ir_work[k + 1] + ir_work[k + 2] + ir_work[k + 3]) / 4;
    }

    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE - 1; ++k)
    {
        derivative[k] = ir_work[k + 1] - ir_work[k];
    }

    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE - 2; ++k)
    {
        derivative[k] = (derivative[k] + derivative[k + 1]) / 2;
    }

    for (i = 0; i < (int32_t)DERIVATIVE_VALID_COUNT; ++i)
    {
        int32_t sum = 0;

        for (k = i; k < i + HAMMING_SIZE; ++k)
        {
            sum -= derivative[k] * (int32_t)hamming[k - i];
        }
        derivative[i] = sum / 1146;
    }

    for (k = 0; k < (int32_t)DERIVATIVE_VALID_COUNT; ++k)
    {
        threshold += derivative[k] > 0 ? derivative[k] : -derivative[k];
    }
    threshold /= (int32_t)DERIVATIVE_VALID_COUNT;

    /* 自适应阈值由导数绝对值均值产生，不依赖固定波形幅度。 */
    find_peaks(derivative_peak_locations, &peak_count, derivative,
               (int32_t)DERIVATIVE_VALID_COUNT, threshold, 8, MAX_RATIOS);

    for (k = 0; k < peak_count; ++k)
    {
        valley_locations[k] = derivative_peak_locations[k] + HAMMING_SIZE / 2;
    }

    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE; ++k)
    {
        ir_work[k] = (int32_t)ir_window[k];
        red_work[k] = (int32_t)red_window[k];
    }

    for (k = 0; k < peak_count; ++k)
    {
        int32_t center = valley_locations[k];
        int32_t minimum = 16777216;
        int32_t minimum_index = -1;

        if (center + 5 < (int32_t)MAX30102_SPO2_WINDOW_SIZE - HAMMING_SIZE &&
            center - 5 > 0)
        {
            for (i = center - 5; i < center + 5; ++i)
            {
                if (ir_work[i] < minimum)
                {
                    minimum = ir_work[i];
                    minimum_index = i;
                }
            }

            if (minimum_index >= 0)
            {
                exact_valley_locations[exact_valley_count++] = minimum_index;
            }
        }
    }

    if (exact_valley_count < 2)
    {
        return false;
    }

    for (k = 0; k < (int32_t)MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE; ++k)
    {
        ir_work[k] =
            (ir_work[k] + ir_work[k + 1] + ir_work[k + 2] + ir_work[k + 3]) / 4;
        red_work[k] = (red_work[k] + red_work[k + 1] + red_work[k + 2] +
                       red_work[k + 3]) /
                      4;
    }

    /* 对相邻脉搏周期分别计算 RED/IR 的交流-直流比值。 */
    for (k = 0; k < exact_valley_count - 1 && ratio_count < MAX_RATIOS; ++k)
    {
        int32_t left = exact_valley_locations[k];
        int32_t right = exact_valley_locations[k + 1];
        int32_t ir_dc_max = -16777216;
        int32_t red_dc_max = -16777216;
        int32_t ir_max_index = left;
        int32_t red_max_index = left;
        int32_t ir_ac;
        int32_t red_ac;
        int64_t numerator;
        int64_t denominator;

        if (right - left <= 10)
        {
            continue;
        }

        for (i = left; i < right; ++i)
        {
            if (ir_work[i] > ir_dc_max)
            {
                ir_dc_max = ir_work[i];
                ir_max_index = i;
            }
            if (red_work[i] > red_dc_max)
            {
                red_dc_max = red_work[i];
                red_max_index = i;
            }
        }

        red_ac = (red_work[right] - red_work[left]) * (red_max_index - left);
        red_ac = red_work[left] + red_ac / (right - left);
        red_ac = red_work[red_max_index] - red_ac;

        ir_ac = (ir_work[right] - ir_work[left]) * (ir_max_index - left);
        ir_ac = ir_work[left] + ir_ac / (right - left);
        ir_ac = ir_work[ir_max_index] - ir_ac;

        numerator = (int64_t)red_ac * ir_dc_max;
        denominator = (int64_t)ir_ac * red_dc_max;
        if (numerator > 0 && denominator > 0)
        {
            int64_t ratio = numerator * 100 / denominator;

            if (ratio > 0 && ratio < 184)
            {
                ratios[ratio_count++] = (int32_t)ratio;
            }
        }
    }

    if (ratio_count == 0)
    {
        return false;
    }

    /* 使用比值中位数抑制单个周期的运动伪影，再查经验表。 */
    sort_ascending(ratios, ratio_count);
    if (ratio_count >= 3)
    {
        int32_t middle = ratio_count / 2;
        int32_t ratio = ratio_count % 2 == 0
                            ? (ratios[middle - 1] + ratios[middle]) / 2
                            : ratios[middle];

        *result = spo2_table[ratio];
    }
    else
    {
        *result = spo2_table[ratios[ratio_count / 2]];
    }

    return *result >= 70U && *result <= 100U;
}

static void clear_window(void)
{
    sample_count = 0U;
    samples_since_calculation = 0U;
}

void MAX30102_SpO2_Init(void)
{
    invalid_samples = 0U;
    stable_results = 0U;
    candidate_spo2 = 0U;
    output_spo2 = 0U;
    clear_window();
}

void MAX30102_SpO2_Reset(void)
{
    MAX30102_SpO2_Init();
}

void MAX30102_SpO2_ProcessSample(uint32_t red, uint32_t ir, bool sample_valid)
{
    uint16_t i;
    uint8_t calculated_spo2;
    uint8_t difference;

    if (!sample_valid)
    {
        /*
         * 允许短暂运动伪影，不因 2~3 个坏点立刻清空有效读数。
         * 连续 250 ms 无效后隐藏结果，连续 1 s 无效后重建窗口。
         */
        if (invalid_samples < SPO2_WINDOW_RESET)
        {
            ++invalid_samples;
        }

        if (invalid_samples >= SPO2_INVALID_CLEAR)
        {
            output_spo2 = 0U;
            stable_results = 0U;
            candidate_spo2 = 0U;
        }

        if (invalid_samples == SPO2_WINDOW_RESET)
        {
            clear_window();
        }
        return;
    }

    invalid_samples = 0U;
    /* 首次填满 500 点后计算，之后每新增 100 点更新一次。 */
    if (sample_count < MAX30102_SPO2_WINDOW_SIZE)
    {
        red_window[sample_count] = red;
        ir_window[sample_count] = ir;
        ++sample_count;

        if (sample_count < MAX30102_SPO2_WINDOW_SIZE)
        {
            return;
        }
    }
    else
    {
        for (i = 1U; i < MAX30102_SPO2_WINDOW_SIZE; ++i)
        {
            red_window[i - 1U] = red_window[i];
            ir_window[i - 1U] = ir_window[i];
        }
        red_window[MAX30102_SPO2_WINDOW_SIZE - 1U] = red;
        ir_window[MAX30102_SPO2_WINDOW_SIZE - 1U] = ir;

        if (++samples_since_calculation < SPO2_UPDATE_SAMPLES)
        {
            return;
        }
    }

    samples_since_calculation = 0U;
    if (!calculate_spo2(&calculated_spo2))
    {
        output_spo2 = 0U;
        stable_results = 0U;
        candidate_spo2 = 0U;
        return;
    }

    /* 连续两次结果差值不超过 3，才将血氧值标记为稳定有效。 */
    difference = calculated_spo2 > candidate_spo2
                     ? calculated_spo2 - candidate_spo2
                     : candidate_spo2 - calculated_spo2;
    if (stable_results == 0U || difference > SPO2_MAX_STABLE_DELTA)
    {
        candidate_spo2 = calculated_spo2;
        stable_results = 1U;
        output_spo2 = 0U;
        return;
    }

    candidate_spo2 = calculated_spo2;
    if (stable_results < SPO2_STABLE_RESULTS)
    {
        ++stable_results;
    }
    if (stable_results >= SPO2_STABLE_RESULTS)
    {
        output_spo2 = calculated_spo2;
    }
}

uint8_t MAX30102_SpO2_GetValue(void)
{
    return output_spo2;
}
