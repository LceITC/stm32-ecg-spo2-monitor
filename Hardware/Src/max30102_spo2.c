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
#define MA4_SIZE 4       /* 去噪时使用的四点滑动平均长度。 */
#define HAMMING_SIZE 5   /* 导数平滑使用的Hamming窗长度。 */
#define MAX_PEAKS 15     /* 寻峰中间阶段允许保存的最大峰数量。 */
#define MAX_RATIOS 5     /* 一个窗口最多参与中值统计的周期比值数。 */
#define DERIVATIVE_VALID_COUNT                                                 \
    (MAX30102_SPO2_WINDOW_SIZE - HAMMING_SIZE - MA4_SIZE - 2U) /* 有效导数点数。 */
#define SPO2_UPDATE_SAMPLES 100U /* 窗口填满后每100点重新计算一次。 */
#define SPO2_WINDOW_RESET 100U /* 连续1秒无效样本后清零并重建窗口。 */
#define SPO2_STABLE_RESULTS 2U /* 明显跳变需要连续两次接近才接受。 */
#define SPO2_MAX_STABLE_DELTA 3U /* 相邻候选结果允许的最大百分比差。 */
#define SPO2_CALCULATION_FAILURE_LIMIT 2U /* 连续失败两次才隐藏旧值。 */

/* 5点对称Hamming权重，总和为1146，使用整数避免浮点计算。 */
static const uint16_t hamming[HAMMING_SIZE] = {41U, 276U, 512U, 276U, 41U};

/* Ratio-of-ratios索引到SpO2百分比的Maxim经验查找表。 */
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

/* 保存最近500点RED原始数据的滑动窗口。 */
static uint32_t red_window[MAX30102_SPO2_WINDOW_SIZE];

/* 保存最近500点IR原始数据的滑动窗口。 */
static uint32_t ir_window[MAX30102_SPO2_WINDOW_SIZE];

/* IR去直流、平滑和周期定位的有符号工作缓冲区。 */
static int32_t ir_work[MAX30102_SPO2_WINDOW_SIZE];

/* RED平滑及周期内交流幅度计算的有符号工作缓冲区。 */
static int32_t red_work[MAX30102_SPO2_WINDOW_SIZE];

/* 保存IR平滑导数，供脉搏谷值位置搜索使用。 */
static int32_t derivative[MAX30102_SPO2_WINDOW_SIZE - MA4_SIZE];

static uint16_t sample_count; /* 初次填充阶段已经保存的有效样本数。 */
static uint16_t samples_since_calculation; /* 距离上次完整计算的新样本数。 */
static uint8_t invalid_samples; /* 当前连续无效样本数，上限100。 */
static uint8_t stable_results; /* 当前候选附近连续一致的结果数量。 */
static uint8_t candidate_spo2; /* 用于确认跳变的最近一次计算值。 */
static uint8_t output_spo2; /* 对外发送和显示的稳定SpO2，0表示无效。 */
static uint8_t calculation_failures; /* 连续未得到有效比值的计算次数。 */

/* 对values原地执行升序插入排序，数据规模很小且无需额外内存。 */
static void sort_ascending(int32_t *values, int32_t count)
{
    int32_t i; /* 从第2个元素开始选择待插入值。 */
    int32_t j; /* 向前寻找插入位置的下标。 */
    int32_t value; /* 当前从数组取出的待插入值。 */

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

/* 按values中对应幅度从大到小排列indices中的峰位置。 */
static void sort_indices_descending(const int32_t *values, int32_t *indices,
                                    int32_t count)
{
    int32_t i; /* 当前待插入的峰索引位置。 */
    int32_t j; /* 向前比较峰幅度的数组下标。 */
    int32_t index; /* 当前待排序峰在values中的原始位置。 */

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

/*
 * 优先保留幅度较大的峰，删除与已保留峰距离不超过minimum_distance的
 * 邻近峰，最后重新按时间升序排列。
 */
static void remove_close_peaks(int32_t *locations, int32_t *peak_count,
                               const int32_t *values, int32_t minimum_distance)
{
    int32_t i; /* 当前已确定保留的峰序号，-1用于首轮整理。 */
    int32_t j; /* 扫描尚未处理峰的位置。 */
    int32_t old_count; /* 本轮过滤开始前的峰总数。 */
    int32_t distance; /* 两个候选峰之间的有符号样本距离。 */

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

/* 搜索高于minimum_height的局部峰，并兼容连续相等的平台峰。 */
static void find_peaks_above_height(int32_t *locations, int32_t *peak_count,
                                    const int32_t *values, int32_t count,
                                    int32_t minimum_height)
{
    int32_t i = 1; /* 当前检查点；首尾点无法同时比较两侧。 */
    int32_t width; /* 平顶峰中连续相等样本的宽度。 */

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

/* 组合高度、最小间距和最大数量限制，输出最终导数峰位置。 */
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

/*
 * 对当前500点窗口执行完整ratio-of-ratios算法。成功时把70~100的整数
 * 百分比写入result；周期不足、交流幅度无效或比值越界时返回false。
 */
static bool calculate_spo2(uint8_t *result)
{
    int32_t derivative_peak_locations[MAX_PEAKS]; /* 平滑导数候选峰位置。 */
    int32_t valley_locations[MAX_PEAKS]; /* 由导数峰粗略换算的IR谷位置。 */
    int32_t exact_valley_locations[MAX_PEAKS]; /* 原始IR附近搜索后的精确谷位置。 */
    int32_t ratios[MAX_RATIOS] = {0, 0, 0, 0, 0}; /* 各脉搏周期RED/IR比值。 */
    uint64_t ir_sum = 0U; /* 500点IR求和，64位避免累加溢出。 */
    uint32_t ir_mean; /* IR窗口直流平均值。 */
    int32_t peak_count; /* 当前找到的导数峰数量。 */
    int32_t exact_valley_count = 0; /* 成功校正到原始IR谷值的数量。 */
    int32_t threshold = 0; /* 导数绝对值均值形成的自适应寻峰阈值。 */
    int32_t ratio_count = 0; /* ratios中已经保存的有效周期数。 */
    int32_t i; /* 内层遍历、搜索和排序使用的通用下标。 */
    int32_t k; /* 外层窗口或周期遍历下标。 */

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
        int32_t sum = 0; /* 当前5点Hamming卷积的加权累加值。 */

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
        int32_t center = valley_locations[k]; /* 导数给出的粗略谷中心。 */
        int32_t minimum = 16777216; /* 搜索范围内当前最小IR值。 */
        int32_t minimum_index = -1; /* 当前最小值位置，-1表示尚未找到。 */

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
        int32_t left = exact_valley_locations[k]; /* 当前周期左侧IR谷位置。 */
        int32_t right = exact_valley_locations[k + 1]; /* 下一IR谷位置。 */
        int32_t ir_dc_max = -16777216; /* 周期内IR最大直流值。 */
        int32_t red_dc_max = -16777216; /* 周期内RED最大直流值。 */
        int32_t ir_max_index = left; /* IR最大值所在样本位置。 */
        int32_t red_max_index = left; /* RED最大值所在样本位置。 */
        int32_t ir_ac; /* IR峰值相对两谷连线的交流幅度。 */
        int32_t red_ac; /* RED峰值相对两谷连线的交流幅度。 */
        int64_t numerator; /* red_ac * ir_dc，使用64位防止溢出。 */
        int64_t denominator; /* ir_ac * red_dc，使用64位防止溢出。 */

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
            int64_t ratio = numerator * 100 / denominator; /* 百分制比值索引。 */

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
        int32_t middle = ratio_count / 2; /* 排序后中间元素的位置。 */
        int32_t ratio = ratio_count % 2 == 0 /* 多周期比值的中位数。 */
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

/* 清空窗口计数，不必擦除数组，因为后续有效样本会从下标0覆盖。 */
static void clear_window(void)
{
    sample_count = 0U;
    samples_since_calculation = 0U;
}

/* 初始化全部窗口和输出状态，确保上电时SpO2为0。 */
void MAX30102_SpO2_Init(void)
{
    invalid_samples = 0U;
    stable_results = 0U;
    candidate_spo2 = 0U;
    output_spo2 = 0U;
    calculation_failures = 0U;
    clear_window();
}

/* 提供运行时语义明确的复位入口，实际复用完整初始化逻辑。 */
void MAX30102_SpO2_Reset(void)
{
    MAX30102_SpO2_Init();
}

/*
 * 消费一个新样本。有效样本用于填充或滑动500点窗口；无效样本只累计
 * 连续时间。窗口满后每100点计算一次，并对失败和明显跳变做迟滞。
 */
void MAX30102_SpO2_ProcessSample(uint32_t red, uint32_t ir, bool sample_valid)
{
    uint16_t i; /* 滑动窗口时把下标1~499前移一位。 */
    uint8_t calculated_spo2; /* 本次完整窗口算法得到的候选百分比。 */
    uint8_t difference; /* calculated_spo2与上次候选值的绝对差。 */

    if (!sample_valid)
    {
        /* 不因短暂运动伪影清空稳定读数，连续1秒无效后再清零并重建窗口。 */
        if (invalid_samples < SPO2_WINDOW_RESET)
        {
            ++invalid_samples;
        }

        if (invalid_samples == SPO2_WINDOW_RESET)
        {
            output_spo2 = 0U;
            stable_results = 0U;
            candidate_spo2 = 0U;
            calculation_failures = 0U;
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
        if (calculation_failures < SPO2_CALCULATION_FAILURE_LIMIT)
        {
            ++calculation_failures;
        }
        if (calculation_failures >= SPO2_CALCULATION_FAILURE_LIMIT)
        {
            output_spo2 = 0U;
            stable_results = 0U;
            candidate_spo2 = 0U;
        }
        return;
    }
    calculation_failures = 0U;

    /*
     * 500点窗口已经覆盖约5秒脉搏。第一个有效结果立即显示，后续仍用
     * 连续结果确认明显跳变；确认期间保留旧值，避免界面闪成“--”。
     */
    difference = calculated_spo2 > candidate_spo2
                     ? calculated_spo2 - candidate_spo2
                     : candidate_spo2 - calculated_spo2;
    if (stable_results == 0U)
    {
        candidate_spo2 = calculated_spo2;
        stable_results = 1U;
        output_spo2 = calculated_spo2;
        return;
    }
    if (difference > SPO2_MAX_STABLE_DELTA)
    {
        candidate_spo2 = calculated_spo2;
        stable_results = 1U;
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

/* 返回对外稳定输出，不触发新的窗口计算或状态变化。 */
uint8_t MAX30102_SpO2_GetValue(void)
{
    return output_spo2;
}
