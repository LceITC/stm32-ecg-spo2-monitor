#include "ecg_filter.h"
#include "ecg_hr.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846
#define TEST_ADC_CENTER 2048.0
#define TEST_ADC_AMPLITUDE 620.0

static unsigned int test_count;
static unsigned int failure_count;

static void expect_range(const char *name, uint16_t actual, uint16_t minimum,
                         uint16_t maximum)
{
    ++test_count;
    if (actual < minimum || actual > maximum)
    {
        ++failure_count;
        printf("FAIL %-34s actual=%u expected=%u..%u\n", name, actual, minimum,
               maximum);
        return;
    }

    printf("PASS %-34s actual=%u\n", name, actual);
}

static void expect_equal(const char *name, uint16_t actual, uint16_t expected)
{
    expect_range(name, actual, expected, expected);
}

static uint16_t sine_sample(double frequency_hz, uint32_t sample_number)
{
    double phase = 2.0 * TEST_PI * frequency_hz * (double)sample_number /
                   (double)ECG_HR_SAMPLE_RATE_HZ;
    double value = TEST_ADC_CENTER + TEST_ADC_AMPLITUDE * sin(phase);

    return (uint16_t)lround(value);
}

/*
 * 生成简化的60 BPM ECG：窄R峰幅度150码，较宽T波幅度115码。
 * 旧max-50阈值为2100，T波2115会被误检；新比例阈值为2120。
 */
static uint16_t ecg_like_sample(uint32_t sample_number, int strong_t_wave)
{
    uint32_t phase = sample_number % 100U;

    if (phase == 8U || phase == 12U)
    {
        return 2040U;
    }
    if (phase == 9U || phase == 11U)
    {
        return 2100U;
    }
    if (phase == 10U)
    {
        return 2150U;
    }

    if (phase == 45U || phase == 49U)
    {
        return 2040U;
    }
    if (phase == 46U || phase == 48U)
    {
        return strong_t_wave ? 2100U : 2080U;
    }
    if (phase == 47U)
    {
        return strong_t_wave ? 2140U : 2115U;
    }

    return 2000U;
}

static uint16_t feed_filtered_sine(ECG_HRDetector *detector, double frequency,
                                   uint32_t sample_count)
{
    ECG_Filter filter;
    uint32_t index;
    uint16_t filtered = 0U;
    uint16_t heart_rate = 0U;

    ECG_Filter_Init(&filter);
    for (index = 0U; index < sample_count; ++index)
    {
        filtered = ECG_Filter_Process(&filter, sine_sample(frequency, index));
        heart_rate = ECG_HR_Process(detector, filtered);
    }

    return heart_rate;
}

static void test_one_hz(void)
{
    ECG_HRDetector detector;

    ECG_HR_Init(&detector);
    expect_range("1 Hz sine gives about 60 BPM",
                 feed_filtered_sine(&detector, 1.0, 800U), 59U, 61U);
}

static void test_two_hz(void)
{
    ECG_HRDetector detector;

    ECG_HR_Init(&detector);
    expect_range("2 Hz sine gives about 120 BPM",
                 feed_filtered_sine(&detector, 2.0, 800U), 118U, 122U);
}

static void test_filter_preserves_one_hz(void)
{
    ECG_Filter filter;
    uint32_t index;
    uint16_t filtered;
    uint16_t minimum = 4095U;
    uint16_t maximum = 0U;

    ECG_Filter_Init(&filter);
    for (index = 0U; index < 500U; ++index)
    {
        filtered = ECG_Filter_Process(&filter, sine_sample(1.0, index));
        if (index >= 100U)
        {
            if (filtered < minimum)
            {
                minimum = filtered;
            }
            if (filtered > maximum)
            {
                maximum = filtered;
            }
        }
    }

    expect_range("1 Hz filtered peak-to-peak", (uint16_t)(maximum - minimum),
                 1100U, 1300U);
}

static void test_low_amplitude_noise(void)
{
    ECG_HRDetector detector;
    uint32_t index;
    uint16_t heart_rate = 0U;

    ECG_HR_Init(&detector);
    for (index = 0U; index < 600U; ++index)
    {
        uint16_t sample = (uint16_t)(2048 + ((int32_t)(index % 7U) - 3) * 4);

        heart_rate = ECG_HR_Process(&detector, sample);
    }

    expect_equal("low amplitude noise gives HR=0", heart_rate, 0U);
}

static void test_t_wave_rejection(void)
{
    ECG_HRDetector detector;
    uint32_t index;
    uint16_t heart_rate = 0U;

    ECG_HR_Init(&detector);
    for (index = 0U; index < 800U; ++index)
    {
        heart_rate = ECG_HR_Process(&detector,
                                    ecg_like_sample(index, 0));
    }

    expect_range("T wave below proportional threshold", heart_rate, 59U,
                 61U);
}

static void test_strong_t_wave_after_rhythm_lock(void)
{
    ECG_HRDetector detector;
    uint32_t index;
    uint16_t heart_rate = 0U;

    ECG_HR_Init(&detector);
    for (index = 0U; index < 500U; ++index)
    {
        heart_rate = ECG_HR_Process(&detector,
                                    ecg_like_sample(index, 0));
    }
    expect_range("HR valid before strong T wave", heart_rate, 59U, 61U);

    for (; index < 1000U; ++index)
    {
        heart_rate = ECG_HR_Process(&detector,
                                    ecg_like_sample(index, 1));
    }
    expect_range("RR consistency rejects strong T wave", heart_rate, 59U,
                 61U);
}

static void test_too_short_intervals(void)
{
    ECG_HRDetector detector;
    uint32_t index;
    uint16_t heart_rate = 0U;

    ECG_HR_Init(&detector);
    for (index = 0U; index < 800U; ++index)
    {
        uint16_t sample = index % 20U == 5U ? 2600U : 2000U;

        heart_rate = ECG_HR_Process(&detector, sample);
    }

    expect_equal("200 ms RR candidates are rejected", heart_rate, 0U);
}

static void test_timeout_invalidates_hr(void)
{
    ECG_HRDetector detector;
    uint32_t index;
    uint16_t heart_rate;

    ECG_HR_Init(&detector);
    heart_rate = feed_filtered_sine(&detector, 1.0, 600U);
    expect_range("HR valid before timeout", heart_rate, 59U, 61U);

    for (index = 0U; index < 200U; ++index)
    {
        heart_rate = ECG_HR_Process(&detector, 2048U);
    }
    expect_equal("more than 1500 ms without R peak", heart_rate, 0U);
}

static void test_source_reset(void)
{
    ECG_HRDetector detector;
    ECG_Filter filter;
    uint32_t index;
    uint16_t heart_rate;

    ECG_HR_Init(&detector);
    heart_rate = feed_filtered_sine(&detector, 1.0, 600U);
    expect_range("HR valid before source reset", heart_rate, 59U, 61U);

    ECG_HR_Init(&detector);
    ECG_Filter_Init(&filter);
    expect_equal("source reset clears old HR", ECG_HR_GetValue(&detector), 0U);

    heart_rate = 0U;
    for (index = 0U; index < 250U; ++index)
    {
        uint16_t filtered =
            ECG_Filter_Process(&filter, sine_sample(1.0, index));

        heart_rate = ECG_HR_Process(&detector, filtered);
    }
    expect_equal("first peak after reset is baseline", heart_rate, 0U);
}

static void test_adc_saturation(void)
{
    ECG_HRDetector detector;
    uint16_t heart_rate;

    ECG_HR_Init(&detector);
    heart_rate = feed_filtered_sine(&detector, 1.0, 600U);
    expect_range("HR valid before ADC saturation", heart_rate, 59U, 61U);
    expect_equal("ADC saturation clears HR", ECG_HR_Process(&detector, 4095U),
                 0U);
}

static void test_adc_near_rail(void)
{
    ECG_HRDetector detector;
    uint16_t heart_rate;

    ECG_HR_Init(&detector);
    heart_rate = feed_filtered_sine(&detector, 1.0, 600U);
    expect_range("HR valid before near-rail input", heart_rate, 59U, 61U);
    expect_equal("ADC near 3.3V clears HR",
                 ECG_HR_Process(&detector, 4000U), 0U);
}

static void test_sample_counter_wrap(void)
{
    ECG_HRDetector detector;

    ECG_HR_Init(&detector);
    detector.sample_index = UINT32_MAX - 250U;
    expect_range("sample counter wrap keeps RR valid",
                 feed_filtered_sine(&detector, 1.0, 800U), 59U, 61U);
}

int main(void)
{
    test_one_hz();
    test_two_hz();
    test_filter_preserves_one_hz();
    test_low_amplitude_noise();
    test_t_wave_rejection();
    test_strong_t_wave_after_rhythm_lock();
    test_too_short_intervals();
    test_timeout_invalidates_hr();
    test_source_reset();
    test_adc_saturation();
    test_adc_near_rail();
    test_sample_counter_wrap();

    printf("\n%u checks, %u failures\n", test_count, failure_count);
    return failure_count == 0U ? 0 : 1;
}
