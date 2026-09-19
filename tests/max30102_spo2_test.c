#include "max30102_spo2.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_SAMPLE_RATE_HZ 100.0
#define TEST_PULSE_HZ 1.2
#define TEST_PI 3.14159265358979323846

static uint32_t ppg_sample(uint32_t dc, double amplitude, uint32_t index)
{
    double phase =
        2.0 * TEST_PI * TEST_PULSE_HZ * (double)index / TEST_SAMPLE_RATE_HZ;
    double pulse = sin(phase) + 0.18 * sin(2.0 * phase);

    return (uint32_t)lround((double)dc + amplitude * pulse);
}

static void expect_equal(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual != expected)
    {
        fprintf(stderr, "FAIL %-42s actual=%lu expected=%lu\n", name,
                (unsigned long)actual, (unsigned long)expected);
        exit(EXIT_FAILURE);
    }
    printf("PASS %-42s actual=%lu\n", name, (unsigned long)actual);
}

static void expect_range(const char *name, uint32_t actual, uint32_t minimum,
                         uint32_t maximum)
{
    if (actual < minimum || actual > maximum)
    {
        fprintf(stderr, "FAIL %-42s actual=%lu expected=%lu..%lu\n", name,
                (unsigned long)actual, (unsigned long)minimum,
                (unsigned long)maximum);
        exit(EXIT_FAILURE);
    }
    printf("PASS %-42s actual=%lu\n", name, (unsigned long)actual);
}

int main(void)
{
    uint32_t index;
    uint8_t stable_spo2;

    MAX30102_SpO2_Init();
    for (index = 0U; index < MAX30102_SPO2_WINDOW_SIZE - 1U; ++index)
    {
        MAX30102_SpO2_ProcessSample(ppg_sample(100000U, 2600.0, index),
                                    ppg_sample(120000U, 5200.0, index), true);
    }
    expect_equal("499 samples keep SpO2 unavailable",
                 MAX30102_SpO2_GetValue(), 0U);

    MAX30102_SpO2_ProcessSample(
        ppg_sample(100000U, 2600.0, MAX30102_SPO2_WINDOW_SIZE - 1U),
        ppg_sample(120000U, 5200.0, MAX30102_SPO2_WINDOW_SIZE - 1U), true);
    stable_spo2 = MAX30102_SpO2_GetValue();
    expect_range("500th sample publishes first valid SpO2", stable_spo2, 70U,
                 100U);

    for (index = 0U; index < 99U; ++index)
    {
        MAX30102_SpO2_ProcessSample(0U, 0U, false);
    }
    expect_equal("990 ms invalid keeps last stable SpO2",
                 MAX30102_SpO2_GetValue(), stable_spo2);

    MAX30102_SpO2_ProcessSample(0U, 0U, false);
    expect_equal("1000 ms invalid clears SpO2", MAX30102_SpO2_GetValue(), 0U);

    return EXIT_SUCCESS;
}
