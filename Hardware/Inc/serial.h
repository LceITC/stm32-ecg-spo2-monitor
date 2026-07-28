#ifndef SERIAL_IT_TEST_SERIAL_H
#define SERIAL_IT_TEST_SERIAL_H

#include "stm32f10x.h"
#include <stdbool.h>

typedef enum
{
    SERIAL_ECG_SOURCE_AD8232 = 0,
    SERIAL_ECG_SOURCE_DIRECT
} Serial_EcgSource;

/* USART1：PA9=TX、PA10=RX、115200 8N1。 */
void My_Serial_Init(void);
bool Serial_PollEcgSource(Serial_EcgSource *source);

/* 严格发送 ECG_Val,SpO2,HR,IR\r\n，不附加任何调试字段。 */
void Serial_SendMonitorFrame(uint16_t ecg, uint8_t spo2, uint16_t heart_rate,
                             uint32_t ir);

#endif
