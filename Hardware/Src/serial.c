#include "serial.h"
#include "stm32f10x.h"
#include "usart.h"

#define SERIAL_COMMAND_MAX_LENGTH 20U

static char command_buffer[SERIAL_COMMAND_MAX_LENGTH];
static uint8_t command_length;

static bool Serial_CommandEquals(const char *expected)
{
    uint8_t index = 0U;

    while (expected[index] != '\0' && index < command_length)
    {
        if (command_buffer[index] != expected[index])
        {
            return false;
        }
        ++index;
    }
    return index == command_length && expected[index] == '\0';
}

/* 将无符号整数直接追加到发送缓冲区，避免引入 printf/vsprintf。 */
static uint8_t Serial_AppendUnsigned(char *buffer, uint8_t position,
                                     uint32_t value)
{
    char reversed_digits[10];
    uint8_t digit_count = 0U;

    do
    {
        reversed_digits[digit_count] = (char)('0' + value % 10U);
        ++digit_count;
        value /= 10U;
    } while (value != 0U);

    while (digit_count != 0U)
    {
        --digit_count;
        buffer[position] = reversed_digits[digit_count];
        ++position;
    }

    return position;
}

void My_Serial_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
    command_length = 0U;
}

bool Serial_PollEcgSource(Serial_EcgSource *source)
{
    char received;

    if (source == 0)
    {
        return false;
    }

    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {
        received = (char)USART_ReceiveData(USART1);
        if (received == '\r')
        {
            continue;
        }
        if (received == '\n')
        {
            if (Serial_CommandEquals("ECG_SRC,AD8232"))
            {
                *source = SERIAL_ECG_SOURCE_AD8232;
                command_length = 0U;
                return true;
            }
            if (Serial_CommandEquals("ECG_SRC,DIRECT"))
            {
                *source = SERIAL_ECG_SOURCE_DIRECT;
                command_length = 0U;
                return true;
            }
            command_length = 0U;
            continue;
        }

        if (command_length < SERIAL_COMMAND_MAX_LENGTH)
        {
            command_buffer[command_length++] = received;
        }
        else
        {
            command_length = 0U;
        }
    }
    return false;
}

/*
 * 上位机协议固定为四个十进制字段。当前阶段调用方传入 ECG=0、HR=0，
 * 后续接入 AD8232 时无需改变协议或本函数接口。
 */
void Serial_SendMonitorFrame(uint16_t ecg, uint8_t spo2, uint16_t heart_rate,
                             uint32_t ir)
{
    char frame[32];
    uint8_t length = 0U;

    length = Serial_AppendUnsigned(frame, length, ecg);
    frame[length++] = ',';
    length = Serial_AppendUnsigned(frame, length, spo2);
    frame[length++] = ',';
    length = Serial_AppendUnsigned(frame, length, heart_rate);
    frame[length++] = ',';
    length = Serial_AppendUnsigned(frame, length, ir);
    frame[length++] = '\r';
    frame[length++] = '\n';

    My_USART_SendBytes(USART1, (const uint8_t *)frame, length);
}
