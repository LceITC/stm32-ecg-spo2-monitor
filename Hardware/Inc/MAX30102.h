#ifndef MAX30102_H
#define MAX30102_H

#include "stm32f10x.h"
#include <stdbool.h>

/* MAX30102 7 位地址为 0x57，以下为包含读写位的 8 位地址。 */
#define MAX30102_WRITE_ADDRESS 0xAEU
#define MAX30102_READ_ADDRESS 0xAFU
#define MAX30102_PART_ID 0x15U

/*
 * FIFO：不平均、不回卷；SpO2：4096 nA、100 Hz、18 位；
 * LED1/LED2 电流使用已通过当前硬件验证的 0x1F。
 */
#define MAX30102_FIFO_CONFIG 0x0FU
#define MAX30102_SPO2_CONFIG 0x27U
#define MAX30102_LED_CURRENT 0x1FU
#define MAX30102_ADC_MAX 0x3FFFFU

/* MAX30102 独占 PB10/PB11 软件 I2C，INT 固定为 PB12 低有效。 */
#define MAX30102_INT_PORT GPIOB
#define MAX30102_INT_PIN GPIO_Pin_12

/* FIFO 中每组样本依次包含 18 位 RED 和 18 位 IR。 */
typedef struct
{
    uint32_t red;
    uint32_t ir;
} MAX30102_Sample;

typedef enum
{
    MAX30102_INIT_OK = 0,
    MAX30102_INIT_I2C_ERROR,
    MAX30102_INIT_PART_ID_ERROR
} MAX30102_InitStatus;

/* 初始化 GPIO、校验 Part ID 并写入采样寄存器。 */
void MAX30102_GPIO_Init(void);
MAX30102_InitStatus MAX30102_Init(uint8_t *part_id);

/* 查询低有效 INT；读取接口每次从 FIFO 取出最旧的一组 RED/IR。 */
bool MAX30102_DataReady(void);
bool MAX30102_ReadSample(MAX30102_Sample *sample);

#endif
