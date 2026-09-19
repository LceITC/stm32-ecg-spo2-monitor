#ifndef MAX30102_H
#define MAX30102_H

#include "stm32f10x.h"
#include <stdbool.h>

/* MAX30102的7位地址为0x57，以下地址包含最低读写位。 */
#define MAX30102_WRITE_ADDRESS 0xAEU /* 软件I2C写操作发送的8位地址。 */
#define MAX30102_READ_ADDRESS 0xAFU  /* 软件I2C读操作发送的8位地址。 */
#define MAX30102_PART_ID 0x15U       /* 芯片PART_ID寄存器的正确值。 */

/*
 * FIFO：不平均、不回卷；SpO2：4096 nA、100 Hz、18 位；
 * LED1/LED2 电流使用已通过当前硬件验证的 0x1F。
 */
#define MAX30102_FIFO_CONFIG 0x0FU /* FIFO不平均、满后停止写入。 */
#define MAX30102_SPO2_CONFIG 0x27U /* 4096nA、100Hz、18位脉宽。 */
#define MAX30102_LED_CURRENT 0x1FU /* RED和IR发光二极管驱动电流码。 */
#define MAX30102_ADC_MAX 0x3FFFFU  /* 18位ADC最大原始值262143。 */

/* MAX30102 独占 PB10/PB11 软件 I2C，INT 固定为 PB12 低有效。 */
#define MAX30102_INT_PORT GPIOB       /* 数据就绪中断所在GPIO端口。 */
#define MAX30102_INT_PIN GPIO_Pin_12  /* PB12，低电平表示中断有效。 */

/* FIFO 中每组样本依次包含 18 位 RED 和 18 位 IR。 */
typedef struct
{
    uint32_t red; /* 红光通道18位原始ADC值。 */
    uint32_t ir;  /* 红外通道18位原始ADC值。 */
} MAX30102_Sample;

/* 初始化结果用于区分总线故障和芯片型号不匹配。 */
typedef enum
{
    MAX30102_INIT_OK = 0,          /* 应答、ID和寄存器回读均正确。 */
    MAX30102_INIT_I2C_ERROR,       /* 软件I2C无应答或寄存器访问失败。 */
    MAX30102_INIT_PART_ID_ERROR    /* 总线正常但PART_ID不是0x15。 */
} MAX30102_InitStatus;

/* 配置PB10/PB11开漏软件I2C和PB12上拉输入。 */
void MAX30102_GPIO_Init(void);

/* 复位芯片、读取part_id、配置100Hz SpO2模式并校验寄存器。 */
MAX30102_InitStatus MAX30102_Init(uint8_t *part_id);

/* 返回PB12是否为低电平，即MAX30102是否报告可处理事件。 */
bool MAX30102_DataReady(void);

/* 从FIFO读取最旧的一组6字节数据并解析为18位RED/IR。 */
bool MAX30102_ReadSample(MAX30102_Sample *sample);

#endif
