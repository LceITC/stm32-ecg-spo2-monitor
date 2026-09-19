#ifndef SERIAL_IT_TEST_SERIAL_H
#define SERIAL_IT_TEST_SERIAL_H

#include "MAX30102.h"
#include "stm32f10x.h"
#include <stdbool.h>

/*
 * ============================================================================
 *  串口通信模块 —— 数据类型与函数声明
 *
 *  功能：
 *    TX：以 100 Hz 发送四字段 CSV 数据帧给上位机网页
 *    RX：通过中断接收上位机的 ECG 来源切换命令
 *
 *  串口参数：115200 baud, 8 data bits, 1 stop bit, no parity
 *
 *  上行数据帧格式（100 Hz，每个 ECG 样本发送一帧）：
 *    ecg_value,spo2_value,heart_rate,ir_value\r\n
 *    示例：1840,0,72,714\r\n
 *
 *  下行命令格式（ASCII，以 LF 结束）：
 *    ECG_SRC,AD8232\n  → 切换到 AD8232 PA0
 *    ECG_SRC,DIRECT\n  → 切换到信号发生器 PA2
 *
 *  注意：本模块不负责 MAX30102 的数据采集和血氧计算，
 *  只负责将其结果打包发送。
 * ============================================================================
 */

/*
 * Serial_EcgSource —— 串口命令解析后的 ECG 来源枚举
 *
 * 不直接暴露 ADC 通道编号，上层代码只需处理两个逻辑来源。
 *
 * SERIAL_ECG_SOURCE_AD8232 = 0：网页发送命令 "ECG_SRC,AD8232"
 * SERIAL_ECG_SOURCE_DIRECT：网页发送命令 "ECG_SRC,DIRECT"
 */
typedef enum
{
    SERIAL_ECG_SOURCE_AD8232 = 0,
    SERIAL_ECG_SOURCE_DIRECT
} Serial_EcgSource;

/*
 * My_Serial_Init —— 初始化 USART1 串口通信
 *
 * 配置 PA9(TX) 为复用推挽输出，PA10(RX) 为上拉输入。
 * 设置 115200 8N1，使能 USART1 接收中断（RXNE）。
 *
 * 调用时机：系统上电时在 main() 中调用一次。
 * 返回值：无。
 */
void My_Serial_Init(void);

/*
 * Serial_PollEcgSource —— 从中断接收队列解析一条 ECG 来源命令
 *
 * 在主循环中轮询调用。从 RX 环形队列中逐字节消费，忽略 CR，
 * 以 LF 作为命令结束符。只有完整匹配两条合法命令之一时才更新
 * source 并返回 true。不完整或非法命令直接丢弃。
 *
 * 参数 source：接收解析结果的枚举指针。
 * 返回值：true 表示成功解析到一条合法来源命令；false 表示无完整命令。
 */
bool Serial_PollEcgSource(Serial_EcgSource *source);

/*
 * Serial_SendMaxInitLog —— 发送 MAX30102 初始化验收日志
 *
 * 在 MAX30102 初始化完成后调用一次，向上位机报告传感器状态。
 * 非 CSV 格式，用于调试和验收。
 *
 * 参数 status：MAX30102 初始化状态。
 *     part_id：读取到的传感器 Part ID（仅 status 非 I2C 错误时有效）。
 * 返回值：无。
 */
void Serial_SendMaxInitLog(MAX30102_InitStatus status, uint8_t part_id);

/*
 * Serial_SendMonitorFrame —— 发送四字段 CSV 数据帧
 *
 * 格式：ecg,spo2,hr,ir\r\n
 * 每个字段为十进制整数，无前导零。
 * 在 ECG 主循环中每收到一个 ECG 样本调用一次（100 Hz）。
 * SpO2、HR 和 IR 使用调用时最新的有效缓存值。
 *
 * 参数 ecg：滤波后的 ECG 样本值（12位 ADC）。
 *     spo2：最新的 SpO2 百分比（0 表示无效）。
 *     heart_rate：最新心率（BPM，0 表示无效）。
 *     ir：最新的 MAX30102 IR 通道原始值。
 * 返回值：无。
 */
void Serial_SendMonitorFrame(uint16_t ecg, uint8_t spo2, uint16_t heart_rate,
                             uint32_t ir);

#endif
