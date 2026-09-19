/*
 * ============================================================================
 *  serial.c —— 串口通信模块实现
 *
 *  功能：
 *    上行（TX）：以 100 Hz 发送四字段 CSV 帧（ECG、SpO2、HR、IR）
 *    下行（RX）：通过 USART1 中断接收并解析上位机 ECG 来源切换命令
 *
 *  接收架构：
 *    USART1 RXNE 中断 → 单字节环形队列（64字节） → 主循环轮询解析
 *    中断负责快速保存字节，主循环负责命令解析，互不阻塞。
 *
 *  发送架构：
 *    主循环中直接组装 CSV 帧并调用 My_USART_SendBytes 发送。
 *    不使用中断发送（TX 为纯轮询），避免 TX 中断与 RX 中断的优先级问题。
 * ============================================================================
 */

#include "serial.h"
#include "misc.h"
#include "stm32f10x.h"
#include "usart.h"

/*
 * ============================================================================
 *  模块常量定义
 * ============================================================================
 */

/*
 * SERIAL_COMMAND_MAX_LENGTH：不含换行的命令最大字符数。
 * 最长合法命令 "ECG_SRC,AD8232" 为 15 字符，预留 5 字符余量。
 */
#define SERIAL_COMMAND_MAX_LENGTH 20U

/*
 * SERIAL_RX_BUFFER_SIZE：USART1 接收中断环形队列长度，64 字节。
 * 64 是 2 的幂，允许使用掩码运算快速计算环形索引。
 * 足够缓冲约 5 行完整命令（@115200，每字节约 87μs）。
 */
#define SERIAL_RX_BUFFER_SIZE 64U

/*
 * SERIAL_RX_BUFFER_MASK：快速回绕掩码，值为 SIZE - 1。
 * 用 (index + 1) & MASK 替代 (index + 1) % SIZE，避免除法。
 */
#define SERIAL_RX_BUFFER_MASK (SERIAL_RX_BUFFER_SIZE - 1U)

/*
 * ============================================================================
 *  模块级静态变量
 * ============================================================================
 */

/*
 * command_buffer[]：当前正在拼接的 ASCII 命令字符串，以 null 结尾。
 * 每次收到 LF 时与合法命令比较，匹配则执行对应操作。
 */
static char command_buffer[SERIAL_COMMAND_MAX_LENGTH];

/*
 * command_length：command_buffer 中当前已存入的有效字符数量。
 * 遇到 LF 时重置为 0。缓冲区满（达到 MAX_LENGTH）时也重置。
 */
static uint8_t command_length;

/*
 * rx_buffer[]：ISR 接收字节环形队列。
 * 中断（USART1_IRQHandler）负责写入，主循环负责读取。
 * volatile 修饰，防止编译器优化掉中断与主循环之间的共享访问。
 */
static volatile uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];

/*
 * rx_read_index：主循环下一次读取的队列位置。
 * 仅在主循环中修改（Serial_TakeReceivedByte）。
 */
static volatile uint8_t rx_read_index;

/*
 * rx_write_index：ISR 下一次写入的队列位置。
 * 仅在中 ISR 中修改。
 * 当 rx_write_index == rx_read_index 时表示队列为空。
 */
static volatile uint8_t rx_write_index;

/*
 * ============================================================================
 *  内部辅助函数
 * ============================================================================
 */

/*
 * Serial_NextRxIndex —— 计算环形队列的下一个位置
 *
 * 利用 64 为 2 的幂这一特性，使用位与运算替代求模。
 * (index + 1) & 63 等价于 (index + 1) % 64，但更高效。
 *
 * 参数 index：当前位置。
 * 返回值：下一个位置（0~63 循环）。
 */
static uint8_t Serial_NextRxIndex(uint8_t index)
{
    return (uint8_t)((index + 1U) & SERIAL_RX_BUFFER_MASK);
}

/*
 * Serial_TakeReceivedByte —— 从 ISR 环形队列取出一个字节
 *
 * 队列为空（读写指针相等）或输出指针无效时返回 false。
 * 读操作完成后更新 rx_read_index，释放该槽位供 ISR 复用。
 *
 * 参数 received：接收取出的 ASCII 字符的输出指针。
 * 返回值：true 表示成功取出一个字节；false 表示队列为空或指针无效。
 */
static bool Serial_TakeReceivedByte(char *received)
{
    /*
     * read_index：本次读取位置的局部副本。
     * 使用局部变量确保读指针更新操作的原子性。
     */
    uint8_t read_index;

    /* 空指针保护 */
    if (received == 0)
    {
        return false;
    }

    /* 队列为空：读写指针相等 */
    if (rx_read_index == rx_write_index)
    {
        return false;
    }

    /* 从当前读指针位置取出一个字节 */
    read_index = rx_read_index;
    *received = (char)rx_buffer[read_index];

    /* 读指针前移，释放该槽位 */
    rx_read_index = Serial_NextRxIndex(read_index);

    return true;
}

/*
 * Serial_CommandEquals —— 判断当前已收集命令是否与指定字符串一致
 *
 * 逐字节比较 command_buffer 和 expected，长度和内容必须完全一致。
 * command_buffer 不以 null 结尾，因此用 command_length 控制比较范围。
 *
 * 参数 expected：待比较的 null 结尾 ASCII 字符串。
 * 返回值：true 表示完全一致；false 表示长度或内容不匹配。
 */
static bool Serial_CommandEquals(const char *expected)
{
    /*
     * index：逐字节比较 command_buffer 和 expected 的下标。
     * 同时遍历两个字符串，直到 expected 结束或 command 结束。
     */
    uint8_t index = 0U;

    /* 逐字节比较 */
    while (expected[index] != '\0' && index < command_length)
    {
        if (command_buffer[index] != expected[index])
        {
            return false;
        }
        ++index;
    }

    /*
     * 两个字符串相等的条件：
     * ① 比较完了 command 中的所有字符（index == command_length）
     * ② expected 也刚好结束（expected[index] == '\0'）
     */
    return index == command_length && expected[index] == '\0';
}

/*
 * Serial_AppendUnsigned —— 将无符号整数转换为十进制 ASCII 字符串
 *
 * 自定义整数转字符串函数，避免引入 printf/vsprintf 的庞大代码。
 * 采用"除 10 取余"法：先按从低位到高位的顺序提取数字，
 * 再反转回高位到低位写入 buffer。
 *
 * 参数 buffer：目标字符缓冲区。
 *     position：buffer 中当前写入位置（追加起始偏移）。
 *     value：待转换的无符号 32 位整数。
 * 返回值：写入后的新位置（可供后续继续追加）。
 */
static uint8_t Serial_AppendUnsigned(char *buffer, uint8_t position,
                                     uint32_t value)
{
    /*
     * reversed_digits[10]：临时数组，先按低位到高位保存十进制字符。
     * 最多 10 位（uint32_t 最大值 4294967295）。
     * digit_count：value 转换得到的十进制位数。
     */
    char reversed_digits[10];
    uint8_t digit_count = 0U;

    /*
     * 循环提取每一位数字（从个位开始）。
     * do-while 保证 value = 0 时也能输出一个 "0" 字符。
     */
    do
    {
        reversed_digits[digit_count] = (char)('0' + value % 10U);
        ++digit_count;
        value /= 10U;
    } while (value != 0U);

    /*
     * 反转数字顺序：从高位到低位写入 buffer。
     * 之前保存的是个位、十位、百位...（reversed_digits[0]=个位），
     * 现在从高位（reversed_digits[digit_count-1]）开始写入。
     */
    while (digit_count != 0U)
    {
        --digit_count;
        buffer[position] = reversed_digits[digit_count];
        ++position;
    }

    return position;
}

/*
 * Serial_HexDigit —— 将 4 位二进制数转换为大写十六进制 ASCII 字符
 *
 * 0-9 → '0'-'9'，10-15 → 'A'-'F'
 *
 * 参数 value：4 位二进制数（仅低 4 位有效）。
 * 返回值：对应的大写十六进制字符。
 */
static char Serial_HexDigit(uint8_t value)
{
    /* 只取低 4 位 */
    value &= 0x0FU;

    if (value < 10U)
    {
        return (char)('0' + value);
    }
    return (char)('A' + (value - 10U));
}

/*
 * ============================================================================
 *  公开函数实现
 * ============================================================================
 */

/*
 * My_Serial_Init —— 初始化 USART1 串口通信
 *
 * 配置引脚：PA9 = TX（复用推挽输出），PA10 = RX（上拉输入）
 * 配置参数：115200 baud, 8 data bits, 1 stop bit, no parity
 * 使能中断：USART1 RXNE（接收缓冲区非空中断）
 *
 * 调用后 USART1 立即可用于双向通信。
 * TX 用于 100 Hz 四字段 CSV 发送（轮询模式）。
 * RX 用于接收网页的 ECG 来源切换命令（中断模式）。
 */
void My_Serial_Init(void)
{
    /*
     * gpio：PA9 复用推挽输出和 PA10 上拉输入的 GPIO 配置。
     * nvic：USART1 接收中断的 NVIC 优先级配置。
     * usart：115200、8N1、无流控的串口参数结构体。
     */
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;
    USART_InitTypeDef usart;

    /* 使能 USART1 和 GPIOA 的时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = USART1_TX：复用推挽输出 */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX：浮空输入（内部上拉） */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* USART1 参数：115200 8N1 */
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    /* 初始化命令解析和环形队列状态 */
    command_length = 0U;
    rx_read_index = 0U;
    rx_write_index = 0U;

    /* 配置 USART1 接收中断优先级 */
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* 使能 USART1 和接收中断 */
    USART_Cmd(USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*
 * Serial_PollEcgSource —— 在主循环中轮询解析 ECG 来源切换命令
 *
 * 工作流程：
 *   ① 从中断环形队列中取出一个字节
 *   ② 忽略 CR（\r，0x0D）
 *   ③ 遇到 LF（\n，0x0A）时，将已收集的命令与合法命令比较：
 *      - "ECG_SRC,AD8232" → 设置 source = AD8232
 *      - "ECG_SRC,DIRECT" → 设置 source = DIRECT
 *      - 其他命令 → 丢弃并清空缓冲区
 *   ④ 普通 ASCII 字符追加到 command_buffer（不超过最大长度）
 *   ⑤ 缓冲区满 → 自动清空（防止超长命令耗尽缓冲区）
 *
 * 参数 source：接收解析结果的枚举指针。
 * 返回值：true 表示成功解析到一条合法命令；false 表示无完整命令。
 */
bool Serial_PollEcgSource(Serial_EcgSource *source)
{
    /*
     * received：本轮从中断队列取出的一个 ASCII 字符。
     * 可能是命令字符、CR、LF 或噪声。
     */
    char received;

    /* 空指针保护 */
    if (source == 0)
    {
        return false;
    }

    /*
     * 循环消费队列，直到队列为空或解析到合法命令。
     * 每次调用只返回一条命令（如果队列中有多条，下次调用继续解析）。
     */
    while (Serial_TakeReceivedByte(&received))
    {
        /* ===== 忽略回车符 ===== */
        if (received == '\r')
        {
            continue;
        }

        /* ===== 遇到换行符：检查已收集的命令 ===== */
        if (received == '\n')
        {
            /* 匹配 AD8232 命令 */
            if (Serial_CommandEquals("ECG_SRC,AD8232"))
            {
                *source = SERIAL_ECG_SOURCE_AD8232;
                command_length = 0U;
                return true;
            }

            /* 匹配 DIRECT 命令 */
            if (Serial_CommandEquals("ECG_SRC,DIRECT"))
            {
                *source = SERIAL_ECG_SOURCE_DIRECT;
                command_length = 0U;
                return true;
            }

            /* 不匹配任何合法命令：丢弃并清空缓冲区 */
            command_length = 0U;
            continue;
        }

        /*
         * ===== 普通字符：追加到命令缓冲区 =====
         * 缓冲区未满时追加。
         * 缓冲区已满时清空（可能是噪声或超长命令）。
         */
        if (command_length < SERIAL_COMMAND_MAX_LENGTH)
        {
            command_buffer[command_length++] = received;
        }
        else
        {
            command_length = 0U;
        }
    }

    /* 队列消费完毕，未找到完整命令 */
    return false;
}

/*
 * Serial_SendMaxInitLog —— 发送 MAX30102 初始化验收日志
 *
 * 根据初始化状态输出不同的调试信息：
 *   - I2C 错误：输出 "MAX30102 I2C ERR\r\n"
 *   - Part ID 错误：输出 "MAX30102 Part ID ERR=0xNN\r\n"
 *   - 成功：输出 "MAX30102 Part ID=0xNN\r\n"（NN = 实际 Part ID）
 *
 * 初始化成功后调用一次，用于确认传感器通信正常。
 * 非 CSV 格式，上位机应忽略此消息或显示在日志区域。
 *
 * 参数 status：MAX30102 初始化状态枚举。
 *     part_id：读取到的传感器 Part ID 值。
 * 返回值：无。
 */
void Serial_SendMaxInitLog(MAX30102_InitStatus status, uint8_t part_id)
{
    /* I2C 通信失败：传感器无应答或线路异常 */
    if (status == MAX30102_INIT_I2C_ERROR)
    {
        My_USART_SendString(USART1, "MAX30102 I2C ERR\r\n");
        return;
    }

    /* Part ID 不匹配：读取到的值与预期不符 */
    if (status == MAX30102_INIT_PART_ID_ERROR)
    {
        /*
         * message[]："MAX30102 Part ID ERR=0xNN\r\n"
         * 使用可写数组，在运行时填入实际 Part ID 的十六进制值。
         * NN 的位置在 sizeof(message)-5（高位）和 sizeof(message)-4（低位）。
         */
        char message[] = "MAX30102 Part ID ERR=0x00\r\n";

        message[sizeof(message) - 5U] =
            Serial_HexDigit((uint8_t)(part_id >> 4U));
        message[sizeof(message) - 4U] = Serial_HexDigit(part_id);
        My_USART_SendString(USART1, message);
        return;
    }

    /* 初始化成功：输出实际 Part ID */
    {
        char message[] = "MAX30102 Part ID=0x00\r\n";

        message[sizeof(message) - 5U] =
            Serial_HexDigit((uint8_t)(part_id >> 4U));
        message[sizeof(message) - 4U] = Serial_HexDigit(part_id);
        My_USART_SendString(USART1, message);
    }
}

/*
 * USART1_IRQHandler —— USART1 接收中断服务函数
 *
 * 触发条件：USART1 接收数据寄存器非空（RXNE）。
 *
 * 功能：
 *   ① 从 USART1 数据寄存器读取接收到的字节。
 *   ② 检查环形队列是否已满（下一个写入位置 == 读指针）。
 *   ③ 未满时写入队列并更新写指针；已满时丢弃（防止阻塞）。
 *
 * 溢出处理：
 *   USART1 没有硬件接收 FIFO。如果 CPU 未能及时读取数据寄存器，
 *   下一个到达的字节会导致 ORE（溢出错误）。读取 SR 后再读取 DR
 *   可清除 ORE 标志。正常 RXNE 路径已经完成了这两个读取操作，
 *   此分支仅在启动或异常期间遗留的溢出状态时执行。
 */
void USART1_IRQHandler(void)
{
    /* ===== RXNE 中断：接收一个字节 ===== */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        /*
         * next_write_index：保存当前字节后的下一个写位置。
         * 提前计算，用于判断队列是否满。
         * received：从 USART1 数据寄存器读出的 8 位字节。
         */
        uint8_t next_write_index;
        uint8_t received;

        /* 读取数据寄存器（此操作同时清除 RXNE 标志） */
        received = (uint8_t)USART_ReceiveData(USART1);

        /* 预计算写入后的下一个写指针位置 */
        next_write_index = Serial_NextRxIndex(rx_write_index);

        /*
         * 环形队列未满（下一个位置 != 读指针）时写入。
         * 如果队列满，直接将字节丢弃。
         */
        if (next_write_index != rx_read_index)
        {
            rx_buffer[rx_write_index] = received;
            rx_write_index = next_write_index;
        }
    }

    /*
     * ORE（OverRun Error）处理：
     * 当 USART1 SR 的 ORE 标志被置位时，读取 SR 后再读取 DR
     * 可以清除 ORE 标志。正常 RXNE 路径已经完成了这两个读取操作。
     * 此分支处理 ISR 启动初期或异常中断时遗留的溢出状态。
     */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        /* 读取 DR 以清除 ORE（返回值无用） */
        (void)USART_ReceiveData(USART1);
    }
}

/*
 * Serial_SendMonitorFrame —— 发送四字段 CSV 数据帧
 *
 * 协议：ecg,spo2,hr,ir\r\n
 * 所有字段为十进制无符号整数，无前导零，无额外字段。
 *
 * 示例帧：
 *   1840,0,72,714\r\n
 *
 * 发送时序：ECG 样本率 100 Hz → 每 10ms 发送一帧。
 * 帧最大长度：5(ECG) + 1 + 3(SpO2) + 1 + 3(HR) + 1 + 5(IR) + 2 = 21 字节
 * @115200 baud：21 × 10 bit ≈ 1.8 ms，占 10ms 周期的 18%，可接受。
 *
 * 参数 ecg：滤波后 ECG 样本值（12 位 ADC，0~4095）。
 *     spo2：最新血氧饱和度百分比（0~100，0 表示无效）。
 *     heart_rate：最新心率值（BPM，0~250，0 表示无效）。
 *     ir：最新 MAX30102 IR 通道原始值。
 * 返回值：无。
 */
void Serial_SendMonitorFrame(uint16_t ecg, uint8_t spo2, uint16_t heart_rate,
                             uint32_t ir)
{
    /*
     * frame[32]：发送缓冲区。32 字节足够容纳：
     *   5(ECG最大65535) + 1(逗号) + 3(SpO2最大100)
     *   + 1(逗号) + 3(HR最大250) + 1(逗号) + 5(IR最大65535)
     *   + 2(CRLF) = 21 字节，余量充裕。
     * length：frame 中已经编码的有效字节数。
     */
    char frame[32];
    uint8_t length = 0U;

    /* 字段1：ECG 样本值 */
    length = Serial_AppendUnsigned(frame, length, ecg);
    frame[length++] = ',';

    /* 字段2：SpO2 百分比 */
    length = Serial_AppendUnsigned(frame, length, spo2);
    frame[length++] = ',';

    /* 字段3：心率（BPM） */
    length = Serial_AppendUnsigned(frame, length, heart_rate);
    frame[length++] = ',';

    /* 字段4：IR 原始值 */
    length = Serial_AppendUnsigned(frame, length, ir);

    /* 帧结束符：CRLF */
    frame[length++] = '\r';
    frame[length++] = '\n';
    My_USART_SendBytes(USART1, (const uint8_t *)frame, length);
}
