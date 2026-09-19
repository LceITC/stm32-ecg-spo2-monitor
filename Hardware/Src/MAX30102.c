#include "MAX30102.h"
#include "delay.h"

/* 本驱动使用 PB10/PB11 开漏输出模拟独立 I2C 总线。 */
#define SCL GPIO_Pin_10 /* PB10，MAX30102软件I2C时钟线。 */
#define SDA GPIO_Pin_11 /* PB11，MAX30102软件I2C数据线。 */

/* MAX30102寄存器地址。 */
#define REG_INTR_STATUS_1 0x00U /* 中断状态1，读取后清除对应标志。 */
#define REG_INTR_STATUS_2 0x01U /* 中断状态2，读取后清除对应标志。 */
#define REG_INTR_ENABLE_1 0x02U /* 中断使能1，本项目只开启PPG Ready。 */
#define REG_INTR_ENABLE_2 0x03U /* 中断使能2，本项目保持关闭。 */
#define REG_FIFO_WR_PTR 0x04U   /* FIFO写指针，低5位有效。 */
#define REG_OVF_COUNTER 0x05U   /* FIFO溢出计数，非0表示数据丢失。 */
#define REG_FIFO_RD_PTR 0x06U   /* FIFO读指针，低5位有效。 */
#define REG_FIFO_DATA 0x07U     /* FIFO数据端口，读取后读指针自动推进。 */
#define REG_FIFO_CONFIG 0x08U   /* FIFO平均、回卷和满阈值配置。 */
#define REG_MODE_CONFIG 0x09U   /* 芯片复位和工作模式配置。 */
#define REG_SPO2_CONFIG 0x0AU   /* ADC量程、采样率和LED脉宽配置。 */
#define REG_LED1_PA 0x0CU       /* LED1红光驱动电流。 */
#define REG_LED2_PA 0x0DU       /* LED2红外驱动电流。 */
#define REG_PILOT_PA 0x10U      /* 导频LED电流，本项目不使用。 */
#define REG_PART_ID 0xFFU       /* 器件型号标识寄存器。 */

#define MODE_RESET 0x40U       /* MODE_CONFIG中的软件复位位。 */
#define MODE_SPO2 0x03U        /* 同时采集RED和IR的SpO2模式。 */
#define INTR_PPG_READY 0x40U    /* 新PPG样本就绪中断使能位。 */
#define FIFO_POINTER_MASK 0x1FU /* FIFO指针只有低5位有效。 */
#define RESET_TIMEOUT_MS 100U   /* 等待芯片自动完成复位的最长时间。 */

/* 每次SCL/SDA电平变化后延时2us，形成约250kHz以内的软件I2C时序。 */
static void d(void)
{
    DelayUs(2U);
}

/* 设置SCL输出；x为0时拉低，非0时释放开漏线由上拉电阻拉高。 */
static void cl(uint8_t x)
{
    GPIO_WriteBit(GPIOB, SCL, x ? Bit_SET : Bit_RESET);
}

/* 设置SDA输出；x为0时拉低，非0时释放开漏数据线。 */
static void da(uint8_t x)
{
    GPIO_WriteBit(GPIOB, SDA, x ? Bit_SET : Bit_RESET);
}

/* 读取SDA总线实际电平，用于接收数据位和从机ACK。 */
static uint8_t rd(void)
{
    return GPIO_ReadInputDataBit(GPIOB, SDA) == Bit_SET;
}

/* 产生I2C START：SCL为高期间将SDA从高拉到低。 */
static void st(void)
{
    da(1U);
    cl(1U);
    d();
    da(0U);
    d();
    cl(0U);
}

/* 产生I2C STOP：SCL为高期间释放SDA从低回到高。 */
static void sp(void)
{
    da(0U);
    cl(1U);
    d();
    da(1U);
    d();
}

/* 最高位优先发送一个字节x，并返回从机是否产生低电平ACK。 */
static bool wb(uint8_t x)
{
    int8_t i; /* 从位7递减到位0的发送位序号。 */

    for (i = 7; i >= 0; --i)
    {
        da((x >> i) & 1U);
        d();
        cl(1U);
        d();
        cl(0U);
    }

    /* 释放 SDA 后读取从机 ACK；低电平表示应答成功。 */
    da(1U);
    d();
    cl(1U);
    d();
    x = rd();
    cl(0U);

    return x == 0U;
}

/*
 * 最高位优先读取一个字节。ack为true时主机在第9个时钟发送ACK，
 * 表示还要继续读取；false发送NACK，表示这是最后一个字节。
 */
static uint8_t rb(bool ack)
{
    int8_t i; /* 从位7递减到位0的接收位序号。 */
    uint8_t x = 0U; /* 按位拼接得到的完整接收字节。 */

    da(1U);
    for (i = 7; i >= 0; --i)
    {
        cl(1U);
        d();
        x |= (uint8_t)(rd() << i);
        cl(0U);
        d();
    }

    da(ack ? 0U : 1U);
    d();
    cl(1U);
    d();
    cl(0U);
    da(1U);

    return x;
}

/* 向寄存器r写入单字节x，任一地址/数据阶段无ACK都返回false。 */
static bool wr(uint8_t r, uint8_t x)
{
    bool ok; /* 三个发送字节是否全部得到从机ACK。 */

    st();
    ok = wb(MAX30102_WRITE_ADDRESS) && wb(r) && wb(x);
    sp();

    return ok;
}

/* 尝试释放被异常事务占用的SDA，并以STOP使总线回到空闲状态。 */
static void recover_bus(void)
{
    uint8_t pulse; /* SDA被拉低时输出的恢复时钟计数。 */

    da(1U);
    cl(1U);
    d();

    if (rd() == 0U)
    {
        /* SDA 被从机占用时输出最多 9 个 SCL 脉冲，释放未完成的字节。 */
        for (pulse = 0U; pulse < 9U; ++pulse)
        {
            cl(0U);
            d();
            cl(1U);
            d();
        }
    }

    sp();
}

/*
 * 从寄存器r开始连续读取n个字节到p。先写寄存器地址，再重复START进入
 * 读方向；除最后一字节外，主机都会回复ACK。
 */
static bool rr(uint8_t r, uint8_t *p, uint8_t n)
{
    uint8_t i; /* p缓冲区的当前写入下标。 */
    bool ok; /* 地址和寄存器阶段是否得到从机ACK。 */

    st();
    ok = wb(MAX30102_WRITE_ADDRESS) && wb(r);
    if (!ok)
    {
        sp();
        return false;
    }

    st();
    ok = wb(MAX30102_READ_ADDRESS);
    for (i = 0U; ok && i < n; ++i)
    {
        p[i] = rb(i + 1U < n);
    }
    sp();

    return ok;
}

/* 将写指针、溢出计数和读指针全部写0，使FIFO重新同步为空。 */
static bool clear_fifo(void)
{
    return wr(REG_FIFO_WR_PTR, 0U) && wr(REG_OVF_COUNTER, 0U) &&
           wr(REG_FIFO_RD_PTR, 0U);
}

/* 回读reg并比较expected，用于确认初始化写入确实生效。 */
static bool register_equals(uint8_t reg, uint8_t expected)
{
    uint8_t actual; /* 从芯片回读的实际寄存器值。 */

    return rr(reg, &actual, 1U) && actual == expected;
}

/* 初始化软件I2C引脚和低有效INT引脚，并恢复可能被占用的总线。 */
void MAX30102_GPIO_Init(void)
{
    GPIO_InitTypeDef g; /* 先复用为I2C开漏输出，再复用为INT上拉输入。 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 开漏模式依靠模块上拉电阻产生高电平，符合 I2C 电气要求。 */
    g.GPIO_Pin = SCL | SDA;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &g);

    /* INT 为低有效输入，内部上拉避免未连接时悬空。 */
    g.GPIO_Pin = MAX30102_INT_PIN;
    g.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &g);

    cl(1U);
    da(1U);
    recover_bus();
}

/*
 * 完成软复位、PART_ID校验、FIFO/SpO2/LED配置和关键寄存器回读。
 * part_id始终由调用者提供存储空间，便于诊断型号不匹配。
 */
MAX30102_InitStatus MAX30102_Init(uint8_t *part_id)
{
    uint8_t id; /* 保存PART_ID，也临时接收启动阶段的状态寄存器。 */
    uint8_t mode; /* 轮询MODE_CONFIG，检查RESET位是否自动清零。 */
    uint32_t reset_start; /* 软复位命令发出时的毫秒时间。 */

    if (part_id == 0)
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    /* 先软复位并等待复位位由芯片自动清零。 */
    if (!wr(REG_MODE_CONFIG, MODE_RESET))
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    reset_start = GetTick();
    do
    {
        Delay(1U);
        if (!rr(REG_MODE_CONFIG, &mode, 1U))
        {
            return MAX30102_INIT_I2C_ERROR;
        }
    } while ((mode & MODE_RESET) != 0U &&
             (uint32_t)(GetTick() - reset_start) < RESET_TIMEOUT_MS);

    if ((mode & MODE_RESET) != 0U)
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    /* Part ID 必须为 0x15，避免把总线噪声误认为有效设备。 */
    if (!rr(REG_PART_ID, &id, 1U))
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    *part_id = id;
    if (id != MAX30102_PART_ID)
    {
        return MAX30102_INIT_PART_ID_ERROR;
    }

    /* 读取两个状态寄存器，清除复位阶段遗留的中断标志。 */
    if (!rr(REG_INTR_STATUS_1, &id, 1U) || !rr(REG_INTR_STATUS_2, &id, 1U))
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    /* 开启 PPG Ready 中断并写入 100 Hz、18 位 SpO2 模式参数。 */
    if (!wr(REG_INTR_ENABLE_1, INTR_PPG_READY) || !wr(REG_INTR_ENABLE_2, 0U) ||
        !clear_fifo() || !wr(REG_FIFO_CONFIG, MAX30102_FIFO_CONFIG) ||
        !wr(REG_SPO2_CONFIG, MAX30102_SPO2_CONFIG) ||
        !wr(REG_LED1_PA, MAX30102_LED_CURRENT) ||
        !wr(REG_LED2_PA, MAX30102_LED_CURRENT) || !wr(REG_PILOT_PA, 0U) ||
        !wr(REG_MODE_CONFIG, MODE_SPO2))
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    /* 回读关键寄存器，防止 I2C 写入异常后继续使用错误参数采样。 */
    if (!register_equals(REG_FIFO_CONFIG, MAX30102_FIFO_CONFIG) ||
        !register_equals(REG_SPO2_CONFIG, MAX30102_SPO2_CONFIG) ||
        !register_equals(REG_LED1_PA, MAX30102_LED_CURRENT) ||
        !register_equals(REG_LED2_PA, MAX30102_LED_CURRENT) ||
        !register_equals(REG_MODE_CONFIG, MODE_SPO2))
    {
        return MAX30102_INIT_I2C_ERROR;
    }

    return MAX30102_INIT_OK;
}

/* 直接读取PB12低有效INT电平，不在此函数中清除任何芯片状态。 */
bool MAX30102_DataReady(void)
{
    return GPIO_ReadInputDataBit(MAX30102_INT_PORT, MAX30102_INT_PIN) ==
           Bit_RESET;
}

/*
 * 读取并检查FIFO指针，再取出最旧的一组RED/IR。返回false表示当前无
 * 未读样本、FIFO溢出、参数无效或任一I2C事务失败。
 */
bool MAX30102_ReadSample(MAX30102_Sample *s)
{
    uint8_t b[6]; /* FIFO依次返回的RED三字节和IR三字节。 */
    uint8_t status; /* 接收状态寄存器并通过读取动作清除中断。 */
    uint8_t pointers[3]; /* 写指针、溢出计数、读指针的连续回读值。 */
    uint8_t unread; /* 根据5位读写指针计算出的未读样本组数。 */

    if (s == 0 || !rr(REG_INTR_STATUS_1, &status, 1U) ||
        !rr(REG_INTR_STATUS_2, &status, 1U) ||
        !rr(REG_FIFO_WR_PTR, pointers, 3U))
    {
        return false;
    }

    /* FIFO 溢出后旧数据的时间顺序不再可靠，因此清空并重新同步。 */
    if ((pointers[1] & FIFO_POINTER_MASK) != 0U)
    {
        clear_fifo();
        return false;
    }

    unread = (uint8_t)((pointers[0] - pointers[2]) & FIFO_POINTER_MASK);
    if (unread == 0U)
    {
        return false;
    }

    /*
     * 每次只取 FIFO 中最旧的一组。调用方持续调用本函数即可逐帧排空，
     * 确保算法不会因为只保留最新值而丢失中间的 10 ms 样本。
     */
    if (!rr(REG_FIFO_DATA, b, 6U))
    {
        return false;
    }

    /* FIFO 顺序为 RED 三字节、IR 三字节，并保留低 18 位。 */
    s->red = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2]) &
             MAX30102_ADC_MAX;
    s->ir = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | (uint32_t)b[5]) &
            MAX30102_ADC_MAX;

    return true;
}
