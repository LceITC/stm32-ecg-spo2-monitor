#include "MAX30102.h"
#include "delay.h"

/* 本驱动使用 PB10/PB11 开漏输出模拟独立 I2C 总线。 */
#define SCL GPIO_Pin_10
#define SDA GPIO_Pin_11

#define REG_INTR_STATUS_1 0x00U
#define REG_INTR_STATUS_2 0x01U
#define REG_INTR_ENABLE_1 0x02U
#define REG_INTR_ENABLE_2 0x03U
#define REG_FIFO_WR_PTR 0x04U
#define REG_OVF_COUNTER 0x05U
#define REG_FIFO_RD_PTR 0x06U
#define REG_FIFO_DATA 0x07U
#define REG_FIFO_CONFIG 0x08U
#define REG_MODE_CONFIG 0x09U
#define REG_SPO2_CONFIG 0x0AU
#define REG_LED1_PA 0x0CU
#define REG_LED2_PA 0x0DU
#define REG_PILOT_PA 0x10U
#define REG_PART_ID 0xFFU

#define MODE_RESET 0x40U
#define MODE_SPO2 0x03U
#define INTR_PPG_READY 0x40U
#define FIFO_POINTER_MASK 0x1FU
#define RESET_TIMEOUT_MS 100U

static void d(void)
{
    DelayUs(2U);
}

static void cl(uint8_t x)
{
    GPIO_WriteBit(GPIOB, SCL, x ? Bit_SET : Bit_RESET);
}

static void da(uint8_t x)
{
    GPIO_WriteBit(GPIOB, SDA, x ? Bit_SET : Bit_RESET);
}

static uint8_t rd(void)
{
    return GPIO_ReadInputDataBit(GPIOB, SDA) == Bit_SET;
}

static void st(void)
{
    da(1U);
    cl(1U);
    d();
    da(0U);
    d();
    cl(0U);
}

static void sp(void)
{
    da(0U);
    cl(1U);
    d();
    da(1U);
    d();
}

static bool wb(uint8_t x)
{
    int8_t i;

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

static uint8_t rb(bool ack)
{
    int8_t i;
    uint8_t x = 0U;

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

static bool wr(uint8_t r, uint8_t x)
{
    bool ok;

    st();
    ok = wb(MAX30102_WRITE_ADDRESS) && wb(r) && wb(x);
    sp();

    return ok;
}

static void recover_bus(void)
{
    uint8_t pulse;

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

static bool rr(uint8_t r, uint8_t *p, uint8_t n)
{
    uint8_t i;
    bool ok;

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

static bool clear_fifo(void)
{
    return wr(REG_FIFO_WR_PTR, 0U) && wr(REG_OVF_COUNTER, 0U) &&
           wr(REG_FIFO_RD_PTR, 0U);
}

static bool register_equals(uint8_t reg, uint8_t expected)
{
    uint8_t actual;

    return rr(reg, &actual, 1U) && actual == expected;
}

void MAX30102_GPIO_Init(void)
{
    GPIO_InitTypeDef g;

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

MAX30102_InitStatus MAX30102_Init(uint8_t *part_id)
{
    uint8_t id;
    uint8_t mode;
    uint32_t reset_start;

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

bool MAX30102_DataReady(void)
{
    return GPIO_ReadInputDataBit(MAX30102_INT_PORT, MAX30102_INT_PIN) ==
           Bit_RESET;
}

bool MAX30102_ReadSample(MAX30102_Sample *s)
{
    uint8_t b[6];
    uint8_t status;
    uint8_t pointers[3];
    uint8_t unread;

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
