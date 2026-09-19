#include "OLED.h"
#include "OLED_Font.h"
#include "stm32f10x.h"
#include <string.h>

/* SSD1306固定参数：128x64像素、8个页、7位地址0x3C（写地址0x78）。 */
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U
#define OLED_PAGE_COUNT 8U
#define OLED_I2C_WRITE_ADDRESS 0x78U
#define OLED_REFRESH_CHUNK_SIZE 32U

/* PB8为SCL，PB9为SDA；两根线均使用开漏输出。 */
#define OLED_W_SCL(level) \
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(level))
#define OLED_W_SDA(level) \
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(level))

/*
 * OLED显存按SSD1306页格式保存：每个元素对应一列的8个垂直像素。
 * 1024字节使用静态存储，不依赖malloc。
 */
static uint8_t oled_buffer[OLED_PAGE_COUNT][OLED_WIDTH];

/* dirty_pages的每一位对应一个需要发送的显存页。 */
static uint8_t dirty_pages;

/* 分段刷新状态：当前页每次只发送32字节，四次完成一个128字节页。 */
static uint8_t refresh_page;
static uint8_t refresh_offset;
static bool refresh_active;
static bool refresh_page_changed;

/* 初始化PB8/PB9为50MHz开漏输出，并释放两根I2C总线。 */
static void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);

    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/* 产生软件I2C起始条件。 */
static void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/* 产生软件I2C停止条件。 */
static void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/*
 * 按高位在前发送一个字节。OLED总线不读取应答，
 * 第9个时钟只用于完成应答时隙。
 */
static void OLED_I2C_SendByte(uint8_t byte)
{
    uint8_t bit_index;

    for (bit_index = 0U; bit_index < 8U; ++bit_index)
    {
        OLED_W_SDA((byte & (uint8_t)(0x80U >> bit_index)) != 0U);
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }

    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SCL(0);
}

/* 在一次I2C事务中连续发送多个SSD1306命令。 */
static void OLED_SendCommands(const uint8_t *commands, uint8_t count)
{
    uint8_t index;

    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDRESS);
    OLED_I2C_SendByte(0x00U);
    for (index = 0U; index < count; ++index)
    {
        OLED_I2C_SendByte(commands[index]);
    }
    OLED_I2C_Stop();
}

/* 在一次I2C事务中批量发送显存数据，避免每个字节重复START/STOP。 */
static void OLED_SendData(const uint8_t *data, uint8_t count)
{
    uint8_t index;

    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDRESS);
    OLED_I2C_SendByte(0x40U);
    for (index = 0U; index < count; ++index)
    {
        OLED_I2C_SendByte(data[index]);
    }
    OLED_I2C_Stop();
}

/* 标记指定显存页发生变化；正在发送该页时安排下一轮重新发送。 */
static void OLED_MarkPageDirty(uint8_t page)
{
    dirty_pages |= (uint8_t)(1U << page);
    if (refresh_active && refresh_page == page)
    {
        refresh_page_changed = true;
    }
}

/* 仅在字节实际变化时修改显存并标记页，减少无意义的I2C传输。 */
static void OLED_SetBufferByte(uint8_t page, uint8_t x, uint8_t value)
{
    if (oled_buffer[page][x] != value)
    {
        oled_buffer[page][x] = value;
        OLED_MarkPageDirty(page);
    }
}

/* 返回无符号十进制/十六进制格式化所需的整数幂。 */
static uint32_t OLED_Pow(uint32_t base, uint32_t exponent)
{
    uint32_t result = 1U;

    while (exponent > 0U)
    {
        result *= base;
        --exponent;
    }

    return result;
}

void OLED_Clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
    OLED_RequestFullRefresh();
}

void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint16_t x_end;
    uint16_t y_end;
    uint8_t page;
    uint8_t column;

    if (width == 0U || height == 0U || x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        return;
    }

    x_end = (uint16_t)x + width;
    y_end = (uint16_t)y + height;
    if (x_end > OLED_WIDTH)
    {
        x_end = OLED_WIDTH;
    }
    if (y_end > OLED_HEIGHT)
    {
        y_end = OLED_HEIGHT;
    }

    for (page = (uint8_t)(y / 8U);
         page <= (uint8_t)((y_end - 1U) / 8U);
         ++page)
    {
        uint8_t bit;
        uint8_t clear_mask = 0U;
        uint8_t page_y = (uint8_t)(page * 8U);

        for (bit = 0U; bit < 8U; ++bit)
        {
            uint8_t pixel_y = (uint8_t)(page_y + bit);
            if (pixel_y >= y && pixel_y < y_end)
            {
                clear_mask |= (uint8_t)(1U << bit);
            }
        }

        for (column = x; column < x_end; ++column)
        {
            OLED_SetBufferByte(page, column,
                               (uint8_t)(oled_buffer[page][column] &
                                         (uint8_t)~clear_mask));
        }
    }
}

void OLED_DrawPoint(uint8_t x, uint8_t y, bool set)
{
    uint8_t page;
    uint8_t mask;
    uint8_t value;

    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        return;
    }

    page = (uint8_t)(y / 8U);
    mask = (uint8_t)(1U << (y % 8U));
    value = oled_buffer[page][x];
    if (set)
    {
        value |= mask;
    }
    else
    {
        value &= (uint8_t)~mask;
    }
    OLED_SetBufferByte(page, x, value);
}

void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    int16_t delta_x = x1 >= x0 ? (int16_t)(x1 - x0)
                               : (int16_t)(x0 - x1);
    int16_t step_x = x0 < x1 ? 1 : -1;
    int16_t delta_y_abs = y1 >= y0 ? (int16_t)(y1 - y0)
                                   : (int16_t)(y0 - y1);
    int16_t delta_y = (int16_t)-delta_y_abs;
    int16_t step_y = y0 < y1 ? 1 : -1;
    int16_t error = (int16_t)(delta_x + delta_y);

    for (;;)
    {
        if (x0 >= 0 && x0 < (int16_t)OLED_WIDTH &&
            y0 >= 0 && y0 < (int16_t)OLED_HEIGHT)
        {
            OLED_DrawPoint((uint8_t)x0, (uint8_t)y0, true);
        }

        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        {
            int16_t twice_error = (int16_t)(2 * error);
            if (twice_error >= delta_y)
            {
                error = (int16_t)(error + delta_y);
                x0 = (int16_t)(x0 + step_x);
            }
            if (twice_error <= delta_x)
            {
                error = (int16_t)(error + delta_x);
                y0 = (int16_t)(y0 + step_y);
            }
        }
    }
}

void OLED_RequestFullRefresh(void)
{
    dirty_pages = 0xFFU;
    if (refresh_active)
    {
        refresh_page_changed = true;
    }
}

bool OLED_RefreshStep(void)
{
    uint8_t cursor_commands[3];
    uint8_t page;

    if (!refresh_active)
    {
        if (dirty_pages == 0U)
        {
            return false;
        }

        for (page = 0U; page < OLED_PAGE_COUNT; ++page)
        {
            if ((dirty_pages & (uint8_t)(1U << page)) != 0U)
            {
                refresh_page = page;
                break;
            }
        }
        refresh_offset = 0U;
        refresh_active = true;
        refresh_page_changed = false;
    }

    cursor_commands[0] = (uint8_t)(0xB0U | refresh_page);
    cursor_commands[1] =
        (uint8_t)(0x10U | ((refresh_offset & 0xF0U) >> 4U));
    cursor_commands[2] = (uint8_t)(refresh_offset & 0x0FU);
    OLED_SendCommands(cursor_commands, 3U);
    OLED_SendData(&oled_buffer[refresh_page][refresh_offset],
                  OLED_REFRESH_CHUNK_SIZE);

    refresh_offset = (uint8_t)(refresh_offset + OLED_REFRESH_CHUNK_SIZE);
    if (refresh_offset >= OLED_WIDTH)
    {
        if (!refresh_page_changed)
        {
            dirty_pages &= (uint8_t)~(uint8_t)(1U << refresh_page);
        }
        refresh_active = false;
    }

    return dirty_pages != 0U || refresh_active;
}

void OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
    uint8_t glyph_index;
    uint8_t buffer_x;
    uint8_t top_page;
    uint8_t index;

    if (line < 1U || line > 4U || column < 1U || column > 16U)
    {
        return;
    }
    if (character < ' ' || character > '~')
    {
        character = ' ';
    }

    glyph_index = (uint8_t)(character - ' ');
    buffer_x = (uint8_t)((column - 1U) * 8U);
    top_page = (uint8_t)((line - 1U) * 2U);
    for (index = 0U; index < 8U; ++index)
    {
        OLED_SetBufferByte(top_page, (uint8_t)(buffer_x + index),
                           OLED_F8x16[glyph_index][index]);
        OLED_SetBufferByte((uint8_t)(top_page + 1U),
                           (uint8_t)(buffer_x + index),
                           OLED_F8x16[glyph_index][index + 8U]);
    }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *string)
{
    uint8_t index;

    if (string == 0)
    {
        return;
    }

    for (index = 0U;
         string[index] != '\0' && (uint16_t)column + index <= 16U;
         ++index)
    {
        OLED_ShowChar(line, (uint8_t)(column + index), string[index]);
    }
}

void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number,
                  uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        OLED_ShowChar(
            line, (uint8_t)(column + index),
            (char)(number / OLED_Pow(10U, length - index - 1U) % 10U + '0'));
    }
}

void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number,
                        uint8_t length)
{
    uint8_t index;
    uint32_t magnitude;

    if (number >= 0)
    {
        OLED_ShowChar(line, column, '+');
        magnitude = (uint32_t)number;
    }
    else
    {
        OLED_ShowChar(line, column, '-');
        magnitude = (uint32_t)(-(number + 1)) + 1U;
    }

    for (index = 0U; index < length; ++index)
    {
        OLED_ShowChar(
            line, (uint8_t)(column + index + 1U),
            (char)(magnitude / OLED_Pow(10U, length - index - 1U) % 10U +
                   '0'));
    }
}

void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number,
                     uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        uint8_t digit =
            (uint8_t)(number / OLED_Pow(16U, length - index - 1U) % 16U);
        char character;

        if (digit < 10U)
        {
            character = (char)('0' + digit);
        }
        else
        {
            character = (char)('A' + (digit - 10U));
        }
        OLED_ShowChar(line, (uint8_t)(column + index), character);
    }
}

void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number,
                     uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        OLED_ShowChar(
            line, (uint8_t)(column + index),
            (char)(number / OLED_Pow(2U, length - index - 1U) % 2U + '0'));
    }
}

void OLED_Init(void)
{
    uint32_t outer_delay;
    uint32_t inner_delay;
    const uint8_t init_commands[] = {
        0xAEU,       /* 关闭显示。 */
        0xD5U, 0x80U, /* 默认时钟分频和振荡频率。 */
        0xA8U, 0x3FU, /* 1/64复用率。 */
        0xD3U, 0x00U, /* 显示偏移为0。 */
        0x40U,       /* 起始行为0。 */
        0xA1U,       /* 列地址从左到右。 */
        0xC8U,       /* COM扫描方向从下到上。 */
        0xDAU, 0x12U, /* 128x64 COM引脚配置。 */
        0x81U, 0xCFU, /* 对比度。 */
        0xD9U, 0xF1U, /* 预充电周期。 */
        0xDBU, 0x30U, /* VCOMH电平。 */
        0xA4U,       /* 使用显存内容。 */
        0xA6U,       /* 正常显示。 */
        0x8DU, 0x14U, /* 开启内部充电泵。 */
        0xAFU        /* 开启显示。 */
    };

    /* 保留原驱动的上电等待，避免OLED电源尚未稳定时写初始化命令。 */
    for (outer_delay = 0U; outer_delay < 1000U; ++outer_delay)
    {
        for (inner_delay = 0U; inner_delay < 1000U; ++inner_delay)
        {
            __NOP();
        }
    }

    OLED_I2C_Init();
    OLED_SendCommands(init_commands, (uint8_t)sizeof(init_commands));

    dirty_pages = 0U;
    refresh_page = 0U;
    refresh_offset = 0U;
    refresh_active = false;
    refresh_page_changed = false;
    OLED_Clear();
}
