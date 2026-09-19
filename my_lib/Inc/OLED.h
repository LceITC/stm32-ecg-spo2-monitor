#ifndef __OLED_H
#define __OLED_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 初始化SSD1306和PB8/PB9软件I2C。显示内容先写入静态显存，
 * 再由OLED_RefreshStep()分段发送到屏幕。
 */
void OLED_Init(void);

/* 清空整块128x64显存，并将全部8个页标记为待刷新。 */
void OLED_Clear(void);

/* 清除像素矩形区域；超出屏幕的部分会被自动裁剪。 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height);

/* 在显存中设置或清除一个像素；坐标范围为x=0~127、y=0~63。 */
void OLED_DrawPoint(uint8_t x, uint8_t y, bool set);

/* 使用Bresenham算法绘制线段，屏幕范围外的像素会被裁剪。 */
void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

/* 将全部显存标记为待发送，用于页面切换后的强制完整重绘。 */
void OLED_RequestFullRefresh(void);

/*
 * 最多发送32字节显存数据。返回true表示仍有待刷新数据，
 * 返回false表示本次调用后显存已经与OLED同步。
 */
bool OLED_RefreshStep(void);

/* 以下接口继续使用现有8x16 ASCII字库，行列编号均从1开始。 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
