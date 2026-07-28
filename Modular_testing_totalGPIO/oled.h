#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

#define OLED_CMD  0	 //写命令
#define OLED_DATA 1	 //写数据

#define OLED_I2C_ADDR    0x3C   // OLED I2C 地址（如屏幕不亮，改为 0x3D）
#define OLED_WIDTH       128    // 屏幕宽度（像素）
#define OLED_HEIGHT      64     // 屏幕高度（像素）
#define OLED_PAGES       8      // 页数（高度/8）
#define OLED_GRAM_COLS   128    // 显存列数（= OLED_WIDTH）

typedef unsigned char u8;
typedef unsigned int  u32;

/* ---- 基础操作 ---- */
void OLED_WR_Byte(uint8_t dat, uint8_t mode);

/* ---- 屏幕控制 ---- */
void OLED_Init(void);
void OLED_ColorTurn(u8 i);      // 0=正常, 1=反色
void OLED_DisplayTurn(u8 i);    // 0=正常, 1=旋转180°
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

/* ---- 显存操作 ---- */
void OLED_Refresh(void);        // 将显存刷新到屏幕
void OLED_Clear(void);          // 清显存并刷新
void OLED_ClearArea(u8 x0, u8 y0, u8 x1, u8 y1); // 清除矩形区域

/* ---- 绘图 ---- */
void OLED_DrawPoint(u8 x, u8 y);
void OLED_ClearPoint(u8 x, u8 y);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);
void OLED_DrawCircle(u8 x, u8 y, u8 r);

/* ---- 文字 ---- */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1);
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1);
void OLED_ShowSignedNum(u8 x, u8 y, int32_t num, u8 len, u8 size1);
void OLED_ShowFloatNum(u8 x, u8 y, float num, u8 intLen, u8 decLen, u8 size1);
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1);

/* ---- 图片 ---- */
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[]);

#endif
