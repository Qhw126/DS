/**
  * @file    sensor.c
  * @brief   7路灰度传感器 + OLED 可视化电平显示
  ******************************************************************************
  */
#include "sensor.h"
#include "oled.h"

/* 切换黑白标志 */
uint8_t check = black;

/**
 * @brief  读取指定通道的传感器值
 * @param  channel 通道号 1~7
 * @retval 1=检测到黑线, 0=未检测到
 */
uint8_t digtal(uint8_t channel)
{
    uint8_t value = 0;

    switch(channel)
    {
        case 1: value = DL_GPIO_readPins(Pathfinding_L3_PORT, Pathfinding_L3_PIN) ? 1 : 0; break;
        case 2: value = DL_GPIO_readPins(Pathfinding_L2_PORT, Pathfinding_L2_PIN) ? 1 : 0; break;
        case 3: value = DL_GPIO_readPins(Pathfinding_L1_PORT, Pathfinding_L1_PIN) ? 1 : 0; break;
        case 4: value = DL_GPIO_readPins(Pathfinding_MC_PORT, Pathfinding_MC_PIN) ? 1 : 0; break;
        case 5: value = DL_GPIO_readPins(Pathfinding_R1_PORT, Pathfinding_R1_PIN) ? 1 : 0; break;
        case 6: value = DL_GPIO_readPins(Pathfinding_R2_PORT, Pathfinding_R2_PIN) ? 1 : 0; break;
        case 7: value = DL_GPIO_readPins(Pathfinding_R3_PORT, Pathfinding_R3_PIN) ? 1 : 0; break;
        default: value = 0; break;
    }

    /* 白底黑线模式：传感器低电平=检测到黑线，取反 */
    if(check == white)
    {
        value = !value;
    }

    return value;
}

/*============================================================================
 * OLED 传感器电平可视化 — 显示布局（128×64 OLED）
 *
 *   Y=0:   "<<< L3  L2  L1  MC  R1  R2  R3 >>>"  ← 方向文字 + 传感器编号
 *   Y=12:  " ■   □   ■   ■   □   □   □ "         ← 电平块（■=黑线 □=白底）
 *   Y=25~52: 水平方向条 + 当前位置游标              ← 图形化左右指示
 *   Y=54:  "A:+2.5  O:-80  I:120"                ← PID 数据
 *============================================================================*/
void Sensor_ShowLevels(PID_Position *pid)
{
    /* 12号字体：每个字符 6×12 像素 */
    #define CW  6
    #define FH  12

    /* ---- 传感器条参数 ---- */
    static const char *labels[]  = {"L3", "L2", "L1", "MC", "R1", "R2", "R3"};
    static const uint8_t bar_x[] = {  2,  20,  38,  56,  74,  92, 110 };  /* 每根条左边缘 X */
    #define BAR_W   14          /* 条宽度  */
    #define BAR_HI  18          /* 触发电平条高度 */
    #define BAR_LO   3          /* 未触发电平条高度 */
    #define BAR_BASE 20         /* 条底部 Y */

    /* ---- 方向条参数 ---- */
    #define DIR_BAR_Y0   24     /* 方向条顶部 */
    #define DIR_BAR_Y1   26     /* 方向条底部（3px 厚） */
    #define DIR_BAR_X0    6     /* 方向条左端 */
    #define DIR_BAR_X1  122     /* 方向条右端 */
    #define DIR_CENTER   64     /* 方向条中心（对应传感器 MC） */
    #define DIR_RANGE    58     /* ±10 映射到 ±58 像素 */

    uint8_t i, x, y;

    /*================================================================
     * 第1行 (Y=0): 方向文字箭头 + 传感器编号
     *===============================================================*/
    OLED_ClearArea(0, 0, 128, FH);

    OLED_ShowString(0 * CW, 0, (u8 *)"<<<", FH);   /* 左箭头 */

    for (i = 0; i < 7; i++)
    {
        /* 传感器编号居中于条上 */
        uint8_t lx = (uint8_t)(bar_x[i] + (BAR_W - 2 * CW) / 2);
        if (lx < 2) lx = 2;
        OLED_ShowString(lx, 0, (u8 *)labels[i], FH);
    }

    OLED_ShowString(118, 0, (u8 *)">>>", FH);        /* 右箭头 */

    /*================================================================
     * 第2行 (Y=FH): 电平方块（■=触发 □=未触发）
     *===============================================================*/
    OLED_ClearArea(0, FH, 128, FH * 2);

    for (i = 0; i < 7; i++)
    {
        uint8_t val  = digtal((uint8_t)(i + 1));
        uint8_t bx   = bar_x[i];
        uint8_t by   = (uint8_t)FH;
        uint8_t bar_h = val ? BAR_HI : BAR_LO;

        /* 画填充矩形作为电平条 */
        uint8_t yy;
        for (yy = by; yy < by + bar_h; yy++)
        {
            for (x = bx; x < bx + BAR_W; x++)
            {
                if (val)
                    OLED_DrawPoint(x, yy);
                else
                    OLED_ClearPoint(x, yy);
            }
        }
        /* 条之间空隙清零 */
        for (yy = by; yy < by + BAR_HI; yy++)
        {
            if (bx > 0)      OLED_ClearPoint((uint8_t)(bx - 2), yy);
            if (bx + BAR_W < 127) OLED_ClearPoint((uint8_t)(bx + BAR_W + 1), yy);
        }
    }

    /*================================================================
     * 第3段 (Y=25~52): 图形化方向指示条 + 当前位置游标
     *
     *   水平方向条: ─────●─────  中心 MC=64
     *   刻度标记:    |   |   |   |   |   |   |   (7个传感器位置)
     *   游标圆点:    根据 PID_findway.Actual 移动
     *===============================================================*/
    OLED_ClearArea(0, DIR_BAR_Y0, 128, 55);

    /* 方向条主体线（3px 厚） */
    for (y = DIR_BAR_Y0; y <= DIR_BAR_Y1; y++)
    {
        for (x = DIR_BAR_X0; x <= DIR_BAR_X1; x++)
        {
            OLED_DrawPoint(x, y);
        }
    }

    /* 中心线 ┃ */
    for (y = DIR_BAR_Y0 - 4; y <= DIR_BAR_Y1 + 4; y++)
    {
        OLED_DrawPoint(DIR_CENTER, y);
    }

    /* 传感器位置刻度 ┃ */
    for (i = 0; i < 7; i++)
    {
        /* 传感器位置映射到方向条 X */
        uint8_t tick_x;
        if (i < 3)      tick_x = (uint8_t)(DIR_CENTER + (int)((i - 3) * 15));  /* L1..L3 */
        else if (i > 3) tick_x = (uint8_t)(DIR_CENTER + (int)((i - 3) * 15));  /* R1..R3 */
        else            tick_x = DIR_CENTER;                                    /* MC */

        for (y = DIR_BAR_Y0 - 3; y <= DIR_BAR_Y1 + 3; y++)
        {
            OLED_DrawPoint(tick_x, y);
        }
    }

    /* 当前位置游标 ● （映射 Actual 范围 -10..+10 → DIR_CENTER±DIR_RANGE） */
    {
        float pos = pid->Actual;
        int32_t cursor_x;

        /* 限幅 */
        if (pos >  10.0f) pos =  10.0f;
        if (pos < -10.0f) pos = -10.0f;

        /* Actual 为正 → 偏左(L侧)，映射到方向条左侧 */
        cursor_x = DIR_CENTER - (int32_t)(pos * (float)DIR_RANGE / 10.0f);

        if (cursor_x < 2)      cursor_x = 2;
        if (cursor_x > 126)    cursor_x = 126;

        /* 画游标圆点（半径 4） */
        int16_t dy, dx;
        for (dy = -4; dy <= 4; dy++)
        {
            for (dx = -4; dx <= 4; dx++)
            {
                if (dx * dx + dy * dy <= 16)
                {
                    int16_t px = (int16_t)cursor_x + dx;
                    int16_t py = DIR_BAR_Y0 + dy;
                    if (px >= 0 && px < 128 && py >= 0 && py < 64)
                    {
                        OLED_DrawPoint((uint8_t)px, (uint8_t)py);
                    }
                }
            }
        }

        /* 垂直游标线 */
        for (y = DIR_BAR_Y0 - 3; y <= DIR_BAR_Y1 + 3; y++)
        {
            OLED_DrawPoint((uint8_t)cursor_x, y);
        }
    }

    /* 方向条两端标签 */
    OLED_ShowString(DIR_BAR_X0 - 4,  (uint8_t)(DIR_BAR_Y0 - 4), (u8 *)"L", FH);
    OLED_ShowString(DIR_BAR_X1 - 2,  (uint8_t)(DIR_BAR_Y0 - 4), (u8 *)"R", FH);

    /*================================================================
     * 第4行 (Y=54): PID 数据 — Actual / Out / Integral
     *===============================================================*/
    OLED_ClearArea(0, 54, 128, 64);

    OLED_ShowString(0 * CW, 54, (u8 *)"A:", FH);
    OLED_ShowFloatNum(2 * CW, 54, pid->Actual, 2, 2, FH);

    OLED_ShowString(10 * CW, 54, (u8 *)"O:", FH);
    {
        int32_t out_i = (int32_t)pid->Out;
        if (out_i < 0)
        {
            OLED_ShowChar(12 * CW, 54, '-', FH);
            OLED_ShowNum(13 * CW, 54, (uint32_t)(-out_i), 3, FH);
        }
        else
        {
            OLED_ShowNum(12 * CW, 54, (uint32_t)out_i, 3, FH);
        }
    }

    OLED_ShowString(17 * CW, 54, (u8 *)"I:", FH);
    {
        int32_t int_i = (int32_t)pid->Integral;
        if (int_i < 0)
        {
            OLED_ShowChar(19 * CW, 54, '-', FH);
            OLED_ShowNum(20 * CW, 54, (uint32_t)(-int_i), 3, FH);
        }
        else
        {
            OLED_ShowNum(19 * CW, 54, (uint32_t)int_i, 3, FH);
        }
    }
}
