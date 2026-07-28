#include "oled.h"
#include "oledfont.h"

/* ============================================================================
 *  显存缓冲区 —— 页优先布局 OLED_GRAM[page][column]
 *  与列优先相比：OLED_GRAM[page] 是连续 128 字节，可直接 I2C 批量发送。
 * ========================================================================== */
u8 OLED_GRAM[OLED_PAGES][OLED_GRAM_COLS];

extern void delay_ms(uint32_t ms);

/* ---------- I2C 超时常量 ---------- */
#define OLED_I2C_SHORT_TIMEOUT   100000UL
#define OLED_I2C_LONG_TIMEOUT    800000UL

/* ============================================================================
 *  I2C 总线恢复
 * ========================================================================== */
static void OLED_I2C_Recover(void)
{
    DL_I2C_resetControllerTransfer(OLED_INST);
    if (!(DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        DL_I2C_reset(OLED_INST);
        DL_I2C_enableController(OLED_INST);
    }
}

/* ============================================================================
 *  等待 I2C 空闲 — 返回 0=成功, -1=超时
 * ========================================================================== */
static int OLED_I2C_WaitIdle(uint32_t timeout)
{
    while (!(DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) { OLED_I2C_Recover(); return -1; }
    }
    return 0;
}

/* ============================================================================
 *  OLED_WR_Byte —— 单个字节写入（命令或数据）
 *  增加超时时间，容忍中断打断，不禁止编码器中断
 * ========================================================================== */
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t txData[2];
    txData[0] = mode ? 0x40 : 0x00;
    txData[1] = dat;

    // 使用更长的超时时间，容忍中断打断
    if (OLED_I2C_WaitIdle(OLED_I2C_LONG_TIMEOUT) < 0) return;

    DL_I2C_fillControllerTXFIFO(OLED_INST, txData, 2);
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    OLED_I2C_WaitIdle(OLED_I2C_LONG_TIMEOUT);
}

/* ============================================================================
 *  OLED_WriteBlock —— 单次 I2C 事务连续写入多字节
 *
 *  将 ctrl(0x00=cmd / 0x40=data) + data[] 合并为一次 I2C 传输。
 *  利用 TX FIFO（深度 8）分批填充，避免大栈缓冲。
 *  增加超时时间，容忍中断打断，不禁止编码器中断
 * ========================================================================== */
static void OLED_WriteBlock(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint16_t total;       /* 总共要发的字节数 = ctrl + data */
    uint16_t sent;        /* 已填入 FIFO 的字节数 */
    uint16_t batch;
    uint32_t tout;

    total = len + 1;
    sent  = 0;

    // 使用更长的超时时间，容忍中断打断
    if (OLED_I2C_WaitIdle(OLED_I2C_LONG_TIMEOUT) < 0) return;

    /* ---- 填满 FIFO 首批 ---- */
    /* 先填 ctrl */
    sent += DL_I2C_fillControllerTXFIFO(OLED_INST, &ctrl, 1);
    /* 再尽可能填 data */
    batch = (total - sent < 7) ? (total - sent) : 7;
    if (batch > 0) {
        sent += DL_I2C_fillControllerTXFIFO(OLED_INST, data, batch);
    }

    /* 启动传输（硬件在 total 字节后自动 STOP） */
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, total);

    /* ---- FIFO 排空后补充后续批次 ---- */
    /*   sent 包含了 ctrl，所以 data 索引 = sent - 1 */
    while (sent < total) {
        tout = OLED_I2C_LONG_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(OLED_INST)) {
            if (--tout == 0) { 
                OLED_I2C_Recover(); 
                return; 
            }
        }
        batch = (total - sent < 8) ? (total - sent) : 8;
        sent += DL_I2C_fillControllerTXFIFO(OLED_INST,
                                            &data[sent - 1], batch);
    }

    /* 等本次传输完成 */
    OLED_I2C_WaitIdle(OLED_I2C_LONG_TIMEOUT);
}

/* ============================================================================
 *  OLED_ColorTurn —— 反显开关
 * ========================================================================== */
void OLED_ColorTurn(u8 i)
{
    OLED_WR_Byte(i ? 0xA7 : 0xA6, OLED_CMD);
}

/* ============================================================================
 *  OLED_DisplayTurn —— 屏幕旋转 180°
 * ========================================================================== */
void OLED_DisplayTurn(u8 i)
{
    uint8_t cmds[2];
    if (i == 0) {
        cmds[0] = 0xC8; cmds[1] = 0xA1;
    } else {
        cmds[0] = 0xC0; cmds[1] = 0xA0;
    }
    OLED_WriteBlock(0x00, cmds, 2);
}

/* ============================================================================
 *  开关显示
 * ========================================================================== */
void OLED_DisPlay_On(void)
{
    uint8_t cmds[3] = { 0x8D, 0x14, 0xAF };
    OLED_WriteBlock(0x00, cmds, 3);
}

void OLED_DisPlay_Off(void)
{
    uint8_t cmds[3] = { 0x8D, 0x10, 0xAF };
    OLED_WriteBlock(0x00, cmds, 3);
}

/* ============================================================================
 *  OLED_Refresh —— 显存 → 屏幕
 *  每页 1 次 I2C 事务（原 128 次），全屏约 27ms @400kHz。
 * ========================================================================== */
void OLED_Refresh(void)
{
    u8 i;
    uint8_t page_cmds[3];

    for (i = 0; i < OLED_PAGES; i++) {
        page_cmds[0] = (uint8_t)(0xB0 + i);
        page_cmds[1] = 0x00;
        page_cmds[2] = 0x10;
        OLED_WriteBlock(0x00, page_cmds, 3);
        OLED_WriteBlock(0x40, OLED_GRAM[i], OLED_GRAM_COLS);
    }
}

/* ============================================================================
 *  OLED_Clear —— 清显存 + 刷新
 * ========================================================================== */
void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < OLED_PAGES; i++)
        for (n = 0; n < OLED_GRAM_COLS; n++)
            OLED_GRAM[i][n] = 0;
    OLED_Refresh();
}

/* ============================================================================
 *  OLED_ClearArea —— 清除矩形区域（像素坐标）
 * ========================================================================== */
void OLED_ClearArea(u8 x0, u8 y0, u8 x1, u8 y1)
{
    u8 i, j;
    u8 page, bit;
    if (x0 > 127) x0 = 127;
    if (x1 > 128) x1 = 128;
    if (y0 > 63)  y0 = 63;
    if (y1 > 64)  y1 = 64;
    for (i = x0; i < x1; i++) {
        for (j = y0; j < y1; j++) {
            page = j >> 3;
            bit  = j & 0x07;
            OLED_GRAM[page][i] &= ~(1 << bit);
        }
    }
}

/* ============================================================================
 *  画点 / 清点
 * ========================================================================== */
void OLED_DrawPoint(u8 x, u8 y)
{
    OLED_GRAM[y >> 3][x] |= (1 << (y & 0x07));
}

void OLED_ClearPoint(u8 x, u8 y)
{
    OLED_GRAM[y >> 3][x] &= ~(1 << (y & 0x07));
}

/* ============================================================================
 *  OLED_DrawLine —— 画线（需 x1≤x2, y1≤y2）
 * ========================================================================== */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2)
{
    int16_t dx, dy, steps, i;
    u8 k;

    if (x1 == x2) {
        for (k = y1; k <= y2; k++) OLED_DrawPoint(x1, k);
    } else if (y1 == y2) {
        for (k = x1; k <= x2; k++) OLED_DrawPoint(k, y1);
    } else {
        dx = (int16_t)(x2 - x1);
        dy = (int16_t)(y2 - y1);
        steps = (dx > dy) ? dx : dy;
        for (i = 0; i <= steps; i++) {
            OLED_DrawPoint((u8)(x1 + (dx * i) / steps),
                           (u8)(y1 + (dy * i) / steps));
        }
    }
}

/* ============================================================================
 *  OLED_DrawCircle —— 画圆（中点圆算法）
 * ========================================================================== */
void OLED_DrawCircle(u8 x, u8 y, u8 r)
{
    int16_t a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);
        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);
        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

/* ============================================================================
 *  OLED_ShowChar —— 显示单个 ASCII 字符
 * ========================================================================== */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1)
{
    u8 y0, size2, chr1, i, m, temp;
    y0 = y;
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++) {
        if      (size1 == 12) temp = asc2_1206[chr1][i];
        else if (size1 == 16) temp = asc2_1608[chr1][i];
        else if (size1 == 24) temp = asc2_2412[chr1][i];
        else return;

        for (m = 0; m < 8; m++) {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else             OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1) { y = y0; x++; break; }
        }
    }
}

/* ============================================================================
 *  OLED_ShowString —— 显示 ASCII 字符串（自动换行）
 * ========================================================================== */
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1)
{
    while (*chr >= ' ' && *chr <= '~') {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > OLED_WIDTH - size1) { x = 0; y += size1; }
        chr++;
    }
}

/* ============================================================================
 *  OLED_Pow —— 整数幂（内部辅助）
 * ========================================================================== */
static u32 OLED_Pow(u8 m, u8 n)
{
    u32 r = 1;
    while (n--) r *= m;
    return r;
}

/* ============================================================================
 *  OLED_ShowNum —— 显示无符号整数（固定位数，前导 '0'）
 * ========================================================================== */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1)
{
    u8 t, d;
    for (t = 0; t < len; t++) {
        d = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2) * t, y, d + '0', size1);
    }
}

/* ============================================================================
 *  OLED_ShowSignedNum —— 显示有符号整数
 * ========================================================================== */
void OLED_ShowSignedNum(u8 x, u8 y, int32_t num, u8 len, u8 size1)
{
    u8 char_w = size1 / 2;

    if (num >= 0)
    {
        OLED_ShowChar(x, y, '+', size1);
        OLED_ShowNum(x + char_w, y, (u32)num, len, size1);
    }
    else
    {
        OLED_ShowChar(x, y, '-', size1);
        OLED_ShowNum(x + char_w, y, (u32)(-num), len, size1);
    }
}

/* ============================================================================
 *  OLED_ShowFloatNum —— 显示浮点数
 *  intLen: 整数部分位数, decLen: 小数部分位数
 * ========================================================================== */
void OLED_ShowFloatNum(u8 x, u8 y, float num, u8 intLen, u8 decLen, u8 size1)
{
    u8 char_w = size1 / 2;
    int32_t intPart;
    uint32_t decPart;
    uint32_t multiplier = 1;
    u8 i;

    for (i = 0; i < decLen; i++)
        multiplier *= 10;

    if (num < 0)
    {
        OLED_ShowChar(x, y, '-', size1);
        x += char_w;
        num = -num;
    }
    else
    {
        OLED_ShowChar(x, y, ' ', size1);
        x += char_w;
    }

    intPart = (int32_t)num;
    decPart = (uint32_t)((num - (float)intPart) * multiplier + 0.5f);

    if (decPart >= multiplier)
    {
        intPart++;
        decPart -= multiplier;
    }

    OLED_ShowNum(x, y, (u32)intPart, intLen, size1);
    x += char_w * intLen;
    OLED_ShowChar(x, y, '.', size1);
    x += char_w;
    OLED_ShowNum(x, y, decPart, decLen, size1);
}

/* ============================================================================
 *  OLED_ShowChinese —— 显示汉字（基于字模索引）
 * ========================================================================== */
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1)
{
    u8 x0, y0, n, size3, chr1, i, m;
    u8 temp;

    x0 = x; y0 = y;
    n = 0;
    size3 = size1 / 8;
    while (size3--) {
        chr1 = num * size1 / 8 + n;
        n++;
        for (i = 0; i < size1; i++) {
            if      (size1 == 16) temp = Hzk1[chr1][i];
            else if (size1 == 24) temp = Hzk2[chr1][i];
            else if (size1 == 32) temp = Hzk3[chr1][i];
            else if (size1 == 64) temp = Hzk4[chr1][i];
            else return;

            for (m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y);
                else             OLED_ClearPoint(x, y);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == size1) { x = x0; y0 += 8; }
            y = y0;
        }
    }
}

/* ============================================================================
 *  OLED_WR_BP —— 设置写入起始页/列（用于 ShowPicture）
 * ========================================================================== */
void OLED_WR_BP(u8 x, u8 y)
{
    uint8_t cmds[3];
    cmds[0] = (uint8_t)(0xB0 + y);
    cmds[1] = (uint8_t)(((x & 0xF0) >> 4) | 0x10);
    cmds[2] = (uint8_t)((x & 0x0F) | 0x01);
    OLED_WriteBlock(0x00, cmds, 3);
}

/* ============================================================================
 *  OLED_ShowPicture —— 显示位图（y 以页为单位）
 * ========================================================================== */
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[])
{
    u32 j;
    u8 y;
    j = 0;
    for (y = y0; y < y1; y++) {
        OLED_WR_BP(x0, y);
        OLED_WriteBlock(0x40, &BMP[j], (uint16_t)(x1 - x0));
        j += (x1 - x0);
    }
}

/* ============================================================================
 *  OLED_Init —— SSD1306 初始化（带重试机制，防止 I2C 失败导致不亮）
 * ========================================================================== */
void OLED_Init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,          /* turn off */
        0x00, 0x10,    /* low/high column */
        0x40,          /* start line */
        0x81, 0xCF,    /* contrast */
        0xA1,          /* segment remap */
        0xC8,          /* COM scan direction */
        0xA6,          /* normal display */
        0xA8, 0x3F,    /* multiplex ratio */
        0xD3, 0x00,    /* display offset */
        0xD5, 0x80,    /* clock divide */
        0xD9, 0xF1,    /* pre-charge */
        0xDA, 0x12,    /* COM pins */
        0xDB, 0x40,    /* VCOMH */
        0x20, 0x02,    /* page addressing mode */
        0x8D, 0x14,    /* charge pump enable */
        0xA4, 0xA6,    /* disable entire on, disable inverse */
        0xAF,          /* turn on */
    };

    uint8_t retry;

    /* 4 针 I2C OLED 无 RST 引脚，等内部 RC 复位 */
    delay_ms(100);

    /* 重试 3 次初始化，防止 I2C 失败导致不亮 */
    for (retry = 0; retry < 3; retry++) {
        OLED_I2C_Recover();  /* 先恢复 I2C 总线 */
        delay_ms(10);
        OLED_WriteBlock(0x00, init_cmds, sizeof(init_cmds));
        delay_ms(10);
        OLED_Clear();
        delay_ms(10);
    }
}
