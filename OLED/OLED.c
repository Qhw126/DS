/*
 * OLED SSD1306 驱动 - 硬件 I2C 版本
 *
 * SysConfig 配置步骤：
 *   1. 添加 I2C 模块
 *   2. peripheral → I2C0
 *   3. sdaPin → PA10
 *   4. sclPin → PA11
 *   5. basicEnableController → true
 *   6. 删除 GPIO 模块中 PB8/PB9 的配置（如果有）
 *
 * 生成的宏：OLED_INST
 */

#include "ti_msp_dl_config.h"
#include "OLED_Font.h"

/* SSD1306 I2C 地址（7位地址，0x3C） */
#define OLED_I2C_ADDR   0x3C

/* SSD1306 控制字节 */
#define OLED_CMD_CTRL   0x00    /* 后续数据为命令 */
#define OLED_DATA_CTRL  0x40    /* 后续数据为显示数据 */

/**
  * @brief  等待 I2C 总线空闲
  */
static void OLED_I2C_WaitIdle(void)
{
    while (DL_I2C_getControllerStatus(OLED_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;
}

/**
  * @brief  通过硬件 I2C 发送一个字节（命令或数据）
  * @param  ctrl   控制字节 (OLED_CMD_CTRL 或 OLED_DATA_CTRL)
  * @param  data   要发送的数据
  */
static void OLED_I2C_Write(uint8_t ctrl, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = ctrl;
    buf[1] = data;

    /* 等待总线空闲 */
    OLED_I2C_WaitIdle();

    /* 填充 TX FIFO */
    DL_I2C_fillControllerTXFIFO(OLED_INST, buf, 2);

    /* 启动传输：地址 + TX + 2字节 + Start + Stop */
    DL_I2C_startControllerTransfer(
        OLED_INST, OLED_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* 等待传输完成 */
    while (DL_I2C_getControllerStatus(OLED_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;
}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Write(OLED_CMD_CTRL, Command);
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Write(OLED_DATA_CTRL, Data);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    //设置Y位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    //设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));            //设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
  * @brief  OLED显示数字（二进制，正数）
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  * @note   I2C 外设由 SysConfig 自动生成的 SYSCFG_DL_init() 完成初始化
  */
void OLED_Init(void)
{
    uint32_t i, j;

    for (i = 0; i < 1000; i++)         //上电延时
    {
        for (j = 0; j < 1000; j++);
    }

    OLED_WriteCommand(0xAE);    //关闭显示

    OLED_WriteCommand(0xD5);    //设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    //设置多路复用率
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    //设置显示偏移
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);    //设置显示开始行

    OLED_WriteCommand(0xA1);    //设置左右方向，0xA1正常 0xA0左右反置

    OLED_WriteCommand(0xC8);    //设置上下方向，0xC8正常 0xC0上下反置

    OLED_WriteCommand(0xDA);    //设置COM引脚硬件配置
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);    //设置对比度控制
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);    //设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);    //设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);    //设置整个显示打开/关闭

    OLED_WriteCommand(0xA6);    //设置正常/倒转显示

    OLED_WriteCommand(0x8D);    //设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);    //开启显示

    OLED_Clear();                //OLED清屏
}
