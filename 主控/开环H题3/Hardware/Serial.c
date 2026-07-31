#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

uint8_t Serial_TxPacket[4];           // 定义发送数据包数组
uint8_t Serial_RxPacket[8];           // 定义接收数据包数组，K230 发来 8 字节总包（AA 55 + 6 数据）
uint8_t Serial_RxFlag = 0;            // 定义接收数据包标志位

uint8_t Serial_Valid = 0;             // 解析出的有效标志
int16_t Serial_BallX = 0;             // 解析出的带符号 X 数据

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
    /* 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); // USART1 + GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);                       // USART2

    /* GPIO 初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 TX PA9 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 RX PA10 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 TX PA2 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 RX PA3 - 如果不使用接收也建议初始化 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 初始化 */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    /* USART2 初始化，仅发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* USART1 接收中断配置 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* NVIC 配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能 USART */
    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);

    Serial_RxFlag = 0;
    Serial_Valid = 0;
    Serial_BallX = 0;
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);              // 通过 USART2 发送电机命令
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/**
  * 函    数：自己封装的printf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/**
  * 函    数：串口发送数据包
  * 参    数：无
  * 返 回 值：无
  */
void Serial_SendPacket(void)
{
    Serial_SendByte(0xFF);
    Serial_SendArray(Serial_TxPacket, 4);
    Serial_SendByte(0xFE);
}

/**
  * 函    数：获取串口接收数据包标志位
  * 参    数：无
  * 返 回 值：串口接收数据包标志位，范围：0~1
  */
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

/**
  * 函    数：获取当前有效标志
  * 参    数：无
  * 返 回 值：有效标志
  */
uint8_t Serial_GetValid(void)
{
    return Serial_Valid;
}

/**
  * 函    数：获取当前 X 坐标
  * 参    数：无
  * 返 回 值：X 坐标
  */
int16_t Serial_GetBallX(void)
{
    return Serial_BallX;
}

/**
  * 函    数：获取接收数据包缓冲区指针
  * 参    数：无
  * 返 回 值：接收数据包缓冲区首地址
  */
uint8_t* Serial_GetRxPacket(void)
{
    return Serial_RxPacket;
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 说明：这里接收 K230 发送过来的 8 字节总包（AA 55 + 6 数据字节），并解析有效标志和带符号 X
  */
void USART1_IRQHandler(void)
{
    static uint8_t RxState = 0;      // 0: 等待 0xAA, 1: 等待 0x55, 2: 接收数据
    static uint8_t pRxPacket = 0;    // 当前接收到了第几个字节

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

        if (RxState == 0)            // 等待包头 0xAA
        {
            if (RxData == 0xAA)
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)       // 等待包头 0x55
        {
            if (RxData == 0x55)
            {
                RxState = 2;
                pRxPacket = 0;
            }
            else
            {
                RxState = 0;
            }
        }
        else if (RxState == 2)       // 接收 6 字节数据（K230 包共 8 字节：AA 55 + 6 数据）
        {
            Serial_RxPacket[pRxPacket] = RxData;
            pRxPacket++;

            if (pRxPacket >= 6)      // 收满 6 字节
            {
                RxState = 0;

                Serial_Valid = (Serial_RxPacket[0] != 0U);

                uint16_t temp_x = (uint16_t)Serial_RxPacket[1] |
                                  ((uint16_t)Serial_RxPacket[2] << 8);
                Serial_BallX = (int16_t)temp_x;

                Serial_RxFlag = 1;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
