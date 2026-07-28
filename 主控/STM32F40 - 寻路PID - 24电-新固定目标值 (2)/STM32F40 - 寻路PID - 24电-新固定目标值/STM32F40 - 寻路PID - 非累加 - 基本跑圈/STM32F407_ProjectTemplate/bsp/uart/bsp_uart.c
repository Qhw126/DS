/**
  * @file    bsp_uart.c
  * @brief   串口驱动模块 - 使用USART3(PD8/PD9)
  ******************************************************************************
  */
#include "bsp_uart.h"
#include "GyroUART.h"
#include <stdio.h>

/**
  * @brief  串口3初始化(用于陀螺仪通信)
  * @param  __Baud: 波特率
  * @retval 无
  * @note   USART3_TX: PD8, USART3_RX: PD9
  */
void uart3_init(uint32_t __Baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* GPIO复用配置 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    /* TX引脚配置 (PD8) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* RX引脚配置 (PD9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 串口参数配置 */
    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate            = __Baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);

    /* 使能接收中断 */
    USART_ClearFlag(USART3, USART_FLAG_RXNE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  串口发送字节数组
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval 无
  */
void UART_SendBytes(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        USART_SendData(USART3, data[i]);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}

/**
  * @brief  串口3中断服务函数
  * @param  无
  * @retval 无
  */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        uint8_t data = USART_ReceiveData(USART3);
        /* 调用陀螺仪数据解析函数 */
        GyroUART_ProcessByte(data);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

/* printf重定向 */
#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif
FILE __stdout;
void _sys_exit(int x)
{
    x = x;
}
#endif

int fputc(int ch, FILE *f)
{
    USART_SendData(USART3, (uint8_t)ch);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    return ch;
}
