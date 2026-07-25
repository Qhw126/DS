/**
  * @file    bsp_uart.h
  * @brief   串口驱动模块 - 使用USART3(PD8/PD9)
  ******************************************************************************
  */
#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "stm32f4xx.h"

/* 函数声明 */
void uart3_init(uint32_t __Baud);
void UART_SendBytes(uint8_t *data, uint32_t len);
void USART3_IRQHandler(void);

#endif /* __BSP_UART_H__ */
