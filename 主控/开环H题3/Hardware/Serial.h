#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_Printf(char *format, ...);

/* 接收数据接口 */
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetValid(void);
int16_t Serial_GetBallX(void);
uint8_t* Serial_GetRxPacket(void);

#endif
