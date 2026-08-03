#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

typedef struct
{
    int16_t x;
    uint16_t y;
    uint8_t valid;
    uint32_t sequence;
} VisionSample;

/* USART1: PA9/PA10连接K230；USART2: PA2/PA3连接X42S。 */
void Serial_Init(void);
void MotorSerial_SendArray(const uint8_t *data, uint16_t length);
void Vision_GetLatest(VisionSample *sample);

#endif
