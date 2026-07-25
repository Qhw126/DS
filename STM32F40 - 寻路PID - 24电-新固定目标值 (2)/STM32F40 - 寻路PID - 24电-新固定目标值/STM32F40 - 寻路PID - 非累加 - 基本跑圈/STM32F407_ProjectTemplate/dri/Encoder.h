#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

// 四路编码器初始化
void Encoder_Init_TIM1(void);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);

// 读取编码器值（返回有符号数）
int16_t Encoder1_Get(void);
int16_t Encoder2_Get(void);
int16_t Encoder3_Get(void);
int16_t Encoder4_Get(void);

#endif
