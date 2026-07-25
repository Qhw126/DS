/**
  * @file    line.h
  * @brief   循线模块头文件
  ******************************************************************************
  */
#ifndef __LINE_H
#define __LINE_H

#include "stm32f4xx.h"

/* 控制模式 */
#define MODE_LINE_TRACK     0   // 循线模式
#define MODE_GYRO_TURN      1   // 陀螺仪旋转模式

/* 外部变量声明 (只声明，不定义) */
extern uint8_t controlMode;
extern uint8_t controlModelast;
extern float yawWhenLost;
extern float targetYaw;
extern uint8_t isLineLost;
extern uint8_t gyroToLineFlag;
extern uint32_t gyroToLineTick;

/* 函数声明 */
uint8_t checkLineLost(void);
void track_zhixian1(void);
uint8_t checkLineFound(void);

#endif /* __LINE_H */
