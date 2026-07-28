#ifndef __LINE_H
#define __LINE_H

#include "PID.h"
#include <stdint.h>

/* 传感器数量 */
#define ADC_N 7

/* 控制模式（对齐参考工程） */
#define MODE_LINE_TRACK     0   /* 循线模式 */
#define MODE_GYRO_TURN      1   /* 陀螺仪旋转模式 */

/* 基础速度（RPM） */
extern float base_speed;

/* 控制模式相关变量 */
extern uint8_t  controlMode;
extern uint8_t  controlModelast;
extern float    yawWhenLost;
extern float    targetYaw;
extern uint8_t  isLineLost;
extern uint8_t  gyroToLineFlag;
extern uint32_t gyroToLineTick;

/* 函数声明 */
void track_zhixian1(void);
void BaseSpeed_Set(float speed);
uint8_t checkLineFound(void);

#endif
