/**
  ******************************************************************************
  * @file    GyroControl.h
  * @brief   陀螺仪PID控制模块
  * @note    用于基于陀螺仪的转向控制，使用串口陀螺仪数据
  *          移植自 STM32 参考工程 dri/GyroControl.h
  ******************************************************************************
  */
#ifndef __GYRO_CONTROL_H
#define __GYRO_CONTROL_H

#include <stdint.h>

/* 角度环PID参数结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float LastError;
    float Integral;
    float IntegralMax;
    float Output;
    float OutputMax;
} GyroPID_Angle_t;

/* 角速度环PID参数结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float LastError;
    float Integral;
    float IntegralMax;
    float Output;
    float OutputMax;
} GyroPID_Gyro_t;

/* 控制参数结构体 */
typedef struct
{
    float BaseSpeed;
    float TargetYaw;
    float CurrentYaw;
    float CurrentGyro;
    float TurnPWM;
    float PWM_Left;
    float PWM_Right;
    float PWM_Max;
    float PWM_Min;
    float DeadZone;
} GyroControl_t;

/* 陀螺仪方向定义 */
#define GYRO_DIR_NORMAL     0
#define GYRO_DIR_REVERSE    1

/* 外部变量声明 */
extern uint8_t gyroDir;
extern GyroPID_Angle_t GyroPID_Angle;
extern GyroPID_Gyro_t GyroPID_Gyro;
extern GyroControl_t GyroControl;

/* 函数声明 */
void GyroControl_Init(void);
void GyroControl_SetPID_Angle(float Kp, float Ki, float Kd);
void GyroControl_SetPID_Gyro(float Kp, float Ki, float Kd);
void GyroControl_SetTarget(float baseSpeed, float targetYaw);
float GyroControl_NormalizeAngle(float angle);
float GyroControl_Limit(float val, float min, float max);
void GyroControl_Update(void);
void GyroControl_Reset(void);

#endif /* __GYRO_CONTROL_H */
