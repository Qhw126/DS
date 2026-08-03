/**
  ******************************************************************************
  * @file    GyroControl.h
  * @brief   陀螺仪PID控制模块
  * @note    用于基于陀螺仪的转向控制，使用串口陀螺仪数据
  ******************************************************************************
  */
#ifndef __GYRO_CONTROL_H
#define __GYRO_CONTROL_H

#include "stm32f4xx.h"

/* 角度环PID参数结构体 */
typedef struct
{
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float Error;            // 当前误差
    float LastError;        // 上次误差
    float Integral;         // 积分累加
    float IntegralMax;      // 积分限幅
    float Output;           // 输出值
    float OutputMax;        // 输出限幅
} GyroPID_Angle_t;

/* 角速度环PID参数结构体 */
typedef struct
{
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float Error;            // 当前误差
    float LastError;        // 上次误差
    float Integral;         // 积分累加
    float IntegralMax;      // 积分限幅
    float Output;           // 输出值
    float OutputMax;        // 输出限幅
} GyroPID_Gyro_t;

/* 控制参数结构体 */
typedef struct
{
    float BaseSpeed;        // 基础速度
    float TargetYaw;        // 目标偏航角(度)
    float CurrentYaw;       // 当前偏航角(度)
    float CurrentGyro;      // 当前角速度(°/s)
    float TurnPWM;          // 转向PWM输出
    float PWM_Left;         // 左电机PWM
    float PWM_Right;        // 右电机PWM
    float PWM_Max;          // PWM最大值
    float PWM_Min;          // PWM最小值
    float DeadZone;         // 死区补偿
} GyroControl_t;

/* 陀螺仪方向定义 */
#define GYRO_DIR_NORMAL     0   // 正常方向（顺时针角度增加）
#define GYRO_DIR_REVERSE    1   // 反向（逆时针角度增加）

/* 外部变量声明 */
extern uint8_t gyroDir;             // 陀螺仪方向选择
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
