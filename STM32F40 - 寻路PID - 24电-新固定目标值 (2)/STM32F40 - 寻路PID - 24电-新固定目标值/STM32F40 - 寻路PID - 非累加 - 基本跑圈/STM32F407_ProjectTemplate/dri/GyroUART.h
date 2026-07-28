/**
  ******************************************************************************
  * @file    GyroUART.h
  * @brief   串口陀螺仪驱动模块
  * @note    适用于六轴陀螺仪模块（串口通信版本）
  *          协议: 0x5A + TYPE + DATA(8字节) + SUM(共11字节)
  ******************************************************************************
  */
#ifndef __GYRO_UART_H
#define __GYRO_UART_H

#include "stm32f4xx.h"

/* 数据结构体定义 */
typedef struct {
    float wx;               // X轴角速度 (°/s)
    float wy;               // Y轴角速度 (°/s)
    float wz;               // Z轴角速度 (°/s)
} GyroData_t;

typedef struct {
    float Roll;             // 横滚角 (°)
    float Pitch;            // 俯仰角 (°)
    float Yaw;              // 偏航角 (°)
} AngleData_t;

typedef struct {
    float ax;               // X轴加速度 (m/s²)
    float ay;               // Y轴加速度 (m/s²)
    float az;               // Z轴加速度 (m/s²)
} AccelData_t;

typedef struct {
    float q0;               // 四元数 q0
    float q1;               // 四元数 q1
    float q2;               // 四元数 q2
    float q3;               // 四元数 q3
} QuatData_t;

/* 外部变量声明 */
extern GyroData_t stcGyro;
extern AngleData_t stcAngle;
extern AccelData_t stcAccel;
extern QuatData_t stcQuat;

/* 函数声明 */
void GyroUART_Init(void);
void GyroUART_ProcessByte(uint8_t ucData);

/* 数据获取接口 */
float GyroUART_GetYaw(void);
float GyroUART_GetRoll(void);
float GyroUART_GetPitch(void);
float GyroUART_GetGyroX(void);
float GyroUART_GetGyroY(void);
float GyroUART_GetGyroZ(void);
float GyroUART_GetAccelX(void);
float GyroUART_GetAccelY(void);
float GyroUART_GetAccelZ(void);

/* 命令发送接口 */
void GyroUART_SendCaliYaw(void);
void GyroUART_SendCaliBias(void);
void GyroUART_SendUnlock(void);
void GyroUART_SendSave(void);

#endif /* __GYRO_UART_H */
