#ifndef __GYRO_H
#define __GYRO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*============================================================================
 * 数据结构定义 - 六轴传感器数据
 *===========================================================================*/

/**
 * @brief 角度结构体 (单位: 度)
 */
typedef struct
{
    float Roll;   // 横滚角  (-180 ~ +180)
    float Pitch;  // 俯仰角  (-180 ~ +180)
    float Yaw;    // 航向角  (-180 ~ +180)
} Gyro_Angle;

/**
 * @brief 角速度结构体 (单位: °/s)
 */
typedef struct
{
    float wx;     // X轴角速度  (±2000°/s)
    float wy;     // Y轴角速度  (±2000°/s)
    float wz;     // Z轴角速度  (±2000°/s)
} Gyro_Rate;

/**
 * @brief 加速度结构体 (单位: m/s²)
 */
typedef struct
{
    float ax;     // X轴加速度  (±16g)
    float ay;     // Y轴加速度  (±16g)
    float az;     // Z轴加速度  (±16g)
} Gyro_Accel;

/*============================================================================
 * 接口函数声明
 *===========================================================================*/

/* 初始化 */
void Gyro_Init(void);

/* 数据解析（在中断中调用） */
void Gyro_ParseByte(uint8_t data);

/* 角度获取 */
float Gyro_GetYaw(void);
float Gyro_GetRoll(void);
float Gyro_GetPitch(void);

/* 角速度获取 */
float Gyro_GetRateX(void);
float Gyro_GetRateY(void);
float Gyro_GetRateZ(void);

/* 加速度获取 */
float Gyro_GetAccelX(void);
float Gyro_GetAccelY(void);
float Gyro_GetAccelZ(void);

/* 校准命令 */
void Gyro_CaliYaw(void);      // Z轴归零
void Gyro_CaliBias(void);     // 零偏校准（需静止6秒）

#endif /* __GYRO_H */
