#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "PID.h"

/*============================================================================
 * PID 控制模块
 * 按照原始 STM32 代码逻辑移植
 *===========================================================================*/

/* PID 实例声明 */
extern PID_Incremental PID_MotorA;   /* 电机A 增量式 PID */
extern PID_Incremental PID_MotorB;   /* 电机B 增量式 PID */

/* 控制标志：启动延时后才启用 PID */
extern volatile uint8_t pid_enabled;

/* 函数声明 */

/**
 * @brief  PID 控制初始化（设置 PID 参数）
 */
void PID_Control_Init(void);

/**
 * @brief  PID 控制更新（在定时器中断里调用，20ms 周期）
 *
 * 原始逻辑：
 *   1. 启动延时（500 周期）后才启用 PID
 *   2. 读编码器 → 转换为速度
 *   3. 增量式 PID 计算
 *   4. 输出到电机
 *
 * @param  pulseA 电机A 编码器增量（Encoder_Get()）
 * @param  pulseB 电机B 编码器增量（Encoder_GPIO_Get()）
 */
void PID_Control_Update(int16_t pulseA, int16_t pulseB);

/**
 * @brief  设置电机目标速度
 * @param  targetA 电机A 目标速度
 * @param  targetB 电机B 目标速度
 */
void PID_Control_SetTarget(float targetA, float targetB);

#endif
