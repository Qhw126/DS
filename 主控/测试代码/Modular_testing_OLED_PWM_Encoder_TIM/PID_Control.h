#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "PID.h"

/*============================================================================
 * PID 控制模块
 * 按照原始 STM32 代码逻辑移植
 * 使用定时器中断（20ms 周期）调用 PID_Control_Update
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
 * 原始逻辑（TIM6_DAC_IRQHandler）：
 *   if(i>=500)
 *   {
 *       PID_Motor3.Actual = -1 * Encoder3_Get() * Pulse_to_speed;
 *       Incremental_PID(&PID_Motor3);
 *       Motor3_SetPWM(PID_Motor3.Out);
 *
 *       PID_Motor4.Actual = 1 * Encoder4_Get() * Pulse_to_speed;
 *       Incremental_PID(&PID_Motor4);
 *       Motor4_SetPWM(PID_Motor4.Out);
 *   }
 *   else
 *   {
 *       i++;
 *   }
 */
void PID_Control_Update(void);

/**
 * @brief  设置电机目标速度
 * @param  targetA 电机A 目标速度
 * @param  targetB 电机B 目标速度
 */
void PID_Control_SetTarget(float targetA, float targetB);

#endif
