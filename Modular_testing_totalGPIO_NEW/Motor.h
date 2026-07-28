#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

/*============================================================================
 * 电机驱动模块
 *
 * 硬件接线：
 *   MOTOR_A: PWMA → TIMA0-CC2 (PB4), AIN1 → PA24, AIN2 → PA25
 *   MOTOR_B: PWMB → TIMA0-CC0 (PA8), BIN1 → PB20, BIN2 → PB25
 *   MOTOR_C: (占位，未接)
 *   MOTOR_D: (占位，未接)
 *
 * PWM 来源：TIMA0，4路 CC 输出（CC0~CC3），共用 PWM_0_INST
 * 方向控制：每个电机 2 个 GPIO（IN1/IN2），通过 H 桥驱动
 *
 * Motor_ID 的 enum 值 = CC 通道号，可直接作为 motor_cc_idx[] 数组索引
 *===========================================================================*/

/* PWM 周期值（占空比范围 0~MOTOR_PWM_PERIOD） */
#define MOTOR_PWM_PERIOD    1000

/* 电机编号（值 = CC 通道索引，直接用于索引 motor_cc_idx[]） */
typedef enum {
    MOTOR_B = 0,    /* CC0 (PA8)  — PWMB */
    MOTOR_C = 1,    /* CC1 (PB12) — 占位，未接 */
    MOTOR_A = 2,    /* CC2 (PB4)  — PWMA */
    MOTOR_D = 3,    /* CC3 (PB26) — 占位，未接 */
} Motor_ID;

/* 最大有效电机编号，用于边界检查 */
#define MOTOR_MAX   MOTOR_D

/* 函数声明 */
void Motor_Init(void);
void Motor_SetSpeed(Motor_ID id, int16_t speed);
void Motor_Stop(Motor_ID id);
void Motor_StopAll(void);
void Motor_SetForward(Motor_ID id);
void Motor_SetReverse(Motor_ID id);
void Motor_Brake(Motor_ID id);

#endif
