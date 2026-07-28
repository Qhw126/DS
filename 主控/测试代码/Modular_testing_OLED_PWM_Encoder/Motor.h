#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

/* PWM 周期值（占空比范围 0~MOTOR_PWM_PERIOD） */
#define MOTOR_PWM_PERIOD    1000

/* 电机编号 */
typedef enum {
    MOTOR_A = 0,    /* CC0 (PA8) */
    MOTOR_B = 1,    /* CC1 (PB9) */
} Motor_ID;

/* 函数声明 */
void Motor_Init(void);
void Motor_SetSpeed(Motor_ID id, int16_t speed);
void Motor_Stop(Motor_ID id);
void Motor_StopAll(void);
void Motor_SetForward(Motor_ID id);
void Motor_SetReverse(Motor_ID id);
void Motor_Brake(Motor_ID id);

#endif
