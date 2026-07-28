#include "Motor.h"

/* CC 通道索引表 */
static const uint8_t motor_cc_idx[] = {
    GPIO_PWM_0_C0_IDX,  /* MOTOR_A → CC0 */
    GPIO_PWM_0_C1_IDX,  /* MOTOR_B → CC1 */
};

/*
 * 设置电机A方向：正转
 * AIN1=1, AIN2=0
 */
void Motor_SetForward(Motor_ID id)
{
    if (id == MOTOR_A)
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);     /* AIN1 = 1 */
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);   /* AIN2 = 0 */
    }
    else if (id == MOTOR_B)
    {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);     /* BIN1 = 1 */
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);   /* BIN2 = 0 */
    }
}

/*
 * 设置电机方向：反转
 * AIN1=0, AIN2=1
 */
void Motor_SetReverse(Motor_ID id)
{
    if (id == MOTOR_A)
    {
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);   /* AIN1 = 0 */
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);     /* AIN2 = 1 */
    }
    else if (id == MOTOR_B)
    {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);   /* BIN1 = 0 */
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);     /* BIN2 = 1 */
    }
}

/*
 * 电机刹车
 * AIN1=0, AIN2=0
 */
void Motor_Brake(Motor_ID id)
{
    if (id == MOTOR_A)
    {
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);   /* AIN1 = 0 */
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);   /* AIN2 = 0 */
    }
    else if (id == MOTOR_B)
    {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);   /* BIN1 = 0 */
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);   /* BIN2 = 0 */
    }
}

/*
 * 电机初始化
 */
void Motor_Init(void)
{
    Motor_StopAll();
}

/*
 * 设置电机速度
 * id:    MOTOR_A 或 MOTOR_B
 * speed: -1000 ~ +1000
 *        正值 → 正转
 *        负值 → 反转
 *        0    → 停止
 *
 * 保护逻辑：切换方向前先停止 PWM，防止电流冲击
 */
void Motor_SetSpeed(Motor_ID id, int16_t speed)
{
    if (id > MOTOR_B) return;

    /* 限幅 */
    if (speed > MOTOR_PWM_PERIOD)  speed = MOTOR_PWM_PERIOD;
    if (speed < -MOTOR_PWM_PERIOD) speed = -MOTOR_PWM_PERIOD;

    /* 先停止 PWM（保护电机） */
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, motor_cc_idx[id]);

    /* 设置方向 */
    if (speed > 0)
        Motor_SetForward(id);
    else if (speed < 0)
        Motor_SetReverse(id);
    else
        Motor_Brake(id);

    /* 取绝对值作为占空比 */
    uint16_t duty = (speed < 0) ? -speed : speed;

    /* 设置 PWM 占空比 */
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, duty, motor_cc_idx[id]);
}

/*
 * 停止单个电机（PWM=0，方向保持）
 */
void Motor_Stop(Motor_ID id)
{
    if (id > MOTOR_B) return;
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, motor_cc_idx[id]);
}

/*
 * 停止所有电机
 */
void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}
