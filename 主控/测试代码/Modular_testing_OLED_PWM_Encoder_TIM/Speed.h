#ifndef __SPEED_H
#define __SPEED_H

#include <stdint.h>

/*============================================================================
 * 速度计算参数宏定义
 *===========================================================================*/

/* 编码器参数 */
#define ENCODER_PPR             265.2f  /* 编码器每转脉冲数（已含减速比） */

/* 采样参数 */
#define SPEED_SAMPLE_TIME       0.02f   /* 采样周期（秒），20ms */

/* 速度转换系数（脉冲 → RPM）
 * 编码器A：硬件QEI，4倍频 → 每转计数 = PPR × 4
 * 编码器B：GPIO中断，2倍频 → 每转计数 = PPR × 2
 * 公式：RPM = 脉冲数 × 60 / (PPR × 倍频 × 采样时间)
 */
#define PULSE_TO_RPM_A  (60.0f / (ENCODER_PPR * 4 * SPEED_SAMPLE_TIME))  /* 硬件编码器，4倍频 */
#define PULSE_TO_RPM_B  (60.0f / (ENCODER_PPR * 2 * SPEED_SAMPLE_TIME))  /* GPIO编码器，2倍频 */

/* 默认使用编码器B的系数（兼容未使用的 Speed_Update 函数） */
#define PULSE_TO_RPM    PULSE_TO_RPM_B

/*============================================================================
 * 前馈控制参数
 * 电机参数：电源12V，额定7.4V，额定转速400RPM
 * 额定PWM = 7.4 / 12 × 1000 ≈ 617
 * 前馈增益 = 617 / 400 = 1.543 PWM/RPM
 *===========================================================================*/
#define MOTOR_SUPPLY_VOLTAGE    12.0f
#define MOTOR_RATED_VOLTAGE     7.4f
#define MOTOR_RATED_RPM         400.0f
#define MOTOR_RATED_PWM         (MOTOR_RATED_VOLTAGE / MOTOR_SUPPLY_VOLTAGE * 1000.0f)
#define FEEDFORWARD_GAIN        (MOTOR_RATED_PWM / MOTOR_RATED_RPM)  /* ≈1.543 */

/* 速度结构体 */
typedef struct {
    int16_t pulse_count;    /* 原始脉冲计数 */
    float   rpm;            /* 转速（RPM） */
} Speed_Data;

/*============================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief  更新速度数据
 * @param  data 速度数据结构体指针
 * @param  encoder_count 编码器增量值
 * @param  dir 方向系数（1 或 -1）
 */
void Speed_Update(Speed_Data *data, int16_t encoder_count, int8_t dir);

#endif
