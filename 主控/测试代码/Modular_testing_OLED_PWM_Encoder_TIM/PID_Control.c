#include "ti_msp_dl_config.h"
#include "PID_Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "Speed.h"

/*============================================================================
 * 按照原始 STM32 代码逻辑移植
 *
 * 原始 time.c 关键逻辑：
 *   1. 启动延时 i>=500 后才启用 PID（让电机稳定）
 *   2. TIM6 中断（20ms）：
 *      - 读编码器 → 转换为速度
 *      - PID_Motor3.Actual = -1 * Encoder3_Get() * Pulse_to_speed
 *      - Incremental_PID(&PID_Motor3)
 *      - Motor3_SetPWM(PID_Motor3.Out)
 *   3. 方向校正：一个编码器取反（-1）
 *===========================================================================*/

/* PID 实例定义 */
PID_Incremental PID_MotorA;
PID_Incremental PID_MotorB;

/* 启动延时计数器（原始代码用 i>=500） */
static volatile uint32_t startup_counter = 0;
volatile uint8_t pid_enabled = 0;

/* 启动延时阈值（原始代码是 500 次中断，约 500×20ms = 10秒） */
#define STARTUP_DELAY   500

/*============================================================================
 * PID 参数初始化
 * 按照原始 STM32 代码的参数
 *===========================================================================*/
void PID_Control_Init(void)
{
    /* 电机A PID 参数（输出单位：RPM）
     * 原始增益是PWM单位，除以前馈系数换算成RPM单位
     * Kp_rpm = Kp_pwm / FEEDFORWARD_GAIN = 1.32 / 1.543 ≈ 0.855
     */
    PID_MotorA.Kp = 1.32f / FEEDFORWARD_GAIN;
    PID_MotorA.Ki = 0.112f / FEEDFORWARD_GAIN;
    PID_MotorA.Kd = 0.0f;
    PID_MotorA.Target_Iint = 0.0f;     /* 初始目标速度 */
    PID_MotorA.Target = PID_MotorA.Target_Iint;
    PID_MotorA.OutMax = 200.0f;        /* 输出上限（RPM） */
    PID_MotorA.OutMin = -200.0f;       /* 输出下限（RPM） */
    PID_MotorA.Actual = 0.0f;
    PID_MotorA.Out = 0.0f;

    /* 电机B PID 参数（输出单位：RPM） */
    PID_MotorB.Kp = 1.313f / FEEDFORWARD_GAIN;
    PID_MotorB.Ki = 0.11f / FEEDFORWARD_GAIN;
    PID_MotorB.Kd = 0.001f / FEEDFORWARD_GAIN;
    PID_MotorB.Target_Iint = 0.0f;
    PID_MotorB.Target = PID_MotorB.Target_Iint;
    PID_MotorB.OutMax = 200.0f;        /* 输出上限（RPM） */
    PID_MotorB.OutMin = -200.0f;       /* 输出下限（RPM） */
    PID_MotorB.Actual = 0.0f;
    PID_MotorB.Out = 0.0f;

    /* 重置启动计数器 */
    startup_counter = 0;
    pid_enabled = 0;
}

/*============================================================================
 * PID 控制更新（在定时器中断里调用，20ms 周期）
 *
 * 完全按照原始 TIM6_DAC_IRQHandler 逻辑：
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
 *===========================================================================*/
void PID_Control_Update(void)
{
    pid_enabled = 1;

    /* 读编码器增量 */
    int16_t pulseA = Encoder_Get();
    int16_t pulseB = Encoder_GPIO_Get();

    /* 电机A：编码器脉冲 → RPM → PID修正(RPM) → 目标+修正 → PWM */
    /* 编码器A硬件QEI 4倍频，方向取反 */
    PID_MotorA.Actual = -1.0f * (float)pulseA * PULSE_TO_RPM_A;
    Incremental_PID(&PID_MotorA);
    /* 总目标RPM = 目标 + PID修正，再转PWM */
    float totalRpmA = PID_MotorA.Target + PID_MotorA.Out;
    float outA = totalRpmA * FEEDFORWARD_GAIN;
    if (outA > MOTOR_RATED_PWM) outA = MOTOR_RATED_PWM;
    if (outA < -MOTOR_RATED_PWM) outA = -MOTOR_RATED_PWM;
    Motor_SetSpeed(MOTOR_A, (int)outA);

    /* 电机B：编码器脉冲 → RPM → PID修正(RPM) → 目标+修正 → PWM */
    /* 编码器B GPIO中断 2倍频 */
    PID_MotorB.Actual = 1.0f * (float)pulseB * PULSE_TO_RPM_B;
    Incremental_PID(&PID_MotorB);
    /* 总目标RPM = 目标 + PID修正，再转PWM */
    float totalRpmB = PID_MotorB.Target + PID_MotorB.Out;
    float outB = totalRpmB * FEEDFORWARD_GAIN;
    if (outB > MOTOR_RATED_PWM) outB = MOTOR_RATED_PWM;
    if (outB < -MOTOR_RATED_PWM) outB = -MOTOR_RATED_PWM;
    Motor_SetSpeed(MOTOR_B, (int)outB);
}

/*============================================================================
 * 设置目标速度
 *===========================================================================*/
void PID_Control_SetTarget(float targetA, float targetB)
{
    PID_MotorA.Target = targetA;
    PID_MotorB.Target = targetB;
}
