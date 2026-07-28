/**
  * @file    time.c
  * @brief   集中式定时器中断（对齐参考工程 dri/time.c）
  * @note    PID_TIM   (TIMG0, 20ms) — 电机PID闭环
  *          Pathfinding_TIM (TIMA1, 40ms) — 寻线/陀螺仪控制
  ******************************************************************************
  */
#include "ti_msp_dl_config.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "line.h"
#include "GyroControl.h"
#include "Speed.h"

/*============================================================================
 * 常量：使用 Speed.h 中定义的 PULSE_TO_RPM_A / PULSE_TO_RPM_B
 *       PULSE_TO_RPM_A = 60/(265.2*4*0.02) ≈ 2.828
 *       PULSE_TO_RPM_B = 60/(265.2*2*0.02) ≈ 5.656
 *===========================================================================*/

/*============================================================================
 * 全局变量
 *===========================================================================*/

/* 启动延时计数器（对齐参考工程 int i = 0） */
static int i = 0;

/* PID 使能标志（1秒后置1） */
volatile uint8_t pid_enabled = 0;

/* gyroToLine 保护计数器（对齐参考工程 timer6Count） */
uint32_t timer6Count = 0;

/*============================================================================
 * 延时函数（从 empty.c 移入）
 *===========================================================================*/
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

/*============================================================================
 * PID_TIM_INST_IRQHandler — 电机PID控制（20ms 周期）
 *
 * 对齐参考工程 TIM6_DAC_IRQHandler：
 *   if(i>=50)
 *   {
 *       gyroToLineCount 保护逻辑
 *       电机A：Encoder3_Get → PID → Motor_SetPWM
 *       电机B：Encoder4_Get → PID → Motor_SetPWM
 *   }
 *   else i++
 *===========================================================================*/
void PID_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PID_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:
        {
            if (i >= 50)
            {
                pid_enabled = 1;

                /* gyroToLine 保护计数器（对齐参考工程 timer6Count） */
                if (gyroToLineFlag == 1)
                {
                    timer6Count++;
                    if (timer6Count >= 50)
                    {
                        timer6Count = 0;
                        gyroToLineFlag = 0;
                    }
                }

                /* 电机A PID 闭环 */
                PID_MotorA.Actual = 1.0f * (float)Encoder3_Get() * PULSE_TO_RPM_A;
                Incremental_PID(&PID_MotorA);
                Motor_SetSpeed(MOTOR_A, (int)(PID_MotorA.Out * speed_to_dutycycle));

                /* 电机B PID 闭环 */
                PID_MotorB.Actual = -1.0f * (float)Encoder4_Get() * PULSE_TO_RPM_B;
                Incremental_PID(&PID_MotorB);
                Motor_SetSpeed(MOTOR_B, (int)(PID_MotorB.Out * speed_to_dutycycle));
            }
            else
            {
                i++;
            }
            break;
        }
        default:
            break;
    }
}

/*============================================================================
 * Pathfinding_TIM_INST_IRQHandler — 寻线/陀螺仪控制（40ms 周期）
 *
 * 对齐参考工程 TIM7_IRQHandler：
 *   if(i>=50)
 *   {
 *       MODE_LINE_TRACK → track_zhixian1 + Position_PID + motor()
 *       MODE_GYRO_TURN  → GyroControl_Update + motor() + checkLineFound
 *   }
 *===========================================================================*/
void Pathfinding_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Pathfinding_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:
        {
            if (i >= 50)
            {
                if (controlMode == MODE_LINE_TRACK)
                {
                    /* 循线模式 */
                    track_zhixian1();
                    Position_PID(&PID_findway);
                    motor((int)(base_speed - PID_findway.Out),
                          (int)(base_speed + PID_findway.Out));
                }
                else if (controlMode == MODE_GYRO_TURN)
                {
                    /* 陀螺仪模式 */
                    GyroControl_SetTarget(base_speed, targetYaw);
                    GyroControl_Update();
                    motor((int)(GyroControl.BaseSpeed + GyroControl.TurnPWM),
                          (int)(GyroControl.BaseSpeed - GyroControl.TurnPWM));

                    /* 检查是否又检测到线，切回循线 */
                    if (checkLineFound())
                    {
                        controlMode = MODE_LINE_TRACK;
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}
