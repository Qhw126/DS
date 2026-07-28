#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "OLED.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "Speed.h"
#include "PID.h"
#include "PID_Control.h"
#include "sensor.h"
#include "line.h"

/* 简单延时 */
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

/* 两个编码器各自的速度数据（用于显示） */
Speed_Data speedA = {0, 0.0f};
Speed_Data speedB = {0, 0.0f};

/* 传感器显示模式：0=显示编码器, 1=显示传感器 */
static uint8_t sensor_show_mode = 0;

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    Motor_Init();
    PID_Control_Init();
    Line_PID_Init();

    /* EncoderA = 硬件编码器 (TIMG8 QEI, PA26/PA27, 4倍频) */
    Encoder_Init();

    /* EncoderB = GPIO 中断编码器 (PA28/PA31, 2倍频) */
    Encoder_GPIO_Init();

    /* 使能 PID 定时器中断（20ms）— 电机速度闭环 */
    NVIC_EnableIRQ(PID_TIM_INST_INT_IRQN);

    /* 使能寻路定时器中断（40ms）— 寻路差速转向 */
    NVIC_EnableIRQ(Pathfinding_TIM_INST_INT_IRQN);

    OLED_ShowString(1, 1, "Target:");
    OLED_ShowString(2, 1, "A RPM:");
    OLED_ShowString(3, 1, "B RPM:");
    OLED_ShowString(4, 1, "PID Fix:");

    while (1)
    {
        /* 读取当前传感器状态 */
        uint8_t sensor_now = (L3 << 6) | (L2 << 5) | (L1 << 4) | (MC << 3)
                           | (R1 << 2) | (R2 << 1) | R3;

        /* 有传感器触发（非全白）→ 持续显示传感器 */
        if(sensor_now != 0)
        {
            if(sensor_show_mode == 0)
            {
                sensor_show_mode = 1;
                OLED_Clear();
                OLED_ShowString(1, 1, "L3 L2 L1 MC");
                OLED_ShowString(3, 1, "R1 R2 R3");
            }

            /* 持续刷新传感器电平（标签在1/3行，数值在2/4行） */
            OLED_ShowNum(2, 1,  L3, 1);  OLED_ShowNum(2, 5,  L2, 1);
            OLED_ShowNum(2, 9,  L1, 1);  OLED_ShowNum(2, 13, MC, 1);
            OLED_ShowNum(4, 1,  R1, 1);  OLED_ShowNum(4, 5,  R2, 1);
            OLED_ShowNum(4, 9,  R3, 1);
        }
        /* 全白 → 恢复编码器显示 */
        else
        {
            if(sensor_show_mode == 1)
            {
                sensor_show_mode = 0;
                OLED_Clear();
                OLED_ShowString(1, 1, "Target:");
                OLED_ShowString(2, 1, "A RPM:");
                OLED_ShowString(3, 1, "B RPM:");
                OLED_ShowString(4, 1, "PID Fix:");
            }

            /* 持续刷新编码器数据 */
            speedA.rpm = PID_MotorA.Actual;
            speedB.rpm = PID_MotorB.Actual;

            OLED_ShowSignedNum(1, 9,  (int)PID_MotorA.Target, 5);
            OLED_ShowSignedNum(2, 9,  (int)speedA.rpm, 5);
            OLED_ShowSignedNum(3, 9,  (int)speedB.rpm, 5);
            OLED_ShowSignedNum(4, 9,  (int)PID_MotorA.Out, 5);
        }
    }
}

/*============================================================================
 * PID 定时器中断服务函数（20ms 周期）
 * 对应原始 TIM6_DAC_IRQHandler
 *===========================================================================*/
void PID_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PID_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:  /* 0 溢出中断 */
            PID_Control_Update(); /* 读编码器 → 算速度 → PID → 输出电机 */
            break;
        default:
            break;
    }
}

/*============================================================================
 * 寻路定时器中断服务函数（40ms 周期）
 * 对应原始 TIM7_IRQHandler
 *===========================================================================*/
void Pathfinding_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Pathfinding_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            Line_PID_Update(); /* 读传感器 → 算位置 → PID → 差速修正目标 */
            break;
        default:
            break;
    }
}
