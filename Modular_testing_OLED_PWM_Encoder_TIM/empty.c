#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "OLED.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "Speed.h"
#include "PID.h"
#include "PID_Control.h"

/* 简单延时 */
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

/* 两个编码器各自的速度数据（用于显示） */
Speed_Data speedA = {0, 0.0f};
Speed_Data speedB = {0, 0.0f};

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    Motor_Init();
    PID_Control_Init();

    /* EncoderA = 硬件编码器 (TIMG8 QEI, PA26/PA27, 4倍频) */
    Encoder_Init();

    /* EncoderB = GPIO 中断编码器 (PA28/PA31, 2倍频) */
    Encoder_GPIO_Init();

    /* 设置目标速度 */
    PID_Control_SetTarget(50.0f, 50.0f);

    /* 使能 PID 定时器中断 */
    NVIC_EnableIRQ(PID_TIM_INST_INT_IRQN);

    OLED_ShowString(1, 1, "Target:");
    OLED_ShowString(2, 1, "A RPM:");
    OLED_ShowString(3, 1, "B RPM:");
    OLED_ShowString(4, 1, "PID Fix:");

    while (1)
    {
        /* 主循环只做显示，PID 控制在定时器中断里 */

        /* 更新速度显示（从 PID 控制模块读取） */
        speedA.rpm = PID_MotorA.Actual;
        speedB.rpm = PID_MotorB.Actual;

        /* 显示 */
        OLED_ShowSignedNum(1, 9,  (int)PID_MotorA.Target, 5);
        OLED_ShowSignedNum(2, 9,  (int)speedA.rpm, 5);
        OLED_ShowSignedNum(3, 9,  (int)speedB.rpm, 5);
        OLED_ShowSignedNum(4, 9,  (int)PID_MotorA.Out, 5);

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
