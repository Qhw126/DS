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

    OLED_ShowString(1, 1, "Target:");
    OLED_ShowString(2, 1, "A RPM:");
    OLED_ShowString(3, 1, "B RPM:");
    OLED_ShowString(4, 1, "PID Out:");

    while (1)
    {
        /* 获取编码器增量 */
        int16_t pulseA = Encoder_Get();
        int16_t pulseB = Encoder_GPIO_Get();

        /* 更新速度（用于显示） */
        Speed_Update(&speedA, pulseA, 1);
        Speed_Update(&speedB, pulseB, 1);

        /* PID 控制更新（传入脉冲增量，内部会转换为速度） */
        PID_Control_Update(pulseA, pulseB);

        /* 显示 */
        OLED_ShowSignedNum(1, 9,  (int)PID_MotorA.Target, 5);
        OLED_ShowSignedNum(2, 9,  (int)speedA.rpm, 5);
        OLED_ShowSignedNum(3, 9,  (int)speedB.rpm, 5);
        OLED_ShowSignedNum(4, 9,  (int)PID_MotorA.Out, 5);

        delay_ms(20);  /* 20ms 控制周期 */
    }
}
