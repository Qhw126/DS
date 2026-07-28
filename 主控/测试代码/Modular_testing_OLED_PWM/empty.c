#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "OLED.h"

/* 简单延时 */
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Motor Test");
    Motor_Init();

    while (1)
    {
        /* 正转 50% */
        Motor_SetSpeed(MOTOR_A, 100);
        OLED_ShowString(2, 1, "Forward 10%  ");
        delay_ms(2000);

        /* 停止 */
        Motor_SetSpeed(MOTOR_A, 0);
        OLED_ShowString(2, 1, "Stop         ");
        delay_ms(1000);

        /* 反转 50% */
        Motor_SetSpeed(MOTOR_A, -100);
        OLED_ShowString(2, 1, "Reverse 10%  ");
        delay_ms(2000);

        /* 停止 */
        Motor_SetSpeed(MOTOR_A, 0);
        OLED_ShowString(2, 1, "Stop         ");
        delay_ms(1000);
    }
}
