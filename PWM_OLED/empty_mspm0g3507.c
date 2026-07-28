/*
 * 呼吸灯 - PWM 渐变亮度
 *
 * 引脚：PA8 (TIMA0 CCP0)
 *       PA1 (TIMA0 CCP1)
 */

#include "ti_msp_dl_config.h"
#include "oled.h"

/* 呼吸灯参数 */
#define PWM_PERIOD    1000
#define BREATH_STEP   1       /* 每步亮度变化量 */
#define BREATH_DELAY  2000    /* 每步延时(微秒)，值越小呼吸越快 */

static void delay_us(uint32_t us)
{
    /* 粗略微秒延时 (80MHz 主频) */
    delay_cycles(us * 20);
}

void delay_ms(uint32_t ms)
{
    delay_cycles(ms * 20000);
}

int main(void)
{
    SYSCFG_DL_init();

    /* 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"PWM Test", 16);

    int32_t brightness = 0;
    int32_t direction  = 1;   /* 1=渐亮, -1=渐暗 */
    uint32_t count = 0;

    while (1)
    {
         /* 更新占空比 */
         brightness += direction * BREATH_STEP;

         /* 到达边界，反转方向 */
         if (brightness >= PWM_PERIOD)
         {
             brightness = PWM_PERIOD;
             direction = -1;
         }
         else if (brightness <= 0)
         {
             brightness = 0;
             direction = 1;
         }

        /* 同时设置两个通道 */
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, (uint32_t)1000, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, (uint32_t)brightness, DL_TIMER_CC_1_INDEX);

        /* 每1000次循环更新一次OLED显示 */
        if (++count >= 1000) {
            count = 0;
            OLED_ShowNum(0, 40, (u32)brightness, 4, 16);
            OLED_Refresh();
        }

        delay_us(BREATH_DELAY);
    }
}
