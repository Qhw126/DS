/**
 * @file    Encoder.c
 * @brief   硬件编码器驱动 - 使用 TIMG8 QEI 模式
 * @note    引脚：PA26 → TIMG8_CCP0 (PHA)
 *                PA27 → TIMG8_CCP1 (PHB)
 *          倍频：4倍频（双边沿计数，每转一圈计数 = PPR × 4）
 */

#include "ti_msp_dl_config.h"
#include "Encoder.h"

/**
 * @brief  编码器初始化
 * @param  无
 * @retval 无
 */
void Encoder_Init(void)
{
    /* 复位 TIMG8 */
    DL_TimerG_reset(TIMG8);

    /* 使能 TIMG8 电源 */
    DL_TimerG_enablePower(TIMG8);
    delay_cycles(16);

    /* 配置 GPIO 引脚为 TIMG8 输入功能 */
    /* PA26 → TIMG8_CCP0 (PHA), function 4 */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM59, IOMUX_PINCM59_PF_TIMG8_CCP0);
    /* PA27 → TIMG8_CCP1 (PHB), function 4 */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM60, IOMUX_PINCM60_PF_TIMG8_CCP1);

    /* 配置 TIMG8 时钟（使用 BUSCLK，不分频） */
    DL_TimerG_setClockConfig(TIMG8,
        &(DL_TimerG_ClockConfig){
            .clockSel = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale = 0
        });

    /* 配置 TIMG8 为 QEI 模式 - 需要分别配置 CC0 和 CC1 */
    DL_TimerG_configQEI(TIMG8,
        DL_TIMER_QEI_MODE_2_INPUT,      /* 2输入模式：PHA + PHB */
        DL_TIMER_CC_INPUT_INV_NOINVERT, /* 不反转 */
        DL_TIMER_CC_0_INDEX             /* 配置 CC0 */
    );
    DL_TimerG_configQEI(TIMG8,
        DL_TIMER_QEI_MODE_2_INPUT,      /* 2输入模式：PHA + PHB */
        DL_TIMER_CC_INPUT_INV_NOINVERT, /* 不反转 */
        DL_TIMER_CC_1_INDEX             /* 配置 CC1 */
    );

    /* 设置计数器最大值 */
    DL_TimerG_setLoadValue(TIMG8, 0xFFFF);

    /* 使能时钟 */
    DL_TimerG_enableClock(TIMG8);

    /* 启动计数器 */
    DL_TimerG_startCounter(TIMG8);
}

/**
 * @brief  获取编码器增量值（读后清零）
 * @param  无
 * @retval 增量值
 */
int16_t Encoder_Get(void)
{
    int16_t Temp;
    Temp = (int16_t)DL_TimerG_getTimerCount(TIMG8);
    DL_TimerG_setTimerCount(TIMG8, 0);
    return Temp;
}

/**
 * @brief  获取编码器计数值
 * @param  无
 * @retval 编码器计数值（有符号）
 */
int16_t Encoder_GetCount(void)
{
    return (int16_t)DL_TimerG_getTimerCount(TIMG8);
}

/**
 * @brief  清零编码器计数值
 * @param  无
 * @retval 无
 */
void Encoder_ClearCount(void)
{
    DL_TimerG_setTimerCount(TIMG8, 0);
}

/**
 * @brief  获取编码器速度（脉冲/采样周期）
 * @param  无
 * @retval 速度值
 */
int16_t Encoder_GetSpeed(void)
{
    static int16_t lastCount = 0;
    int16_t currentCount = Encoder_GetCount();
    int16_t speed = currentCount - lastCount;
    lastCount = currentCount;
    return speed;
}
