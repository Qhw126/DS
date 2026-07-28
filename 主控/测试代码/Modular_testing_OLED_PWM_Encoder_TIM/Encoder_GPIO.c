#include "ti_msp_dl_config.h"
#include "Encoder_GPIO.h"

/**
 * @note    引脚：PA28 (A相), PA31 (B相)
 *          倍频：2倍频（仅上升沿计数，每转一圈计数 = PPR × 2）
 */

/* 编码器位置 */
static volatile int32_t encoder_count = 0;

/**
 * @brief  GPIO 中断编码器初始化
 */
void Encoder_GPIO_Init(void)
{
    encoder_count = 0;
    NVIC_ClearPendingIRQ(Encoders_INT_IRQN);
    NVIC_EnableIRQ(Encoders_INT_IRQN);
}

/**
 * @brief  获取编码器计数值
 */
int32_t Encoder_GPIO_GetCount(void)
{
    return encoder_count;
}

/**
 * @brief  清零编码器计数值
 */
void Encoder_GPIO_ClearCount(void)
{
    encoder_count = 0;
}

/**
 * @brief  获取编码器增量值（读后清零）
 */
int16_t Encoder_GPIO_Get(void)
{
    int16_t temp = (int16_t)encoder_count;
    encoder_count = 0;
    return temp;
}

/**
 * @brief  GPIO 中断服务函数
 *         E1 (PA28/A相) 上升沿：读 B 相判断方向
 *         E2 (PA31/B相) 上升沿：读 A 相判断方向
 */
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_interrupt;
    gpio_interrupt = DL_GPIO_getEnabledInterruptStatus(Encoders_PORT,
                        Encoders_E1_PIN | Encoders_E2_PIN);

    /* E1 (PA28 = A相) 上升沿 */
    if (gpio_interrupt & Encoders_E1_PIN)
    {
        if (!DL_GPIO_readPins(Encoders_PORT, Encoders_E2_PIN))
            encoder_count++;   /* B=0 → 正转 */
        else
            encoder_count--;   /* B=1 → 反转 */
    }
    /* E2 (PA31 = B相) 上升沿 */
    else if (gpio_interrupt & Encoders_E2_PIN)
    {
        if (!DL_GPIO_readPins(Encoders_PORT, Encoders_E1_PIN))
            encoder_count--;   /* A=0 → 反转 */
        else
            encoder_count++;   /* A=1 → 正转 */
    }

    /* 清除中断标志 */
    DL_GPIO_clearInterruptStatus(Encoders_PORT, Encoders_E1_PIN | Encoders_E2_PIN);
    __NVIC_ClearPendingIRQ(Encoders_INT_IRQN);
}
