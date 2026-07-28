#ifndef __ENCODER_GPIO_H
#define __ENCODER_GPIO_H

#include <stdint.h>

/**
 * @brief  GPIO 中断编码器初始化
 * @param  无
 * @retval 无
 */
void Encoder_GPIO_Init(void);

/**
 * @brief  获取编码器计数值
 * @param  无
 * @retval 计数值（有符号）
 */
int32_t Encoder_GPIO_GetCount(void);

/**
 * @brief  清零编码器计数值
 * @param  无
 * @retval 无
 */
void Encoder_GPIO_ClearCount(void);

/**
 * @brief  获取编码器增量值（读后清零）
 * @param  无
 * @retval 增量值
 */
int16_t Encoder_GPIO_Get(void);

#endif
