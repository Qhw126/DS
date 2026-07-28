#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/**
 * @brief  编码器初始化
 * @param  无
 * @retval 无
 */
void Encoder_Init(void);

/**
 * @brief  获取编码器增量值（读后清零）
 * @param  无
 * @retval 增量值
 */
int16_t Encoder_Get(void);

/**
 * @brief  获取编码器计数值
 * @param  无
 * @retval 编码器计数值（有符号）
 */
int16_t Encoder_GetCount(void);

/**
 * @brief  清零编码器计数值
 * @param  无
 * @retval 无
 */
void Encoder_ClearCount(void);

/**
 * @brief  获取编码器速度（脉冲/采样周期）
 * @param  无
 * @retval 速度值
 */
int16_t Encoder_GetSpeed(void);

#endif
