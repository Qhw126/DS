#ifndef __SPEED_H
#define __SPEED_H

#include <stdint.h>

/*============================================================================
 * 速度计算参数宏定义
 *===========================================================================*/

/* 编码器参数 */
#define ENCODER_PPR             265.2f  /* 编码器每转脉冲数（已含减速比） */
#define ENCODING_MODE           2       /* 倍频数：1=单倍频, 2=二倍频, 4=四倍频 */

/* 采样参数 */
#define SPEED_SAMPLE_TIME       0.02f   /* 采样周期（秒），20ms */

/* 速度转换系数 */
#define PULSE_TO_RPM    (60.0f / (ENCODER_PPR * ENCODING_MODE * SPEED_SAMPLE_TIME))

/* 速度结构体 */
typedef struct {
    int16_t pulse_count;    /* 原始脉冲计数 */
    float   rpm;            /* 转速（RPM） */
} Speed_Data;

/*============================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief  更新速度数据
 * @param  data 速度数据结构体指针
 * @param  encoder_count 编码器增量值
 * @param  dir 方向系数（1 或 -1）
 */
void Speed_Update(Speed_Data *data, int16_t encoder_count, int8_t dir);

#endif
