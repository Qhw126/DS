#ifndef __LINE_H
#define __LINE_H

#include "PID.h"

/* 传感器数量 */
#define ADC_N 7

/* 基础速度（RPM） */
extern float base_speed;

/* 寻路 PID 实例 */
extern PID_Position PID_findway;

/* 函数声明 */
void track_zhixian1(void);
void Line_PID_Init(void);
void Line_PID_Update(void);

/**
 * @brief  设置寻路基础速度
 * @param  speed 基础速度（RPM）
 */
void BaseSpeed_Set(float speed);

#endif
