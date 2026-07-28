#ifndef __PID_H
#define __PID_H

/*============================================================================
 * PID 控制器模块
 * 支持：位置式 PID 和 增量式 PID
 *===========================================================================*/

/* 位置式 PID 结构体 */
typedef struct {
    float Target;       /* 目标值 */
    float Actual;       /* 实际值 */
    float Out;          /* 输出值 */
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    float Error_0;      /* 当前误差 */
    float Error_1;      /* 上一次误差 */
    float Error_2;      /* 上上次误差 */
    float Integral;     /* 积分累加值 */
    float OutMax;       /* 输出上限 */
    float OutMin;       /* 输出下限 */
    float I_MAX;        /* 积分上限 */
    float I_MIN;        /* 积分下限 */
} PID_Position;

/* 增量式 PID 结构体 */
typedef struct {
    float Target;       /* 目标值 */
    float Actual;       /* 实际值 */
    float Out;          /* 输出值 */
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    float Error0;       /* 当前误差 e(k) */
    float Error1;       /* 上一次误差 e(k-1) */
    float Error2;       /* 上上次误差 e(k-2) */
    float Target_Iint;  /* 初始目标值 */
    float OutMax;       /* 输出上限 */
    float OutMin;       /* 输出下限 */
} PID_Incremental;

/*============================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief  位置式 PID 计算
 * @param  pid PID 结构体指针
 * @note   输出 = Kp×误差 + Ki×积分 + Kd×微分
 */
void Position_PID(PID_Position *pid);

/**
 * @brief  增量式 PID 计算
 * @param  pid PID 结构体指针
 * @note   输出增量 = Kp×(本次误差-上次误差) + Ki×本次误差 + Kd×(本次-2×上次+上上次)
 */
void Incremental_PID(PID_Incremental *pid);

#endif
