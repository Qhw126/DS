#include "PID.h"

/*============================================================================
 * 位置式 PID
 *
 * 公式：Out = Kp × e(k) + Ki × Σe(k) + Kd × [e(k) - e(k-1)]
 *
 * 特点：
 *   - 输出与位置相关
 *   - 有积分累加，适合需要精确位置控制的场景
 *   - 可能会有积分饱和问题
 *===========================================================================*/
void Position_PID(PID_Position *pid)
{
    /* 1. 计算当前误差 */
    pid->Error_0 = pid->Target - pid->Actual;

    /* 2. 积分分离：误差小时累加积分，误差大时清零（防饱和） */
    if (pid->Error_0 < 3.0f && pid->Error_0 > -3.0f)
        pid->Integral += pid->Error_0;  /* 误差小，继续累加 */
    else
        pid->Integral = 0;              /* 误差大，清零积分 */

    /* 积分限幅 */
    if (pid->Integral > pid->I_MAX) pid->Integral = pid->I_MAX;
    if (pid->Integral < pid->I_MIN) pid->Integral = pid->I_MIN;

    /* 3. 计算输出 */
    pid->Out = pid->Kp * pid->Error_0                      /* P: 比例 */
             + pid->Ki * pid->Integral                     /* I: 积分 */
             + pid->Kd * (pid->Error_0 - pid->Error_1);    /* D: 微分 */

    /* 4. 更新历史误差 */
    pid->Error_1 = pid->Error_0;

    /* 5. 输出限幅 */
    if (pid->Out > pid->OutMax) pid->Out = pid->OutMax;
    if (pid->Out < pid->OutMin) pid->Out = pid->OutMin;
}

/*============================================================================
 * 增量式 PID
 *
 * 公式：ΔOut = Kp×[e(k)-e(k-1)] + Ki×e(k) + Kd×[e(k)-2e(k-1)+e(k-2)]
 *       Out += ΔOut
 *
 * 特点：
 *   - 输出增量，不需要积分累加
 *   - 无积分饱和问题
 *   - 适合电机速度控制
 *===========================================================================*/
void Incremental_PID(PID_Incremental *pid)
{
    /* 1. 更新历史误差 */
    pid->Error2 = pid->Error1;  /* e(k-2) = e(k-1) */
    pid->Error1 = pid->Error0;  /* e(k-1) = e(k)   */
    pid->Error0 = pid->Target - pid->Actual;  /* e(k) = 目标 - 实际 */

    /* 2. 计算增量 */
    /* ΔOut = Kp×[e(k)-e(k-1)] + Ki×e(k) + Kd×[e(k)-2e(k-1)+e(k-2)] */
    float DeltaOut = pid->Kp * (pid->Error0 - pid->Error1)
                   + pid->Ki * pid->Error0
                   + pid->Kd * (pid->Error0 - 2 * pid->Error1 + pid->Error2);

    /* 3. 累加输出 */
    pid->Out += DeltaOut;

    /* 4. 输出限幅 */
    if (pid->Out > pid->OutMax) pid->Out = pid->OutMax;
    if (pid->Out < pid->OutMin) pid->Out = pid->OutMin;
}
