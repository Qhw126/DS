/**
  ******************************************************************************
  * @file    GyroControl.c
  * @brief   陀螺仪PID控制模块（双环PID：角度外环→角速度内环→TurnPWM）
  * @note    移植自 STM32 参考工程 dri/GyroControl.c
  *          STM32 API 替换为 MSP API:
  *            GyroUART_GetYaw()  → Gyro_GetYaw()
  *            GyroUART_GetGyroZ() → Gyro_GetRateZ()
  ******************************************************************************
  */
#include "GyroControl.h"
#include "gyro.h"

/* 陀螺仪方向切换 */
uint8_t gyroDir = GYRO_DIR_NORMAL;

/* 全局变量定义 */
GyroPID_Angle_t GyroPID_Angle;
GyroPID_Gyro_t  GyroPID_Gyro;
GyroControl_t   GyroControl;

/**
  * @brief  陀螺仪控制初始化
  */
void GyroControl_Init(void)
{
    /* 初始化角度环PID参数 */
    GyroPID_Angle.Kp          = 6.0f;
    GyroPID_Angle.Ki          = 0.05f;
    GyroPID_Angle.Kd          = 0.0f;
    GyroPID_Angle.Error       = 0;
    GyroPID_Angle.LastError   = 0;
    GyroPID_Angle.Integral    = 0;
    GyroPID_Angle.IntegralMax = 300.0f;
    GyroPID_Angle.Output      = 0;
    GyroPID_Angle.OutputMax   = 200.0f;

    /* 初始化角速度环PID参数 */
    GyroPID_Gyro.Kp          = 0.50f;
    GyroPID_Gyro.Ki          = 0.035f;
    GyroPID_Gyro.Kd          = 0.0f;
    GyroPID_Gyro.Error       = 0;
    GyroPID_Gyro.LastError   = 0;
    GyroPID_Gyro.Integral    = 0;
    GyroPID_Gyro.IntegralMax = 80.0f;
    GyroPID_Gyro.Output      = 0;
    GyroPID_Gyro.OutputMax   = 200.0f;

    /* 初始化控制参数 */
    GyroControl.BaseSpeed  = 70.0f;
    GyroControl.TargetYaw  = 0.0f;
    GyroControl.CurrentYaw = 0.0f;
    GyroControl.CurrentGyro = 0.0f;
    GyroControl.TurnPWM    = 0.0f;
    GyroControl.PWM_Left   = 0.0f;
    GyroControl.PWM_Right  = 0.0f;
    GyroControl.PWM_Max    = 200.0f;
    GyroControl.PWM_Min    = -200.0f;
    GyroControl.DeadZone   = 2.0f;
}

/**
  * @brief  设置角度环PID参数
  */
void GyroControl_SetPID_Angle(float Kp, float Ki, float Kd)
{
    GyroPID_Angle.Kp = Kp;
    GyroPID_Angle.Ki = Ki;
    GyroPID_Angle.Kd = Kd;
}

/**
  * @brief  设置角速度环PID参数
  */
void GyroControl_SetPID_Gyro(float Kp, float Ki, float Kd)
{
    GyroPID_Gyro.Kp = Kp;
    GyroPID_Gyro.Ki = Ki;
    GyroPID_Gyro.Kd = Kd;
}

/**
  * @brief  设置控制目标
  */
void GyroControl_SetTarget(float baseSpeed, float targetYaw)
{
    GyroControl.BaseSpeed = baseSpeed;
    GyroControl.TargetYaw = targetYaw;
}

/**
  * @brief  角度归一化到 [-180, 180]
  */
float GyroControl_NormalizeAngle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
  * @brief  限幅函数
  */
float GyroControl_Limit(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
  * @brief  陀螺仪控制更新函数（双环PID）
  * @note   需在定时器中断中周期调用(40ms)
  *         外环: 角度PID → 期望角速度
  *         内环: 角速度PID → 转向PWM
  */
void GyroControl_Update(void)
{
    float desired_gyro;
    float angle_error, gyro_error;

    /* ===== 读取传感器数据 ===== */
    if (gyroDir == GYRO_DIR_REVERSE)
    {
        GyroControl.CurrentYaw  = GyroControl_NormalizeAngle(-Gyro_GetYaw());
        GyroControl.CurrentGyro = -Gyro_GetRateZ();
    }
    else
    {
        GyroControl.CurrentYaw  = GyroControl_NormalizeAngle(Gyro_GetYaw());
        GyroControl.CurrentGyro = Gyro_GetRateZ();
    }

    /* ===== 外环: 角度PID ===== */
    angle_error = GyroControl_NormalizeAngle(GyroControl.TargetYaw - GyroControl.CurrentYaw);

    /* 死区 */
    if (angle_error > -GyroControl.DeadZone &&
        angle_error <  GyroControl.DeadZone)
    {
        angle_error = 0;
    }

    GyroPID_Angle.Integral += angle_error;
    GyroPID_Angle.Integral  = GyroControl_Limit(GyroPID_Angle.Integral,
                                                 -GyroPID_Angle.IntegralMax,
                                                  GyroPID_Angle.IntegralMax);

    desired_gyro = GyroPID_Angle.Kp * angle_error
                 + GyroPID_Angle.Ki * GyroPID_Angle.Integral;

    GyroPID_Angle.LastError = angle_error;

    /* ===== 内环: 角速度PID ===== */
    gyro_error = desired_gyro - GyroControl.CurrentGyro;

    GyroPID_Gyro.Integral += gyro_error;
    GyroPID_Gyro.Integral  = GyroControl_Limit(GyroPID_Gyro.Integral,
                                                -GyroPID_Gyro.IntegralMax,
                                                 GyroPID_Gyro.IntegralMax);

    GyroControl.TurnPWM = GyroPID_Gyro.Kp * gyro_error
                        + GyroPID_Gyro.Ki * GyroPID_Gyro.Integral;

    GyroPID_Gyro.LastError = gyro_error;
}

/**
  * @brief  重置陀螺仪控制状态
  */
void GyroControl_Reset(void)
{
    GyroPID_Angle.Error     = 0;
    GyroPID_Angle.LastError = 0;
    GyroPID_Angle.Integral  = 0;
    GyroPID_Angle.Output    = 0;

    GyroPID_Gyro.Error     = 0;
    GyroPID_Gyro.LastError = 0;
    GyroPID_Gyro.Integral  = 0;
    GyroPID_Gyro.Output    = 0;

    GyroControl.TurnPWM   = 0;
    GyroControl.PWM_Left  = 0;
    GyroControl.PWM_Right = 0;
}
