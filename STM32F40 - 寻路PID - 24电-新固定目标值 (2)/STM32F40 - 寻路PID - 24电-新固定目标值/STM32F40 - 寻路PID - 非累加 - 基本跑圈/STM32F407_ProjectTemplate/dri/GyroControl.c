/**
  ******************************************************************************
  * @file    GyroControl.c
  * @brief   陀螺仪PID控制模块
  * @note    用于基于陀螺仪的转向控制，实现双环PID控制
  *          使用串口陀螺仪模块获取角度和角速度数据
  ******************************************************************************
  */
#include "GyroControl.h"
#include "GyroUART.h"
#include "Motor.h"
#include "PID.h"

/*
 * 陀螺仪方向切换
 * =============================================
 * 使用方法：修改 gyroDir 的值即可切换方向
 *
 * GYRO_DIR_NORMAL (0): 顺时针旋转时角度增加
 * GYRO_DIR_REVERSE (1): 逆时针旋转时角度增加
 *
 * 如果发现转向方向相反，把 GYRO_DIR_NORMAL 改成 GYRO_DIR_REVERSE 即可
 * =============================================
 */
#define GYRO_DIR_NORMAL     0   // 正常方向（顺时针角度增加）
#define GYRO_DIR_REVERSE    1   // 反向（逆时针角度增加）

/* ★ 修改这里可以快速切换陀螺仪方向 ★ */
uint8_t gyroDir = GYRO_DIR_NORMAL;  // 默认正常方向，如方向相反改为 GYRO_DIR_REVERSE

/* 全局变量定义 */
GyroPID_Angle_t GyroPID_Angle;
GyroPID_Gyro_t GyroPID_Gyro;
GyroControl_t GyroControl;

/**
  * @brief  陀螺仪控制初始化
  * @param  无
  * @retval 无
  */
void GyroControl_Init(void)
{
    /* 初始化串口陀螺仪 */
    GyroUART_Init();

    /* 初始化角度环PID参数 */
    GyroPID_Angle.Kp = 6.0f;
    GyroPID_Angle.Ki = 0.05f;
    GyroPID_Angle.Kd = 0.0f;
    GyroPID_Angle.Error = 0;
    GyroPID_Angle.LastError = 0;
    GyroPID_Angle.Integral = 0;
    GyroPID_Angle.IntegralMax = 300.0f;
    GyroPID_Angle.Output = 0;
    GyroPID_Angle.OutputMax = 200.0f;

    /* 初始化角速度环PID参数 */
    GyroPID_Gyro.Kp = 0.50f;
    GyroPID_Gyro.Ki = 0.035f;
    GyroPID_Gyro.Kd = 0.0f;
    GyroPID_Gyro.Error = 0;
    GyroPID_Gyro.LastError = 0;
    GyroPID_Gyro.Integral = 0;
    GyroPID_Gyro.IntegralMax = 80.0f;
    GyroPID_Gyro.Output = 0;
    GyroPID_Gyro.OutputMax = 200.0f;

    /* 初始化控制参数 */
    GyroControl.BaseSpeed = 70.0f;
    GyroControl.TargetYaw = 0.0f;
    GyroControl.CurrentYaw = 0.0f;
    GyroControl.CurrentGyro = 0.0f;
    GyroControl.TurnPWM = 0.0f;
    GyroControl.PWM_Left = 0.0f;
    GyroControl.PWM_Right = 0.0f;
    GyroControl.PWM_Max = 200.0f;
    GyroControl.PWM_Min = -200.0f;
    GyroControl.DeadZone = 2.0f;   // 角度死区(度)
}

/**
  * @brief  设置角度环PID参数
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @retval 无
  */
void GyroControl_SetPID_Angle(float Kp, float Ki, float Kd)
{
    GyroPID_Angle.Kp = Kp;
    GyroPID_Angle.Ki = Ki;
    GyroPID_Angle.Kd = Kd;
}

/**
  * @brief  设置角速度环PID参数
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @retval 无
  */
void GyroControl_SetPID_Gyro(float Kp, float Ki, float Kd)
{
    GyroPID_Gyro.Kp = Kp;
    GyroPID_Gyro.Ki = Ki;
    GyroPID_Gyro.Kd = Kd;
}

/**
  * @brief  设置控制目标
  * @param  baseSpeed: 基础速度
  * @param  targetYaw: 目标偏航角(度)
  * @retval 无
  */
void GyroControl_SetTarget(float baseSpeed, float targetYaw)
{
    GyroControl.BaseSpeed = baseSpeed;
    GyroControl.TargetYaw = targetYaw;
}

/**
  * @brief  角度归一化到 [-180, 180]
  * @param  angle: 输入角度(度)
  * @retval 归一化后的角度
  * @note   这个函数已经实现了最短路径计算
  *         因为归一化到 [-180, 180] 就是最短路径
  */
float GyroControl_NormalizeAngle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
  * @brief  限幅函数
  * @param  val: 输入值
  * @param  min: 最小值
  * @param  max: 最大值
  * @retval 限幅后的值
  */
float GyroControl_Limit(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
  * @brief  陀螺仪控制更新函数
  * @param  无
  * @retval 无
  * @note   需要在定时器中断中周期调用(建议10ms或20ms)
  *         实现双环PID控制:
  *         外环: 角度PID -> 输出期望角速度
  *         内环: 角速度PID -> 输出转向PWM
  *         死区: 直接作用于角度误差，误差在死区范围内不处理
  */
void GyroControl_Update(void)
{
    float desired_gyro;
    float angle_error, gyro_error;

    /* ========== 读取传感器数据 ========== */
    /* 从串口陀螺仪获取当前角度和角速度 */
    if (gyroDir == GYRO_DIR_REVERSE)
    {
        /* 反向模式：角度取反 */
        GyroControl.CurrentYaw = GyroControl_NormalizeAngle(-GyroUART_GetYaw());
        GyroControl.CurrentGyro = -GyroUART_GetGyroZ();  // 角速度也取反
    }
    else
    {
        /* 正常模式 */
        GyroControl.CurrentYaw = GyroControl_NormalizeAngle(GyroUART_GetYaw());
        GyroControl.CurrentGyro = GyroUART_GetGyroZ();
    }

    /* ========== 外环: 角度PID控制 ========== */
    /* 1. 计算角度误差（最短路径） */
    angle_error = GyroControl_NormalizeAngle(GyroControl.TargetYaw - GyroControl.CurrentYaw);

    /* 2. 角度保护死区 - 误差在死区范围内不处理 */
    /*    直接作用于角度误差，防止小车在目标角度附近抖动 */
    if (angle_error > -GyroControl.DeadZone &&
        angle_error < GyroControl.DeadZone)
    {
        angle_error = 0;  // 在死区内，角度误差清零
    }

    /* 3. 积分累加与限幅 */
    GyroPID_Angle.Integral += angle_error;
    GyroPID_Angle.Integral = GyroControl_Limit(GyroPID_Angle.Integral,
                                                -GyroPID_Angle.IntegralMax,
                                                GyroPID_Angle.IntegralMax);

    /* 4. 角度环PID输出 -> 期望角速度(°/s) */
    desired_gyro = GyroPID_Angle.Kp * angle_error
                 + GyroPID_Angle.Ki * GyroPID_Angle.Integral;

    GyroPID_Angle.LastError = angle_error;

    /* ========== 内环: 角速度PID控制 ========== */
    /* 5. 计算角速度误差 */
    gyro_error = desired_gyro - GyroControl.CurrentGyro;

    /* 6. 积分累加与限幅 */
    GyroPID_Gyro.Integral += gyro_error;
    GyroPID_Gyro.Integral = GyroControl_Limit(GyroPID_Gyro.Integral,
                                               -GyroPID_Gyro.IntegralMax,
                                               GyroPID_Gyro.IntegralMax);

    /* 7. 角速度环PID输出 -> 转向PWM */
    GyroControl.TurnPWM = GyroPID_Gyro.Kp * gyro_error
                        + GyroPID_Gyro.Ki * GyroPID_Gyro.Integral;

    GyroPID_Gyro.LastError = gyro_error;

    /* TurnPWM计算完成，由外部调用motor()设置目标速度 */
}

/**
  * @brief  重置陀螺仪控制状态
  * @param  无
  * @retval 无
  */
void GyroControl_Reset(void)
{
    /* 重置PID状态 */
    GyroPID_Angle.Error = 0;
    GyroPID_Angle.LastError = 0;
    GyroPID_Angle.Integral = 0;
    GyroPID_Angle.Output = 0;

    GyroPID_Gyro.Error = 0;
    GyroPID_Gyro.LastError = 0;
    GyroPID_Gyro.Integral = 0;
    GyroPID_Gyro.Output = 0;

    /* 重置控制状态 */
    GyroControl.TurnPWM = 0;
    GyroControl.PWM_Left = 0;
    GyroControl.PWM_Right = 0;
}
