/**
  * @file    line.c
  * @brief   循线模块（对齐参考工程 dri/line.c）
  * @note    增加 controlMode 切换：丢线 → 陀螺仪模式 → 找回线 → 循线模式
  ******************************************************************************
  */
#include "line.h"
#include "sensor.h"
#include "GyroControl.h"

/* 寻路 PID 实例定义（已移至 PID.c） */

/* 基础速度（RPM） */
float base_speed = 70.0f;

/* 控制模式相关变量定义 */
uint8_t  controlMode     = MODE_LINE_TRACK;
uint8_t  controlModelast = 0;
float    yawWhenLost     = 0;
float    targetYaw       = 0;
uint8_t  isLineLost      = 1;
uint8_t  gyroToLineFlag  = 0;
uint32_t gyroToLineTick  = 0;

/**
  * @brief  寻路位置计算（加权平均法）
  *
  * 7路传感器加权位置：
  *   D1(L3)=+9  D2(L2)=+3  D3(L1)=+1.25  D4(MC)=0
  *   D5(R1)=-1.25  D6(R2)=-3  D7(R3)=-9
  *
  * 对齐参考工程 track_zhixian1()：
  *   1-2个传感器触发：加权平均算偏移
  *   3+个传感器触发：路口，强制急转
  *   0个传感器触发：丢线，切换陀螺仪模式
  */
void track_zhixian1(void)
{
    static const float pos[7] = {9, 3, 1.25, 0, -1.25, -3, -9};
    float sum_weight = 0;
    float sum_pos = 0;

    for (int i = 0; i < 7; i++)
    {
        if (digtal(i + 1) == 1)
        {
            sum_weight += 1;
            sum_pos += pos[i];
        }
    }

    if (sum_weight > 0 && sum_weight < 3)
    {
        PID_findway.Actual = sum_pos / sum_weight;
        controlMode = MODE_LINE_TRACK;

        if (gyroToLineFlag == 0)
        {
            if (controlModelast == MODE_GYRO_TURN && controlMode == MODE_LINE_TRACK)
            {
                isLineLost++;
                controlModelast = MODE_LINE_TRACK;
                gyroToLineFlag = 1;
            }

            if (isLineLost % 2 == 1)
                targetYaw = GyroControl_NormalizeAngle(yawWhenLost + 180);
            else if (isLineLost % 2 == 0)
                targetYaw = GyroControl_NormalizeAngle(yawWhenLost);
        }
    }
    else if (sum_weight >= 3)
    {
        controlMode = MODE_LINE_TRACK;

        if (gyroToLineFlag == 0)
        {
            if (controlModelast == MODE_GYRO_TURN && controlMode == MODE_LINE_TRACK)
            {
                isLineLost++;
                controlModelast = MODE_LINE_TRACK;
                gyroToLineFlag = 1;
            }

            if (isLineLost % 2 == 1)
                targetYaw = GyroControl_NormalizeAngle(yawWhenLost + 180);
            if (isLineLost % 2 == 0)
                targetYaw = GyroControl_NormalizeAngle(yawWhenLost);
        }

        if (sum_pos > 0)
            PID_findway.Actual = 10;
        else
            PID_findway.Actual = -10;
    }
    else
    {
        /* 0个传感器触发 */
        if (gyroToLineFlag == 0)
        {
            /* 丢线 → 切换到陀螺仪模式 */
            controlMode = MODE_GYRO_TURN;
            controlModelast = MODE_GYRO_TURN;
        }
        else if (gyroToLineFlag == 1)
        {
            /* 保护中，用上次方向继续 */
            if (PID_findway.Actual >= 8)
                PID_findway.Actual = 10;
            else if (PID_findway.Actual <= -8)
                PID_findway.Actual = -10;
            /* else: 保持当前 Actual 不变 */
        }
    }
}

/**
  * @brief  检查是否重新检测到线
  * @retval 1=检测到线, 0=未检测到线
  */
uint8_t checkLineFound(void)
{
    for (int i = 1; i <= 7; i++)
    {
        if (digtal(i) == 1)
            return 1;
    }
    return 0;
}

/**
  * @brief  设置寻路基础速度
  */
void BaseSpeed_Set(float speed)
{
    base_speed = speed;
}
