/**
  * @file    line.c
  * @brief   循线模块
  ******************************************************************************
  */
#include "stm32f4xx.h"
#include <stdint.h>
#include "line.h"
#include "Motor.h"
#include "sensor.h"
#include "PWM.h"
#include "board.h"
#include "PID.h"
#include "OLED.h"
#include "GyroUART.h"
#include "GyroControl.h"

/* 全局变量定义 */
uint8_t controlMode = MODE_LINE_TRACK;  // 当前控制模式
uint8_t controlModelast = 0;			// 上一次控制模式
float yawWhenLost = 0;                   // 上电时记录的偏航角
float targetYaw = 0;                     // 目标偏航角(丢线时角度+180)
uint8_t isLineLost = 1;                  // 丢线标志

/* 2秒保护相关变量 */
uint8_t gyroToLineFlag = 0;       // 保护标志 (0=无保护, 1=保护中)
uint32_t gyroToLineTick = 0;      // 记录切回循线时的计数值


/**
  * @brief  循线函数
  * @param  无
  * @retval 无
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
        if(gyroToLineFlag == 0)
		{
			if (controlModelast == MODE_GYRO_TURN && controlMode == MODE_LINE_TRACK)
			{
				isLineLost++;
				controlModelast = MODE_LINE_TRACK;
				gyroToLineFlag = 1;
			}
			
			if (isLineLost%2 == 1)
			{
				targetYaw = GyroControl_NormalizeAngle(yawWhenLost + 180);
			}
			else if (isLineLost%2 == 0)
			{
				targetYaw = GyroControl_NormalizeAngle(yawWhenLost);
			}
		}
	}
    else if (sum_weight >= 3)
    {
        controlMode = MODE_LINE_TRACK;
        if(gyroToLineFlag == 0)
		{
			if (controlModelast == MODE_GYRO_TURN && controlMode == MODE_LINE_TRACK)
			{
				isLineLost++;
				controlModelast = MODE_LINE_TRACK;
				gyroToLineFlag = 1;
			}
			
			if (isLineLost%2 == 1)
			{
				targetYaw = GyroControl_NormalizeAngle(yawWhenLost + 180);
			}
			if (isLineLost%2 == 0)
			{
				targetYaw = GyroControl_NormalizeAngle(yawWhenLost);
			}
		}
        if (sum_pos > 0)
            PID_findway.Actual = 10;
        else
            PID_findway.Actual = -10;
    }
    else
    {
		if(gyroToLineFlag == 0)
		{
			/* 丢线 - 切换到陀螺仪模式 */
			controlMode = MODE_GYRO_TURN;
			controlModelast = MODE_GYRO_TURN;
		}
		else if(gyroToLineFlag == 1)
		{
			if(PID_findway.Actual >= 8)
			{
				PID_findway.Actual = 10;
			}
			else if(PID_findway.Actual <= -8)
			{
				PID_findway.Actual = -10;
			}
			else
			{
				PID_findway.Actual = PID_findway.Actual;
			}
		}
        
    }
}

/**
  * @brief  检查是否重新检测到线
  * @param  无
  * @retval 1-检测到线, 0-未检测到线
  */
uint8_t checkLineFound(void)
{
    for (int i = 1; i <= 7; i++)
    {
        if (digtal(i) == 1)
            return 1;  // 检测到线
    }
    return 0;  // 未检测到线
}
