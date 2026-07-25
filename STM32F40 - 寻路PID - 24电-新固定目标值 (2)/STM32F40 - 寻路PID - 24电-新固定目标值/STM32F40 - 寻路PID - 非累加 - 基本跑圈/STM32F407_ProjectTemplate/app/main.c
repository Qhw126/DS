/**
  * @file    main.c
  * @brief   主程序 - 显示陀螺仪偏航角和目标角度
  ******************************************************************************
  */
#include "board.h"
#include "bsp_uart.h"
#include <stdio.h>
#include "PID.h"
#include "PWM.h"
#include "Encoder.h"
#include "OLED.h"
#include "Motor.h"
#include "time.h"
#include "line.h"
#include "sensor.h"
#include "GyroUART.h"
#include "GyroControl.h"

/* 外部变量 */
extern float targetYaw;
extern uint8_t controlMode;
extern uint8_t controlModelast;
extern uint8_t isLineLost;
extern uint8_t gyroToLineFlag;
extern uint32_t timer6Count;

int main(void)
{
    board_init();
    OLED_Init();
    SENSOR_GPIO_Config();
    PWM_Init_3_4();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    Motor_Init();
    PID_Iint();
    time6_Init();
    time7_Init();

    /* 初始化串口(用于陀螺仪通信) - USART3: PD8(TX), PD9(RX) */
    uart3_init(115200);

    /* 初始化陀螺仪 */
    GyroUART_Init();

    /* 初始化陀螺仪控制模块(PID参数等) */
    GyroControl_Init();

    OLED_Clear();
    /* 行号:  1234567890123456 */
    OLED_ShowString(1, 1, "M:  L:  F:  ML:");  // 第1行：模式标志
    OLED_ShowString(2, 1, "Ct:   S:   A:");     // 第2行：计数/传感器/PID
    OLED_ShowString(3, 1, "Yaw:");               // 第3行：当前角度
    OLED_ShowString(4, 1, "Tgt:");               // 第4行：目标角度

    yawWhenLost = GyroUART_GetYaw();

    while (1)
    {
        /* ===== 计算传感器权重和 ===== */
        float sum_weight = 0;
        for (int ch = 1; ch <= 7; ch++)
        {
            if (digtal(ch) == 1)
                sum_weight += 1;
        }

        /* ===== 第1行：状态标志位 ===== */
        /* M:控制模式  L:isLineLost  F:gyroToLineFlag  ML:controlModelast */
        OLED_ShowNum(1, 3, controlMode, 1);        // M:0=LINE 1=GYRO
        OLED_ShowNum(1, 6, isLineLost, 2);         // L:丢线次数
        OLED_ShowNum(1, 10, gyroToLineFlag, 1);    // F:保护标志
        OLED_ShowNum(1, 14, controlModelast, 1);    // ML:上次模式

        /* ===== 第2行：计数器/传感器/PID ===== */
        /* Ct:timer6Count  S:sum_weight  A:PID_findway.Actual */
        OLED_ShowNum(2, 4, timer6Count, 2);         // Ct:保护计数
        OLED_ShowNum(2, 8, (uint32_t)sum_weight, 1);// S:传感器权重和
        OLED_ShowSignedNum(2, 12, (int)PID_findway.Actual, 3); // A:PID实际值

        /* ===== 第3行：当前偏航角 ===== */
        float yaw = GyroUART_GetYaw();
        OLED_ShowSignedNum(3, 5, (int)yaw, 4);
        int yaw_d = (int)((yaw - (int)yaw) * 10);
        if (yaw_d < 0) yaw_d = -yaw_d;
        OLED_ShowString(3, 10, ".");
        OLED_ShowNum(3, 11, yaw_d, 1);

        /* ===== 第4行：目标偏航角 ===== */
        OLED_ShowSignedNum(4, 5, (int)targetYaw, 4);
        int tgt_d = (int)((targetYaw - (int)targetYaw) * 10);
        if (tgt_d < 0) tgt_d = -tgt_d;
        OLED_ShowString(4, 10, ".");
        OLED_ShowNum(4, 11, tgt_d, 1);
    }
}
