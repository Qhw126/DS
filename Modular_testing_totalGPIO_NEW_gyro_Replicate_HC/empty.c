#include "ti_msp_dl_config.h"
#include <string.h>
#include "oled.h"
#include "gyro.h"
#include "Motor.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "Speed.h"
#include "PID.h"
#include "GyroControl.h"
#include "sensor.h"
#include "line.h"
#include "bsp_hc05.h"
#include "time.h"

/*
 * OLED坐标映射（12号字体，每字符6x12像素）
 * 行1: y=0,  行2: y=16, 行3: y=32, 行4: y=48
 * 列: x = (列号-1) * 6
 */
#define FONT_SIZE   12
#define CHAR_W      6
#define LINE1_Y     0
#define LINE2_Y     16
#define LINE3_Y     32
#define LINE4_Y     48

/* 保存上次显示的蓝牙数据，避免重复刷新 */
static char last_ble_data[BLERX_LEN_MAX] = {0};

/* 帧协议解析状态 */
static uint8_t frame_started = 0;

/* 发送计数 */
static uint32_t tx_count = 0;

/* 确认标志：收到OK后停止发送 */
static uint8_t confirmed = 0;

/* 发送带帧协议的数据：<数据> */
static void BLE_Send_Frame(const char *data)
{
    BLE_send_String((unsigned char *)"<");
    BLE_send_String((unsigned char *)data);
    BLE_send_String((unsigned char *)">");
    tx_count++;
}

int main(void)
{
    SYSCFG_DL_init();

    /* 初始化外设（对齐参考工程 main.c 初始化序列） */
    OLED_Init();
    OLED_Clear();
    Motor_Init();
    Encoder_Init();
    Encoder_GPIO_Init();
    Bluetooth_Init();

    /* PID 总初始化（对齐参考工程 PID_Iint） */
    PID_Iint();

    /* 初始化陀螺仪控制模块 */
    GyroControl_Init();

    /* 初始 OLED 显示标签 */
    OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"M:  L:  F:  ML:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"Ct:  S:  A:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"Yaw:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"TX:", FONT_SIZE);
    OLED_ShowString(6 * CHAR_W, LINE4_Y, (u8 *)"RX:", FONT_SIZE);
    OLED_Refresh();

    /* 使能陀螺仪 UART 中断 */
    NVIC_ClearPendingIRQ(Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(Gyro_INST_INT_IRQN);

    /* 使能 PID 定时器中断（20ms） */
    NVIC_EnableIRQ(PID_TIM_INST_INT_IRQN);

    /* 使能寻路定时器中断（40ms） */
    NVIC_EnableIRQ(Pathfinding_TIM_INST_INT_IRQN);

    /* 记录上电时偏航角（对齐参考工程 yawWhenLost） */
    yawWhenLost = Gyro_GetYaw();

    while (1)
    {
        /* 轮询陀螺仪 UART 数据 */
        if (DL_UART_isRXFIFOEmpty(Gyro_INST) == false)
        {
            uint8_t data = DL_UART_Main_receiveData(Gyro_INST);
            Gyro_ParseByte(data);
        }

        /* ===== 计算传感器权重和 ===== */
        float sum_weight = 0;
        for (int ch = 1; ch <= 7; ch++)
        {
            if (digtal(ch) == 1)
                sum_weight += 1;
        }

        /* ===== 第1行：状态标志位（对齐参考工程 main.c） ===== */
        OLED_ClearArea(0, LINE1_Y, 127, LINE2_Y);
        OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"M:", FONT_SIZE);
        OLED_ShowNum(2 * CHAR_W, LINE1_Y, controlMode, 1, FONT_SIZE);
        OLED_ShowString(4 * CHAR_W, LINE1_Y, (u8 *)"L:", FONT_SIZE);
        OLED_ShowNum(6 * CHAR_W, LINE1_Y, isLineLost, 2, FONT_SIZE);
        OLED_ShowString(9 * CHAR_W, LINE1_Y, (u8 *)"F:", FONT_SIZE);
        OLED_ShowNum(11 * CHAR_W, LINE1_Y, gyroToLineFlag, 1, FONT_SIZE);
        OLED_ShowString(13 * CHAR_W, LINE1_Y, (u8 *)"ML:", FONT_SIZE);
        OLED_ShowNum(16 * CHAR_W, LINE1_Y, controlModelast, 1, FONT_SIZE);

        /* ===== 第2行：计数器/传感器/PID ===== */
        OLED_ClearArea(0, LINE2_Y, 127, LINE3_Y);
        OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"Ct:", FONT_SIZE);
        OLED_ShowNum(3 * CHAR_W, LINE2_Y, timer6Count, 2, FONT_SIZE);
        OLED_ShowString(6 * CHAR_W, LINE2_Y, (u8 *)"S:", FONT_SIZE);
        OLED_ShowNum(8 * CHAR_W, LINE2_Y, (uint32_t)sum_weight, 1, FONT_SIZE);
        OLED_ShowString(10 * CHAR_W, LINE2_Y, (u8 *)"A:", FONT_SIZE);
        {
            int32_t actual_i = (int32_t)PID_findway.Actual;
            if (actual_i < 0)
            {
                OLED_ShowString(12 * CHAR_W, LINE2_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(13 * CHAR_W, LINE2_Y, (u32)(-actual_i), 2, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(12 * CHAR_W, LINE2_Y, (u32)actual_i, 3, FONT_SIZE);
            }
        }

        /* ===== 第3行：当前偏航角 ===== */
        {
            float yaw = Gyro_GetYaw();
            int32_t yaw_i = (int32_t)yaw;

            OLED_ClearArea(0, LINE3_Y, 127, LINE4_Y);
            OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"Yaw:", FONT_SIZE);
            if (yaw_i < 0)
            {
                OLED_ShowString(4 * CHAR_W, LINE3_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(5 * CHAR_W, LINE3_Y, (u32)(-yaw_i), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(4 * CHAR_W, LINE3_Y, (u32)yaw_i, 4, FONT_SIZE);
            }
        }

        /* ===== 第4行：发送计数 + 接收数据 ===== */
        OLED_ClearArea(0, LINE4_Y, 127, 64);
        OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"TX:", FONT_SIZE);
        OLED_ShowNum(3 * CHAR_W, LINE4_Y, tx_count, 5, FONT_SIZE);
        OLED_ShowString(9 * CHAR_W, LINE4_Y, (u8 *)"RX:", FONT_SIZE);

        /* 处理收到的帧 */
        if (BLERX_FLAG == 1)
        {
            memset(last_ble_data, 0, sizeof(last_ble_data));
            strncpy(last_ble_data, (const char *)BLERX_BUFF, sizeof(last_ble_data) - 1);

            /* 收到 OK 则停止发送 */
            if (strcmp(last_ble_data, "OK") == 0)
                confirmed = 1;

            Clear_BLERX_BUFF();
            frame_started = 0;
        }

        /* 每次刷新都显示最新接收数据（缓存在 last_ble_data 中） */
        OLED_ShowString(12 * CHAR_W, LINE4_Y, (u8 *)last_ble_data, FONT_SIZE);

        OLED_Refresh();

        /* 未确认则持续发送，收到OK后停止 */
        if (!confirmed)
            BLE_Send_Frame("Hello");
    }
}

/*============================================================================
 * 陀螺仪串口中断服务函数
 *===========================================================================*/
void Gyro_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(Gyro_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t data = DL_UART_Main_receiveData(Gyro_INST);
            Gyro_ParseByte(data);
            break;
        }
        default:
            break;
    }
}

/*============================================================================
 * 蓝牙串口中断服务函数（UART1）— 帧协议解析
 * 起始符 '<' → 开始缓存
 * 结束符 '>' → 一帧完成，设 BLERX_FLAG
 *===========================================================================*/
void HC_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(HC_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t ch = DL_UART_Main_receiveData(HC_INST);

            if (ch == '<')
            {
                BLERX_LEN = 0;
                frame_started = 1;
            }
            else if (ch == '>' && frame_started)
            {
                BLERX_BUFF[BLERX_LEN] = '\0';
                BLERX_FLAG = 1;
                frame_started = 0;
            }
            else if (frame_started && BLERX_LEN < BLERX_LEN_MAX - 1)
            {
                BLERX_BUFF[BLERX_LEN++] = ch;
            }
            break;
        }
        default:
            break;
    }
}
