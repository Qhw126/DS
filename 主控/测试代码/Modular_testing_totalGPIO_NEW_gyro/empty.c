#include "ti_msp_dl_config.h"
#include <string.h>
#include "oled.h"
#include "gyro.h"
#include "Motor.h"
#include "Encoder.h"
#include "Encoder_GPIO.h"
#include "Speed.h"
#include "PID.h"
#include "PID_Control.h"
#include "sensor.h"
#include "line.h"
#include "bsp_hc05.h"

/* 简单延时 */
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

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

int main(void)
{
    SYSCFG_DL_init();

    /* 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"Yaw:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"BLE:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"Data:", FONT_SIZE);
    OLED_Refresh();

    Motor_Init();
    PID_Control_Init();
    Line_PID_Init();

    /* EncoderA = 硬件编码器 (TIMG8 QEI, PA26/PA27, 4倍频) */
    Encoder_Init();

    /* EncoderB = GPIO 中断编码器 (PA28/PA31, 2倍频) */
    Encoder_GPIO_Init();

    /* 初始化 HC-05 蓝牙模块（UART1, 115200, PB6/PB7） */
    Bluetooth_Init();

    /* 使能 UART 中断 */
    NVIC_ClearPendingIRQ(Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(Gyro_INST_INT_IRQN);

    /* 使能 PID 定时器中断（20ms） */
    NVIC_EnableIRQ(PID_TIM_INST_INT_IRQN);

    /* 使能寻路定时器中断（40ms） */
    NVIC_EnableIRQ(Pathfinding_TIM_INST_INT_IRQN);

    while (1)
    {
        /* 轮询陀螺仪 UART 数据 */
        if (DL_UART_isRXFIFOEmpty(Gyro_INST) == false)
        {
            uint8_t data = DL_UART_Main_receiveData(Gyro_INST);
            Gyro_ParseByte(data);
        }

        /* 轮询蓝牙 UART 数据（与陀螺仪相同的轮询方式） */
        if (DL_UART_isRXFIFOEmpty(HC_INST) == false)
        {
            uint8_t ch = DL_UART_Main_receiveData(HC_INST);
            if (BLERX_LEN < BLERX_LEN_MAX - 1)
            {
                BLERX_BUFF[BLERX_LEN++] = ch;
                BLERX_BUFF[BLERX_LEN] = '\0';
                BLERX_FLAG = 1;
            }
        }

        /*=== 第1行：显示 Yaw 偏航角 ===*/
        {
            float yaw = Gyro_GetYaw();
            int32_t yaw_i = (int32_t)yaw;

            OLED_ClearArea(4 * CHAR_W, LINE1_Y, 127, LINE2_Y);
            if (yaw_i < 0)
            {
                OLED_ShowString(4 * CHAR_W, LINE1_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(5 * CHAR_W, LINE1_Y, (u32)(-yaw_i), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(4 * CHAR_W, LINE1_Y, (u32)yaw_i, 4, FONT_SIZE);
            }
        }

        /*=== 第2行：显示蓝牙 STATE 引脚实际电平 ===*/
        {
            uint32_t state_pin = DL_GPIO_readPins(HC05_PORT, HC05_STATE_PIN);

            OLED_ClearArea(4 * CHAR_W, LINE2_Y, 127, LINE3_Y);
            OLED_ShowString(4 * CHAR_W, LINE2_Y, (u8 *)"PB23=", FONT_SIZE);
            if (state_pin != 0)
            {
                OLED_ShowNum(10 * CHAR_W, LINE2_Y, 1, 1, FONT_SIZE);  // 高电平
            }
            else
            {
                OLED_ShowNum(10 * CHAR_W, LINE2_Y, 0, 1, FONT_SIZE);  // 低电平
            }
        }

        /*=== 第3-4行：显示蓝牙接收到的数据 ===*/
        if (BLERX_FLAG == 1)
        {
            /* 保存本次接收的数据用于显示 */
            memset(last_ble_data, 0, sizeof(last_ble_data));
            strncpy(last_ble_data, (const char *)BLERX_BUFF, sizeof(last_ble_data) - 1);

            /* 清除数据行并显示新数据 */
            OLED_ClearArea(0, LINE3_Y, 127, 64);
            OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"Data:", FONT_SIZE);
            OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)last_ble_data, FONT_SIZE);

            Clear_BLERX_BUFF();
        }

        OLED_Refresh();
    }
}

/*============================================================================
 * PID 定时器中断服务函数（20ms 周期）
 *===========================================================================*/
void PID_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PID_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            PID_Control_Update();
            break;
        default:
            break;
    }
}

/*============================================================================
 * 寻路定时器中断服务函数（40ms 周期）
 *===========================================================================*/
void Pathfinding_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(Pathfinding_TIM_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            Line_PID_Update();
            break;
        default:
            break;
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
