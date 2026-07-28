#include "ti_msp_dl_config.h"
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

/* 简单延时 */
void delay_ms(uint32_t ms)
{
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

/* 两个编码器各自的速度数据（用于显示） */
Speed_Data speedA = {0, 0.0f};
Speed_Data speedB = {0, 0.0f};

/* 传感器显示模式：0=显示编码器, 1=显示传感器 */
static uint8_t sensor_show_mode = 0;

/* 陀螺仪中断调试 */
volatile uint32_t gyro_irq_count = 0;

/* 轮询调试计数 */
volatile uint32_t poll_count = 0;

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

int main(void)
{
    SYSCFG_DL_init();

    /* 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"Target:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"A RPM:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"B RPM:", FONT_SIZE);
    OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"I:   P:   Y:", FONT_SIZE);
    OLED_Refresh();

    Motor_Init();
    PID_Control_Init();
    Line_PID_Init();

    /* EncoderA = 硬件编码器 (TIMG8 QEI, PA26/PA27, 4倍频) */
    Encoder_Init();

    /* EncoderB = GPIO 中断编码器 (PA28/PA31, 2倍频) */
    Encoder_GPIO_Init();

    /* 使能 UART 中断，设置最高优先级 */
    NVIC_ClearPendingIRQ(Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(Gyro_INST_INT_IRQN);
    
    /* 使能 PID 定时器中断（20ms）— 电机速度闭环 */
    NVIC_EnableIRQ(PID_TIM_INST_INT_IRQN);

    /* 使能寻路定时器中断（40ms）— 寻路差速转向 */
    NVIC_EnableIRQ(Pathfinding_TIM_INST_INT_IRQN);

    while (1)
    {
        /* 轮询测试：如果 UART RX FIFO 不为空，直接读取 */
        if (DL_UART_isRXFIFOEmpty(Gyro_INST) == false)
        {
            uint8_t data = DL_UART_Main_receiveData(Gyro_INST);
            Gyro_ParseByte(data);
            poll_count++;
        }

        /* 读取当前传感器状态 */
        uint8_t sensor_now = (L3 << 6) | (L2 << 5) | (L1 << 4) | (MC << 3)
                           | (R1 << 2) | (R2 << 1) | R3;

        /* 有传感器触发（非全白）→ 持续显示传感器 */
        if(sensor_now != 0)
        {
            if(sensor_show_mode == 0)
            {
                sensor_show_mode = 1;
                OLED_Clear();
                OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"L3 L2 L1 MC", FONT_SIZE);
                OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"R1 R2 R3", FONT_SIZE);
            }

            /* 持续刷新传感器电平（标签在1/3行，数值在2/4行） */
            OLED_ClearArea(0, LINE2_Y, 127, LINE3_Y);
            OLED_ShowNum(0 * CHAR_W,  LINE2_Y, L3, 1, FONT_SIZE);
            OLED_ShowNum(4 * CHAR_W,  LINE2_Y, L2, 1, FONT_SIZE);
            OLED_ShowNum(8 * CHAR_W,  LINE2_Y, L1, 1, FONT_SIZE);
            OLED_ShowNum(12 * CHAR_W, LINE2_Y, MC, 1, FONT_SIZE);

            OLED_ClearArea(0, LINE4_Y, 127, 64);
            OLED_ShowNum(0 * CHAR_W,  LINE4_Y, R1, 1, FONT_SIZE);
            OLED_ShowNum(4 * CHAR_W,  LINE4_Y, R2, 1, FONT_SIZE);
            OLED_ShowNum(8 * CHAR_W,  LINE4_Y, R3, 1, FONT_SIZE);

            OLED_Refresh();
        }
        /* 全白 → 恢复编码器显示 */
        else
        {
            if(sensor_show_mode == 1)
            {
                sensor_show_mode = 0;
                OLED_Clear();
                OLED_ShowString(0 * CHAR_W, LINE1_Y, (u8 *)"Target:", FONT_SIZE);
                OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"A RPM:", FONT_SIZE);
                OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"B RPM:", FONT_SIZE);
                OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"I:   P:   Y:", FONT_SIZE);
            }

            /* 持续刷新编码器数据 */
            speedA.rpm = PID_MotorA.Actual;
            speedB.rpm = PID_MotorB.Actual;

            /* 清除数值区域并重绘 */
            OLED_ClearArea(8 * CHAR_W, LINE1_Y, 127, LINE2_Y);
            OLED_ShowNum(8 * CHAR_W, LINE1_Y, (u32)PID_MotorA.Target, 5, FONT_SIZE);

            OLED_ClearArea(8 * CHAR_W, LINE2_Y, 127, LINE3_Y);
            OLED_ShowNum(8 * CHAR_W, LINE2_Y, (u32)speedA.rpm, 5, FONT_SIZE);

            OLED_ClearArea(8 * CHAR_W, LINE3_Y, 127, LINE4_Y);
            OLED_ShowNum(8 * CHAR_W, LINE3_Y, (u32)speedB.rpm, 5, FONT_SIZE);

            /* 显示中断计数、轮询计数、偏航角 */
            OLED_ClearArea(0, LINE4_Y, 127, 64);
            OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"I:", FONT_SIZE);
            OLED_ShowNum(2 * CHAR_W, LINE4_Y, gyro_irq_count, 3, FONT_SIZE);
            OLED_ShowString(6 * CHAR_W, LINE4_Y, (u8 *)"P:", FONT_SIZE);
            OLED_ShowNum(8 * CHAR_W, LINE4_Y, poll_count, 3, FONT_SIZE);
            {
                float yaw = Gyro_GetYaw();
                int32_t yaw_i = (int32_t)yaw;
                OLED_ShowString(12 * CHAR_W, LINE4_Y, (u8 *)"Y:", FONT_SIZE);
                if (yaw_i < 0)
                {
                    OLED_ShowString(14 * CHAR_W, LINE4_Y, (u8 *)"-", FONT_SIZE);
                    OLED_ShowNum(15 * CHAR_W, LINE4_Y, (u32)(-yaw_i), 2, FONT_SIZE);
                }
                else
                {
                    OLED_ShowNum(14 * CHAR_W, LINE4_Y, (u32)yaw_i, 3, FONT_SIZE);
                }
            }

            OLED_Refresh();
        }
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
 * 陀螺仪串口中断服务函数 - 直接用函数名，不用宏
 *===========================================================================*/
void Gyro_INST_IRQHandler(void)
{
    gyro_irq_count++;  // 调试计数

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
