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

#define FONT_SIZE   12
#define CHAR_W      6
#define LINE1_Y     0
#define LINE2_Y     16
#define LINE3_Y     32
#define LINE4_Y     48

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    Motor_Init();
    Encoder_Init();
    Encoder_GPIO_Init();

    PID_Iint();
    GyroControl_Init();

    /* 使能陀螺仪 UART 中断 */
    NVIC_ClearPendingIRQ(Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(Gyro_INST_INT_IRQN);

    /* 记录上电时偏航角 */
    yawWhenLost = Gyro_GetYaw();

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

        /* ===== 第1行：模式 / Yaw / TargetYaw ===== */
        OLED_ClearArea(0, LINE1_Y, 127, LINE2_Y);
        OLED_ShowString(0 * CHAR_W,  LINE1_Y, (u8 *)"M:", FONT_SIZE);
        OLED_ShowNum(2 * CHAR_W,    LINE1_Y, controlMode, 1, FONT_SIZE);

        OLED_ShowString(4 * CHAR_W,  LINE1_Y, (u8 *)"Y:", FONT_SIZE);
        {
            int32_t yi = (int32_t)Gyro_GetYaw();
            if (yi < 0)
            {
                OLED_ShowString(6 * CHAR_W, LINE1_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(7 * CHAR_W, LINE1_Y, (u32)(-yi), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(6 * CHAR_W, LINE1_Y, (u32)yi, 3, FONT_SIZE);
            }
        }

        OLED_ShowString(11 * CHAR_W, LINE1_Y, (u8 *)"T:", FONT_SIZE);
        {
            int32_t ti = (int32_t)targetYaw;
            if (ti < 0)
            {
                OLED_ShowString(13 * CHAR_W, LINE1_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(14 * CHAR_W, LINE1_Y, (u32)(-ti), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(13 * CHAR_W, LINE1_Y, (u32)ti, 3, FONT_SIZE);
            }
        }

        /* ===== 第2行：传感器权重和 ===== */
        OLED_ClearArea(0, LINE2_Y, 127, LINE3_Y);
        {
            float sum_weight = 0;
            for (int ch = 1; ch <= 7; ch++)
            {
                if (digtal(ch) == 1) sum_weight += 1;
            }
            OLED_ShowString(0 * CHAR_W, LINE2_Y, (u8 *)"S:", FONT_SIZE);
            OLED_ShowNum(2 * CHAR_W, LINE2_Y, (uint32_t)sum_weight, 1, FONT_SIZE);
        }

        OLED_ShowString(5 * CHAR_W, LINE2_Y, (u8 *)"A:", FONT_SIZE);
        {
            int32_t ai = (int32_t)PID_findway.Actual;
            if (ai < 0)
            {
                OLED_ShowString(7 * CHAR_W, LINE2_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(8 * CHAR_W, LINE2_Y, (u32)(-ai), 2, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(7 * CHAR_W, LINE2_Y, (u32)ai, 2, FONT_SIZE);
            }
        }

        /* ===== 第3行：TurnPWM / 电机 Target ===== */
        OLED_ClearArea(0, LINE3_Y, 127, LINE4_Y);
        OLED_ShowString(0 * CHAR_W, LINE3_Y, (u8 *)"TP:", FONT_SIZE);
        {
            int32_t tp = (int32_t)GyroControl.TurnPWM;
            if (tp < 0)
            {
                OLED_ShowString(3 * CHAR_W, LINE3_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(4 * CHAR_W, LINE3_Y, (u32)(-tp), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(3 * CHAR_W, LINE3_Y, (u32)tp, 3, FONT_SIZE);
            }
        }

        OLED_ShowString(9 * CHAR_W, LINE3_Y, (u8 *)"L:", FONT_SIZE);
        {
            int32_t lt = (int32_t)PID_MotorA.Target;
            if (lt < 0)
            {
                OLED_ShowString(11 * CHAR_W, LINE3_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(12 * CHAR_W, LINE3_Y, (u32)(-lt), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(11 * CHAR_W, LINE3_Y, (u32)lt, 3, FONT_SIZE);
            }
        }

        OLED_ShowString(16 * CHAR_W, LINE3_Y, (u8 *)"R:", FONT_SIZE);
        {
            int32_t rt = (int32_t)PID_MotorB.Target;
            if (rt < 0)
            {
                OLED_ShowString(18 * CHAR_W, LINE3_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(19 * CHAR_W, LINE3_Y, (u32)(-rt), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(18 * CHAR_W, LINE3_Y, (u32)rt, 3, FONT_SIZE);
            }
        }

        /* ===== 第4行：PID 寻路输出 + 积分 ===== */
        OLED_ClearArea(0, LINE4_Y, 127, 64);
        OLED_ShowString(0 * CHAR_W, LINE4_Y, (u8 *)"O:", FONT_SIZE);
        {
            int32_t oi = (int32_t)PID_findway.Out;
            if (oi < 0)
            {
                OLED_ShowString(2 * CHAR_W, LINE4_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(3 * CHAR_W, LINE4_Y, (u32)(-oi), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(2 * CHAR_W, LINE4_Y, (u32)oi, 3, FONT_SIZE);
            }
        }

        OLED_ShowString(8 * CHAR_W, LINE4_Y, (u8 *)"I:", FONT_SIZE);
        {
            int32_t ii = (int32_t)PID_findway.Integral;
            if (ii < 0)
            {
                OLED_ShowString(10 * CHAR_W, LINE4_Y, (u8 *)"-", FONT_SIZE);
                OLED_ShowNum(11 * CHAR_W, LINE4_Y, (u32)(-ii), 3, FONT_SIZE);
            }
            else
            {
                OLED_ShowNum(10 * CHAR_W, LINE4_Y, (u32)ii, 3, FONT_SIZE);
            }
        }

        OLED_Refresh();
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
