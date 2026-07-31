#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"

/*
 * X42S Emm固件测试
 * PA2 (USART2_TX) -> X42S TTL RX
 * GND             -> X42S TTL GND
 * 电机：地址1，115200，固定0x6B校验，1.8度，256细分
 */

#define MOTOR_CW   0x00U  /* 正转/顺时针 */
#define MOTOR_CCW  0x01U  /* 反转/逆时针 */

static void Motor_Enable(void)
{
    uint8_t command[] = {
        0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B
    };

    Serial_SendArray(command, sizeof(command));
}

/* 回到X42S屏幕菜单中保存的单圈零点。 */
static void Motor_GoZero(void)
{
    uint8_t command[] = {
        0x01,       /* 电机地址 */
        0x9A,       /* 触发回零 */
        0x00,       /* 单圈就近回零 */
        0x00,       /* 立即执行 */
        0x6B        /* 固定校验 */
    };

    Serial_SendArray(command, sizeof(command));
}

static void Motor_MoveRelative(uint8_t direction, float degrees)
{
    uint8_t command[13];
    uint32_t pulses;

    /* 1.8度电机、256细分：200 * 256 = 51200脉冲/圈。 */
    pulses = (uint32_t)(degrees * 51200.0f / 360.0f + 0.5f);

    command[0]  = 0x01;                    /* 电机地址 */
    command[1]  = 0xFD;                    /* Emm位置模式 */
    command[2]  = direction;               /* 00正转，01反转 */
    command[3]  = 0x00;
    command[4]  = 0x1E;                    /* 速度30 RPM */
    command[5]  = 100;                     /* 加速度档位 */
    command[6]  = (uint8_t)(pulses >> 24);
    command[7]  = (uint8_t)(pulses >> 16);
    command[8]  = (uint8_t)(pulses >> 8);
    command[9]  = (uint8_t)pulses;
    command[10] = 0x02;                    /* 相对当前位置运动 */
    command[11] = 0x00;                    /* 立即执行 */
    command[12] = 0x6B;                    /* 固定校验 */

    Serial_SendArray(command, sizeof(command));
}

int main(void)
{
    Serial_Init();
    Delay_ms(500);

    Motor_Enable();
    Delay_ms(500);

    Motor_GoZero();                          /* 回到硬件保存的0点 */
    Delay_ms(3000);
	
    Motor_MoveRelative(MOTOR_CW, 8.0f);      /* 正转4度 */
    Delay_ms(600);                        
	
    Motor_MoveRelative(MOTOR_CCW, 10.0f);     /* 反转4度 */
    Delay_ms(1450);                         
	
	Motor_MoveRelative(MOTOR_CW, 9.0f);      /* 正转4度 */
    Delay_ms(500); 
	
    Motor_GoZero();                         /* 再回到硬件保存的0点 */
    Delay_ms(600);

    while (1)
    {
        /* 动作只执行一次，最后保持在硬件0点。 */
    }
}
