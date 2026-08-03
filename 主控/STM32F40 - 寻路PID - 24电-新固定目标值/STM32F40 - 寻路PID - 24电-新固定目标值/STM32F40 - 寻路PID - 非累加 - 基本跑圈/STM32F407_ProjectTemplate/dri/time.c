#include "stm32f4xx.h"
#include "PID.h"
#include "Motor.h"
#include "line.h"
#include "Encoder.h"
#include "GyroUART.h"
#include "GyroControl.h"

#define Sampling_time       0.02f
#define Pulse_to_speed      (60.0f/ (4.0f*265.2f*Sampling_time))

int i = 0;

/* 定时器6计数器，每20ms加1 */
uint32_t timer6Count = 0;

/* 外部变量引用 */
extern uint8_t controlMode;
extern float targetYaw;
extern uint8_t isLineLost;

void time6_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /*选择时钟源*/
    TIM_InternalClockConfig(TIM6);

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

    /*清除标志位*/
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    /*使能中断*/
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    /*使能*/
    TIM_Cmd(TIM6, ENABLE);

    /*中断配置*/
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void time7_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_InternalClockConfig(TIM7);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 40000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM7, TIM_FLAG_Update);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM7, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIM7中断服务函数 - 控制逻辑(40ms周期)
  * @param  无
  * @retval 无
  */
void TIM7_IRQHandler(void)
{
     if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
     {
		if (i >= 50)
		{
			/* 根据控制模式执行不同逻辑 */
			if (controlMode == MODE_LINE_TRACK)
            {
                /* 循线模式 */
                track_zhixian1();
                Position_PID(&PID_findway);
                motor(PID_Motor3.Target_Iint - PID_findway.Out,
                      PID_Motor4.Target_Iint + PID_findway.Out);
            }

            else if (controlMode == MODE_GYRO_TURN)
            {
                /* 陀螺仪模式 - 双环PID计算TurnPWM */
                GyroControl_SetTarget(70.0f, targetYaw);
                GyroControl_Update();

                /* 与循线模式相同的写法，更清晰可读 */
                motor(GyroControl.BaseSpeed - GyroControl.TurnPWM,
                      GyroControl.BaseSpeed + GyroControl.TurnPWM);

                /* 检查是否又检测到线，如果是则切回循线 */
                if (checkLineFound())
                {
                    controlMode = MODE_LINE_TRACK;
                }
            }           
		}
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
	
}

/**
  * @brief  TIM6中断服务函数 - 电机PID控制(20ms周期)
  * @param  无
  * @retval 无
  */
void TIM6_DAC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
         if (i >= 50)
		{
			/* 计数器递增，每20ms加1 */
			if(gyroToLineFlag == 1)
            {
				timer6Count++;
				if(timer6Count >= 50)
				{
					timer6Count = 0;
					gyroToLineFlag = 0;
				}
			}

            /* 电机PID闭环 - 两种模式通用 */
            PID_Motor3.Actual = -1 * Encoder3_Get() * Pulse_to_speed;
            Incremental_PID(&PID_Motor3);
            Motor3_SetPWM(PID_Motor3.Out);

            PID_Motor4.Actual = 1 * Encoder4_Get() * Pulse_to_speed;
            Incremental_PID(&PID_Motor4);
            Motor4_SetPWM(PID_Motor4.Out);           
        }
		else
		{
			i++;
		}
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	
    }
    
}
