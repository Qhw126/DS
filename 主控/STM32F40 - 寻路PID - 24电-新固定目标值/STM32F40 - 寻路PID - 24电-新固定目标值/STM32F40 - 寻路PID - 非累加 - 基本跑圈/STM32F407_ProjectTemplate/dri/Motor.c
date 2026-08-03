#include "stm32f4xx.h"  // Device header
#include <stdint.h>
#include "PWM.h"
#include "PID.h"

//B01 E07 E08 E10 E12 E13 E14 E15  


#define AIN1(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_1, (BitAction)(x))
#define AIN2(x)     GPIO_WriteBit(GPIOE, GPIO_Pin_7, (BitAction)(x))
#define BIN1(x)		GPIO_WriteBit(GPIOE, GPIO_Pin_8, (BitAction)(x))
#define BIN2(x)		GPIO_WriteBit(GPIOE, GPIO_Pin_10, (BitAction)(x))
#define CIN1(x)		GPIO_WriteBit(GPIOE, GPIO_Pin_13, (BitAction)(x))
#define CIN2(x)     GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction)(x))
#define DIN1(x)		GPIO_WriteBit(GPIOE, GPIO_Pin_15, (BitAction)(x))
#define DIN2(x)		GPIO_WriteBit(GPIOE, GPIO_Pin_14, (BitAction)(x))


/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

/**
  * 函    数：直流电机设置PWM
  * 参    数：PWM 要设置的PWM值，范围：-100~100（负数为反转）
  * 返 回 值：无
  */
void Motor1_SetPWM(float PWM)
{
	if (PWM >= 0)							//如果设置正转的PWM
	{
		AIN1(1);	//置高电平
		AIN2(0);	//置低电平
		PWM_SetCompare1(PWM);				//设置PWM占空比
	}
	else									//否则，即设置反转的速度值
	{
		AIN1(0);	//置低电平
		AIN2(1);	//置高电平
		PWM_SetCompare1(-PWM);				//设置PWM占空比
	}
}
void Motor2_SetPWM(float PWM)
{
	if (PWM >= 0)
	{
		BIN1(1);	//置高电平
		BIN2(0);	//置低电平
		PWM_SetCompare2(PWM);   // 用通道2，和电机1区分开
	}
	else
	{
		BIN1(0);	//置低电平
		BIN2(1);	//置高电平
		PWM_SetCompare2(-PWM);
	}
}
void Motor3_SetPWM(float PWM)
{
	if (PWM >= 0)
	{
		CIN1(1);	//置高电平
		CIN2(0);	//置低电平
		PWM_SetCompare3(PWM);   // 用通道2，和电机1区分开
	}
	else
	{
		CIN1(0);	//置低电平
		CIN2(1);	//置高电平
		PWM_SetCompare3(-PWM);
	}
}
void Motor4_SetPWM(float PWM)
{
	if (PWM >= 0)
	{
		DIN1(1);	//置高电平
		DIN2(0);	//置低电平
		PWM_SetCompare4(PWM);   // 用通道2，和电机1区分开
	}
	else
	{
		DIN1(0);	//置低电平
		DIN2(1);	//置高电平
		PWM_SetCompare4(-PWM);
	}
}
void motor(int Motor_1, int Motor_2)
{
	// 电机1
	if(Motor_1 > 200)       Motor_1 = 200;
	else if(Motor_1 < -30) Motor_1 = -30;
	PID_Motor3.Target = Motor_1;
	
	if (Motor_1 > 0)	
	{
//		PWM_SetCompare3(Motor_1);
		AIN1(1); AIN2(0);
	}
	else if (Motor_1 < 0)	
	{
//		PWM_SetCompare3(-Motor_1);
		AIN1(0); AIN2(1);
	}
	else
	{
//		PWM_SetCompare3(0);
		AIN1(0); AIN2(0);
	}

	// 电机2
	if(Motor_2 > 200)       Motor_2 = 200;
	else if(Motor_2 < -30) Motor_2 = -30;
	PID_Motor4.Target = Motor_2;
	if (Motor_2 > 0)	
	{
//		PWM_SetCompare4(Motor_2);
		BIN1(1); BIN2(0);
	}
	else if (Motor_2 < 0)	
	{
//		PWM_SetCompare4(-Motor_2);
		BIN1(0); BIN2(1);
	}
	else
	{
//		PWM_SetCompare4(0);
		BIN1(0); BIN2(0);
	}
}

