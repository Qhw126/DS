#include "stm32f4xx.h" // Device header
#include "PWM.h"


/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void PWM_Init_1_2(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);			
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 引脚复用映射 -> TIM9 */	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);	//通道1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9);	//通道2
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM9);
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;				
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;				
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseInitStructure);
	
	/*输出比较初始化*/
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	// 开启 4 个通道
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	
	/* 4 个通道都开启预装载 */
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
	/* TIM 自动重装载使能（必须加！F4 容易不出 PWM 的关键！）*/
	TIM_ARRPreloadConfig(TIM9, ENABLE);

	/*TIM使能*/
	TIM_Cmd(TIM9, ENABLE);
}

void PWM_Init_3_4(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);			
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 引脚复用映射 -> TIM9 */	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);	//通道1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);	//通道2
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM12);
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;				
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;				
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStructure);
	
	/*输出比较初始化*/
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	// 开启 4 个通道
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);
	
	/* 4 个通道都开启预装载 */
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
	/* TIM 自动重装载使能（必须加！F4 容易不出 PWM 的关键！）*/
	TIM_ARRPreloadConfig(TIM12, ENABLE);

	/*TIM使能*/
	TIM_Cmd(TIM12, ENABLE);
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~2000
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */

// 通道1 PD12
void PWM_SetCompare1(float speed)
{
	uint16_t Compare =  (uint16_t)speed_to_dutycycle * speed;
	TIM_SetCompare1(TIM9, Compare);
}

// 通道2 PD13
void PWM_SetCompare2(float speed)
{
	uint16_t Compare = (uint16_t)speed_to_dutycycle * speed;
	TIM_SetCompare2(TIM9, Compare);
}

// 通道3 PD14
void PWM_SetCompare3(float speed)
{
	uint16_t Compare = (uint16_t)speed_to_dutycycle * speed;
	TIM_SetCompare1(TIM12, Compare);
}

// 通道4 PD15
void PWM_SetCompare4(float speed)
{
	uint16_t Compare = (uint16_t)speed_to_dutycycle * speed;
	TIM_SetCompare2(TIM12, Compare);
}
