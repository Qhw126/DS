//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//////////////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h" 
#include <stdint.h>

#define PINA(x)		GPIO_ReadInputDataBit(GPIOA, 1 << (x))
#define PIND(x)		GPIO_ReadInputDataBit(GPIOD, 1 << (x))


//切换黑白检测
#define black 1
#define white 0


/*************************************
*函数名称：SENSOR_GPIO_Config
*函数功能：GPIO管脚的配置
*参数：
*说明：
*			
**************************************/
void SENSOR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//开启GPIOB的时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;      // 输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   // 无上下拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//开启GPIOB的时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}
/*************************************
*函数名称：digtal
*函数功能：获取X通道数字值
*参数：
*说明：
*			
**************************************/
uint8_t check = black;
unsigned char digtal(unsigned char channel) // 1~7 ??
{
    u8 value = 0;

    switch(channel)//检测黑，黑0 白1
    {
        case 1:  value = (PINA(8) == 1); break;
        case 2:  value = (PIND(0) == 1); break;
        case 3:  value = (PIND(1) == 1); break;
        case 4:  value = (PIND(2) == 1); break;
        case 5:  value = (PIND(3) == 1); break;
        case 6:  value = (PIND(5) == 1); break;
        case 7:  value = (PIND(6) == 1); break;
        default: value = 0; break;
    }

    if(check == white)//检测白，黑1 白0
    {
        value = !value;
    }

    return value;
}



