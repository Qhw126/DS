#include "stm32f4xx.h"
#include "OLED_Font.h"

/*引脚定义*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

/*I2C必要延时（F4主频高，不加延时识别不到波形）*/
static void I2C_Delay(void)
{
	uint16_t i = 20;
	while(i--);
}

/*引脚初始化 纯模拟I2C 无硬件复用*/
void OLED_I2C_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   // 开漏必备
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    // 内部上拉必备
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/*I2C起始信号（带标准延时）*/
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	I2C_Delay();
	OLED_W_SCL(1);
	I2C_Delay();
	OLED_W_SDA(0);
	I2C_Delay();
	OLED_W_SCL(0);
	I2C_Delay();
}

/*I2C停止信号（带标准延时）*/
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	I2C_Delay();
	OLED_W_SCL(1);
	I2C_Delay();
	OLED_W_SDA(1);
	I2C_Delay();
}

/*I2C发送一字节（带延时 波形稳定）*/
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		I2C_Delay();
		OLED_W_SCL(1);
		I2C_Delay();
		OLED_W_SCL(0);
		I2C_Delay();
	}
	OLED_W_SCL(1);
	I2C_Delay();
	OLED_W_SCL(0);
}

/*OLED写命令*/
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);   // OLED常规I2C地址
	OLED_I2C_SendByte(0x00);   // 命令标识
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/*OLED写数据*/
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x40);   // 数据标识
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/*设置光标(页地址+列地址)*/
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}

/*清屏*/
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/*显示单个8*16字符*/
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
	}
}

/*显示字符串*/
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/*次方工具函数*/
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while(Y--)
	{
		Result *= X;
	}
	return Result;
}

/*显示十进制无符号数字*/
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*显示十进制带符号数字*/
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*显示十六进制数字*/
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/*显示二进制数字*/
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/*OLED整体初始化配置*/
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)   // 上电稳定延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();
	
	OLED_WriteCommand(0xAE);  // 关闭显示
	
	OLED_WriteCommand(0xD5);  // 分频+振荡
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);  // 多路复用
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);  // 显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);  // 起始行
	
	OLED_WriteCommand(0xA1);  // 左右正常
	OLED_WriteCommand(0xC8);  // 上下正常

	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);  // 对比度
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);  // 预充电
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);
	OLED_WriteCommand(0xA6);

	OLED_WriteCommand(0x8D);  // 电荷泵开启
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);  // 打开显示
		
	OLED_Clear();
}
