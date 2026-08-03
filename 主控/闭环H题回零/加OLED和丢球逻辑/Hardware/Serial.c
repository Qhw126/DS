#include "Serial.h"
#include "stm32f10x.h"

static volatile VisionSample g_vision;
static uint8_t g_packet[8];
static uint8_t g_index;

void Serial_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA9=USART1_TX，PA2=USART2_TX。 */
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10=USART1_RX，PA3=USART2_RX。 */
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Init(USART2, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);

    g_index = 0;
    g_vision.x = 0;
    g_vision.y = 0;
    g_vision.valid = 0;
    g_vision.sequence = 0;
}

void MotorSerial_SendArray(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0; index < length; ++index)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART2, data[index]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
    }
}

void Vision_GetLatest(VisionSample *sample)
{
    __disable_irq();
    sample->x = g_vision.x;
    sample->y = g_vision.y;
    sample->valid = g_vision.valid;
    sample->sequence = g_vision.sequence;
    __enable_irq();
}

void USART1_IRQHandler(void)
{
    uint8_t byte;
    uint8_t checksum;
    uint8_t i;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        byte = (uint8_t)USART_ReceiveData(USART1);

        if (g_index == 0)
        {
            if (byte == 0xAA)
            {
                g_packet[g_index++] = byte;
            }
        }
        else if (g_index == 1)
        {
            if (byte == 0x55)
            {
                g_packet[g_index++] = byte;
            }
            else if (byte == 0xAA)
            {
                g_packet[0] = byte;
            }
            else
            {
                g_index = 0;
            }
        }
        else
        {
            g_packet[g_index++] = byte;
            if (g_index >= 8)
            {
                checksum = 0;
                for (i = 0; i < 7; ++i)
                {
                    checksum = (uint8_t)(checksum + g_packet[i]);
                }

                if (checksum == g_packet[7])
                {
                    g_vision.valid = (g_packet[2] == 1U) ? 1U : 0U;
                    g_vision.x = (int16_t)((uint16_t)g_packet[3] |
                                  ((uint16_t)g_packet[4] << 8));
                    g_vision.y = (uint16_t)((uint16_t)g_packet[5] |
                                  ((uint16_t)g_packet[6] << 8));
                    ++g_vision.sequence;
                }
                g_index = 0;
            }
        }
    }

    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
    }
}
