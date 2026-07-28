#include "sensor.h"

/* 切换黑白标志 */
uint8_t check = white;

/**
 * @brief  读取指定通道的传感器值
 * @param  channel 通道号 1~7
 * @retval 1=检测到黑线, 0=未检测到
 */
uint8_t digtal(uint8_t channel)
{
    uint8_t value = 0;

    switch(channel)
    {
        case 1: value = DL_GPIO_readPins(Pathfinding_L3_PORT, Pathfinding_L3_PIN) ? 1 : 0; break;
        case 2: value = DL_GPIO_readPins(Pathfinding_L2_PORT, Pathfinding_L2_PIN) ? 1 : 0; break;
        case 3: value = DL_GPIO_readPins(Pathfinding_L1_PORT, Pathfinding_L1_PIN) ? 1 : 0; break;
        case 4: value = DL_GPIO_readPins(Pathfinding_MC_PORT, Pathfinding_MC_PIN) ? 1 : 0; break;
        case 5: value = DL_GPIO_readPins(Pathfinding_R1_PORT, Pathfinding_R1_PIN) ? 1 : 0; break;
        case 6: value = DL_GPIO_readPins(Pathfinding_R2_PORT, Pathfinding_R2_PIN) ? 1 : 0; break;
        case 7: value = DL_GPIO_readPins(Pathfinding_R3_PORT, Pathfinding_R3_PIN) ? 1 : 0; break;
        default: value = 0; break;
    }

    /* 白底黑线模式：传感器低电平=检测到黑线，取反 */
    if(check == white)
    {
        value = !value;
    }

    return value;
}
