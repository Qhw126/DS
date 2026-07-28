#ifndef __SENSOR_H
#define __SENSOR_H

#include "ti_msp_dl_config.h"
#include "PID.h"

/* 黑白模式 */
#define black 1
#define white 0

/* 函数声明 */
uint8_t digtal(uint8_t channel);
void    Sensor_ShowLevels(PID_Position *pid);

/* 便捷宏 — 使用 SysConfig 命名 */
#define L3 digtal(1)
#define L2 digtal(2)
#define L1 digtal(3)
#define MC digtal(4)
#define R1 digtal(5)
#define R2 digtal(6)
#define R3 digtal(7)

#endif
