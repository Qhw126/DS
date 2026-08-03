#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

static uint32_t angleToPulses(float angle);
void angelup(float angle);
void controlByVision(float pixel_error);
void controlLostBall(void);

extern uint8_t Control_TxPacket[13];    // 当前发送的控制包，供OLED显示
extern float Control_DeltaAngle;        // PID 输出角度，供OLED显示
extern uint32_t Control_Pulses;         // 实际发送的脉冲数，供OLED显示

extern float Control_TargetVelocity;    // 位置环输出的期望速度(像素/帧)，供OLED调试
extern float Control_ActualVelocity;    // 滤波后的实际速度(像素/帧)，供OLED调试
extern uint8_t Control_BallLost;        // 球是否丢失(1=正在找回)，供OLED显示
extern float Control_PositionGain;      // 本帧按距离调度出的角度倍率，供OLED调试

#endif