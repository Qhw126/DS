#ifndef __TIME_H
#define __TIME_H

#include <stdint.h>

/* 定时器中断服务函数声明 */
void PID_TIM_INST_IRQHandler(void);
void Pathfinding_TIM_INST_IRQHandler(void);

/* 启动延时后的 PID 使能标志 */
extern volatile uint8_t pid_enabled;

/* gyroToLine 保护计数器 */
extern uint32_t timer6Count;

/* 延时函数 */
void delay_ms(uint32_t ms);

#endif /* __TIME_H */
