#ifndef __PWM_H
#define __PWM_H

#define speed_to_dutycycle    	(2000.0f/((12.0f/7.4f)*400.0f))

void PWM_Init_1_2(void);
void PWM_Init_3_4(void);
void PWM_SetCompare1(float speed);//PA2
void PWM_SetCompare2(float speed);//PA3
void PWM_SetCompare3(float speed);//PB14
void PWM_SetCompare4(float speed);//PE15

#endif
