#ifndef __PID_H
#define __PID_H

typedef struct 
{
	float Target, Actual, Out;
	float Kp, Ki, Kd;
	float Error_0, Error_1, Error_2, Integral;
	float OutMax, OutMin, I_MAX, I_MIN;
	
}PID_Position;

typedef struct 
{
	float Target, Actual, Out;
	float Kp, Ki, Kd;
	float Error0, Error1, Error2;
	float Target_Iint;
	float OutMax,OutMin;
}PID_Incremental;

extern PID_Position PID_findway;

extern PID_Incremental PID_Motor1;
extern PID_Incremental PID_Motor2;
extern PID_Incremental PID_Motor3;
extern PID_Incremental PID_Motor4;


void PID_Iint(void);
void Incremental_PID(PID_Incremental *pid);
void Position_PID(PID_Position *pid);

#endif
