#include "stm32f4xx.h"
#include <stdlib.h>
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "line.h"
#include "OLED.h"
#include <math.h>

#define PID_period		40

PID_Position PID_findway;

PID_Incremental PID_Motor1;
PID_Incremental PID_Motor2;
PID_Incremental PID_Motor3;
PID_Incremental PID_Motor4;

void PID_Iint(void)
{
	PID_Motor3.Kp = 1.32;	//修改Kp，调整范围：0~2
	PID_Motor3.Ki = 0.112;			//修改Ki，调整范围：0~2
	PID_Motor3.Kd = 0;	//修改Kd，调整范围：0~2
	PID_Motor3.Target_Iint =70;	//修改目标值，调整范围：-150~150
	PID_Motor3.Target = PID_Motor3.Target_Iint;
	PID_Motor3.OutMax = 400;
	PID_Motor3.OutMin = -400;
		
	PID_Motor4.Kp = 1.313;	//修改Kp，调整范围：0~2
	PID_Motor4.Ki = 0.11;			//修改Ki，调整范围：0~2
	PID_Motor4.Kd = 0.001;	//修改Kd，调整范围：0~2
	PID_Motor4.Target_Iint =70;	//修改目标值，调整范围：-150~150
	PID_Motor4.Target = PID_Motor4.Target_Iint;
	PID_Motor4.OutMax = 400;
	PID_Motor4.OutMin = -400;
	
	PID_findway.Kp = 8;
	PID_findway.Ki = 0.03;
	PID_findway.Kd = 1.5;
	PID_findway.Target = 0;
	PID_findway.Out = 0;
	PID_findway.OutMax = 100;
	PID_findway.OutMin = -100;
	PID_findway.I_MAX = 200;
	PID_findway.I_MIN = -200;
}

void Position_PID(PID_Position *pid)
{
	// 1.计算当前误差
	pid->Error_0 = pid->Target - pid->Actual;
	
	// 2.累加误差
	pid->Integral += pid->Error_0;
	if(pid->Error_0 < 3.0f && pid->Error_0 > -3.0f)  pid->Integral += pid->Error_0;// 偏差小于3时（接近中线）才积分
	else	pid->Integral = 0;       // 大偏差时清零积分
	//积分限幅
	if (pid->Integral > pid->I_MAX) {pid->Integral = pid->I_MAX;}
	if (pid->Integral < pid->I_MIN) {pid->Integral = pid->I_MIN;}
	
	// 3.计算输出
	pid->Out  =  pid->Kp * pid->Error_0							// P
	          +  pid->Ki * pid->Integral						// I
	          +  pid->Kd * (pid->Error_0 - pid->Error_1);		// D
	
	// 4.替换误差
	pid->Error_1 = pid->Error_0;
	
	// 5. 输出限幅
	if (pid->Out > pid->OutMax) {pid->Out = pid->OutMax;}
	if (pid->Out < pid->OutMin) {pid->Out = pid->OutMin;}
}

void Incremental_PID(PID_Incremental *pid)
{	
    // 1. 计算误差
    pid->Error2 = pid->Error1;  // e(k-2) = e(k-1)
    pid->Error1 = pid->Error0;  // e(k-1) = e(k)
    pid->Error0 = pid->Target - pid->Actual;
    
    // 2. 计算PID
    // au(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
    float DeltaOut = pid->Kp * (pid->Error0 - pid->Error1) 
                     + pid->Ki * pid->Error0 
                     + pid->Kd * (pid->Error0 - 2 * pid->Error1 + pid->Error2);
    
    // 3. 累加输出 u(k) = u(k-1) + au(k)
    pid->Out += DeltaOut;
	
	// 4. 输出限幅
	if (pid->Out > pid->OutMax) {pid->Out = pid->OutMax;}
	if (pid->Out < pid->OutMin) {pid->Out = pid->OutMin;}
	
}
