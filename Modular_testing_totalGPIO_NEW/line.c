#include "line.h"
#include "sensor.h"
#include "PID_Control.h"

/* 寻路 PID 实例 */
PID_Position PID_findway;

/* 基础速度（RPM），可通过 BaseSpeed_Set() 运行时修改 */
float base_speed = 70.0f;

/**
 * @brief  寻路位置计算（加权平均法）
 *
 * 7路传感器加权位置：
 *   D1(L3)=+9  D2(L2)=+3  D3(L1)=+1.25  D4(MC)=0
 *   D5(R1)=-1.25  D6(R2)=-3  D7(R3)=-9
 *
 * 1-2个传感器触发：加权平均算偏移
 * 3+个传感器触发：路口，强制急转
 * 0个传感器触发：丢线，用上次方向继续
 */
void track_zhixian1(void)
{
    static const float pos[7] = {9, 3, 1.25, 0, -1.25, -3, -9};
    float sum_weight = 0;
    float sum_pos = 0;

    for(int i = 0; i < 7; i++)
    {
        if(digtal(i + 1) == 1)
        {
            sum_weight += 1;
            sum_pos += pos[i];
        }
    }

    if(sum_weight > 0 && sum_weight < 3)
    {
        /* 1-2个传感器触发：加权平均 */
        PID_findway.Actual = sum_pos / sum_weight;
    }
    else if(sum_weight >= 3)
    {
        /* 3+个传感器触发：路口，强制急转 */
        if(sum_pos > 0)
            PID_findway.Actual = 10;
        else
            PID_findway.Actual = -10;
    }
    else
    {
        /* 0个传感器触发：丢线，用上次方向 */
        if(PID_findway.Actual >= 8)
            PID_findway.Actual = 10;
        else if(PID_findway.Actual <= -8)
            PID_findway.Actual = -10;
        else
            PID_findway.Actual = 0;
    }
}

/**
 * @brief  寻路 PID 初始化（按源文件参数）
 *
 * 源文件 time.c 中 TIM7 中断逻辑：
 *   track_zhixian1();
 *   Position_PID(&PID_findway);
 *   motor(base_speed - PID_findway.Out, base_speed + PID_findway.Out);
 */
void Line_PID_Init(void)
{
    PID_findway.Kp = 8.0f;
    PID_findway.Ki = 0.03f;
    PID_findway.Kd = 1.5f;
    PID_findway.Target = 0.0f;         /* 目标：在线中间（偏移=0） */
    PID_findway.OutMax = 100.0f;       /* 输出上限 */
    PID_findway.OutMin = -100.0f;      /* 输出下限 */
    PID_findway.I_MAX = 200.0f;        /* 积分上限 */
    PID_findway.I_MIN = -200.0f;       /* 积分下限 */
    PID_findway.Actual = 0.0f;
    PID_findway.Out = 0.0f;
    PID_findway.Integral = 0.0f;
    PID_findway.Error_0 = 0.0f;
    PID_findway.Error_1 = 0.0f;
}

/**
 * @brief  寻路 PID 更新（在定时器中断里调用，40ms 周期）
 *
 * 逻辑：读传感器 → 算位置 → 位置式PID → 差速修正电机目标
 */
void Line_PID_Update(void)
{
    /* 启动延时：等待 PID_Control 的 1 秒延时结束后再启用寻路 */
    if (pid_enabled == 0) return;

    /* 1. 读传感器，计算偏移量 */
    track_zhixian1();

    /* 2. 位置式 PID 计算 */
    Position_PID(&PID_findway);

    /* 3. 差速转向：基础速度 ± 修正量 */
    float targetA = base_speed - PID_findway.Out;
    float targetB = base_speed + PID_findway.Out;

    /* 4. 更新电机目标速度 */
    PID_Control_SetTarget(targetA, targetB);
}

/**
 * @brief  设置寻路基础速度
 * @param  speed 基础速度（RPM），寻路差速从此值加减
 */
void BaseSpeed_Set(float speed)
{
    base_speed = speed;
}
