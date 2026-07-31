#include "stm32f10x.h"                  // 包含 STM32 的基础头文件
#include "Serial.h"                    // 包含串口发送函数的声明
#include <stdint.h>                    // 包含标准整型类型定义

/* ==================== 串级PID参数（位置环 + 速度环） ====================
 * 控制流程：K230像素误差 →[位置环]→ 期望速度 →[速度环]→ 电机角度 → 步进电机
 *   位置环(外环)：球离中心有多远 → 希望球以多快的速度朝中心走
 *   速度环(内环)：实际速度与期望速度之差 → 电机该给多大角度
 * 串级的好处：球靠近中心时"期望速度"自动变小、球不会冲过头，
 *            速度环提供阻尼，让球能平稳停在杆中间（这是单环PID做不到的）。
 * 调参顺序建议：先调 KP_POS 让球能追着中心动，再调 KP_VEL 让它不震荡/不过冲，
 * 最后调 VEL_FILTER 和距离调节那组参数。下面每个参数都附了调参说明。
 */

/* ---------- 位置环（外环）参数 ---------- */
#define KP_POS        0.02f    // 位置环比例系数：误差1px对应的期望速度(像素/帧)
                               // 决定整体响应快慢：太小球追不上中心、到位慢；太大容易震荡。
                               // 先把 KP_POS 加大到球能快速朝中心走，再回调到不抖为止。
#define KI_POS        0.002f   // 位置环积分系数：消除残余静差(可先置0，有静差再慢慢加)。
                               // 现象：球差一点到不了中心(停在中心旁边)，就加大 KI_POS。
#define POS_I_LIMIT   2000.0f  // 位置环积分限幅：积分项不会超过该值，防止积分饱和后猛冲。
#define VEL_LIMIT     10.0f    // 期望速度限幅(像素/帧)：防止大误差时猛冲。
                               // 期望速度一顶到该值就封顶。KP_POS 很大时目标速度常被它饱和。

/* ---------- 速度环（内环）参数 ---------- */
#define KP_VEL        40.0f    // 速度环比例系数：速度误差1(像素/帧)对应的角度(度)。
                               // 决定"刹车"强度：球冲过头/来回抖，就加大 KP_VEL。
#define KI_VEL        0.0f     // 速度环积分系数(一般用不到，先置0)。
#define VEL_I_LIMIT   300.0f   // 速度环积分限幅，防止积分饱和。

/* ---------- 公共参数 ---------- */
#define DEAD_BAND_PIXELS  1.0f  // 位置死区(px)：误差小于该值时认为已到位，停止调节防抖动。
#define MAX_DELTA_ANGLE   15.0f // 电机角度输出限幅(度)：最终发给电机的角度上限。
                                // 想让边缘区"加大角度"的效果更明显，可适当加大(如18~20)。
#define VEL_FILTER        0.9f  // 速度一阶低通滤波系数(0~1)：越大滤波越强、速度越平滑。
                                // 像素噪声大就加大；但太大会让速度环反应变慢、刹不住。

/* ---------- 丢球找回参数 ---------- */
#define RECOVERY_ANGLE         5.0f // 找回角度(度)：球丢失后固定用此角度把球推回视野。
                                    // 球回得慢就加大，但别超过 MAX_DELTA_ANGLE，太猛会推过头。
#define RECOVERY_SWEEP_FRAMES  40    // 同一方向盲推多少帧还没找到，就反向扫描一次。
                                     // 按 K230 帧率约10帧/秒估算，40帧≈4秒，够球滚回视野了。

/* ---------- 找回后反向刹车参数 ----------
 * 找球期间电机一直朝某个方向推，杆和球都会积累惯性；
 * 球一重新出现，先朝反方向发力几帧把惯性抵消掉，防止球被甩过头再冲出视野。 */
#define RECOVERY_BRAKE_FRAMES  3     // 找回后反向刹车的帧数(1~5左右，越多刹得越狠)
#define BRAKE_GAIN             1.0f  // 刹车力度倍率(相对找回角度RECOVERY_ANGLE)：
                                     // 觉得惯性大、球还刹不住就加大(如1.5~2.0)

/* ---------- 距离调节(增益调度)参数 ----------
 * 根据球距中心的距离动态改变调节角度：
 *   靠近边缘 → 角度放大，球被用力拉回，防止冲出视野；
 *   靠近中心 → 角度缩小，球平缓减速、稳稳停下。
 * 注意：KP_POS/KP_VEL 很大时角度常顶到 MAX_DELTA_ANGLE 上限，
 *       此时"边缘区放大"效果可能看不出来，需要加大 MAX_DELTA_ANGLE。 */
#define CENTER_THRESHOLD_PX  10.0f   // 距中心多少像素内算"中心区"，该范围内减小调节角度。
#define EDGE_THRESHOLD_PX    250.0f  // 距中心多少像素外算"边缘区"，该范围内加大调节角度。
                                    // K230最大可见误差约±250px，边缘留50px余量：
                                    // 球到200px后进入全力拉回区(EDGE_GAIN)，防止冲出视野。
                                    // 注意：CENTER_THRESHOLD_PX 必须 < EDGE_THRESHOLD_PX，
                                    // 否则中间线性过渡区失效，调度会退化成"全程弱力"。
#define CENTER_GAIN          0.1f  // 中心区角度倍率(<1)：让球到中心附近时温柔减速停下，越小停得越温柔。
#define EDGE_GAIN            3.0f  // 边缘区角度倍率(>1)：球接近视野边缘时用力拉回防冲出，越大拉回越猛。

/* ---------- 串级PID状态变量 ---------- */
static float pos_integral      = 0.0f; // 位置环积分项
static float vel_integral      = 0.0f; // 速度环积分项
static float last_pixel_error  = 0.0f; // 上一帧像素误差，用于相邻两帧差分求速度
static float filtered_velocity = 0.0f; // 低通滤波后的实际速度(像素/帧，正值=朝中心运动)
static uint8_t velocity_ready  = 0;    // 速度是否已由连续两帧建立(0=只有一帧/刚出死区)
static float last_known_error = 0.0f;  // 丢失前最后一帧的有效误差(记住球在中心的哪一侧)
static uint32_t lost_frames = 0;       // 球丢失后经过的帧数(每收到一帧K230数据+1)
static float recover_angle = 0.0f;     // 当前找回方向对应的带符号角度
static uint8_t brake_frames = 0;       // 找回后还剩几帧刹车(>0 = 正在抵消找球惯性)

uint8_t Control_TxPacket[13] = {0};    // 当前发送的控制包，供OLED显示
float Control_DeltaAngle = 0.0f;       // PID 输出角度，供OLED显示
uint32_t Control_Pulses = 0U;          // 实际发送的脉冲数，供OLED显示

float Control_TargetVelocity = 0.0f;   // 位置环输出的期望速度(像素/帧)，供OLED调试
float Control_ActualVelocity = 0.0f;   // 滤波后的实际速度(像素/帧)，供OLED调试
uint8_t Control_BallLost = 0;          // 球是否丢失(1=正在找回)，供OLED显示
float Control_PositionGain = 1.0f;     // 本帧按距离调度出的角度倍率，供OLED调试

static uint32_t angleToPulses(float angle)   // 定义角度转脉冲数函数
{
    if (angle < 0.0f)                  // 如果角度为负数
    {
        angle = -angle;               // 取绝对值，后面按大小计算脉冲
    }
    if (angle < 0.001f)                // 如果角度太小
    {
        return 0;                     // 返回 0，不发送命令
    }
    return (uint32_t)(angle * 3200.0f / 360.0f + 0.5f); // 按 3200 脉冲/360° 计算脉冲数，并四舍五入
}

void angelup(float angle)              // 定义角度控制函数，正数表示上移，负数表示下移
{
    uint8_t d[13] = {0};               // 定义 13 字节数据包，并初始化为 0
    uint8_t dir = 0x00;                // 定义方向变量，默认正方向
    uint32_t pulses = 0U;              // 定义脉冲数变量
    uint8_t i;

    Control_Pulses = 0U;               // 默认不发送，脉冲数记 0

    if (angle > 0.0f)                  // 如果输入角度为正数
    {
        dir = 0x01;                    // 设置方向为正方向
    }
    else if (angle < 0.0f)             // 如果输入角度为负数
    {
        dir = 0x00;                    // 设置方向为反方向
        angle = -angle;                // 将角度变成正数，方便计算脉冲数
    }
    else                               // 如果角度等于 0
    {
        return;                        // 直接返回，不发送命令
    }

    pulses = angleToPulses(angle);     // 将角度转换成脉冲数
    if (pulses == 0U)                  // 如果脉冲数为 0
    {
        return;                        // 直接返回
    }
    Control_Pulses = pulses;           // 记录实际发送的脉冲数

    d[0] = 0x01;                       // 设置通信地址
    d[1] = 0xFD;                       // 设置功能码
    d[2] = dir;                        // 设置方向字段
    d[3] = 0x3E;                       // 设置速度高字节（0x3E80 = 16000 脉冲/秒）
    d[4] = 0x80;                       // 设置速度低字节
    d[5] = 0x00;                       // 设置加速度字段
    d[6] = (pulses >> 24) & 0xFF;      // 设置脉冲数的最高 8 位
    d[7] = (pulses >> 16) & 0xFF;      // 设置脉冲数的次高 8 位
    d[8] = (pulses >> 8) & 0xFF;       // 设置脉冲数的次低 8 位
    d[9] = pulses & 0xFF;              // 设置脉冲数的最低 8 位
    d[10] = 0x02;                      // 设置运动模式
    d[11] = 0x00;                      // 设置不同步标志
    d[12] = 0x6B;                      // 设置固定校验码

    Serial_SendArray(d, 13);           // 调用串口发送函数发送数据

    /* 保存当前发送包供OLED显示 */
    for (i = 0; i < 13; i++)
    {
        Control_TxPacket[i] = d[i];
    }
}

static float positionGain(float pixel_error)   // 按球距中心的距离调度调节角度的倍率
{
    float abs_err;                    // 球距中心的距离(像素，取绝对值)

    /* 误差可能带正负号，先取绝对值，只按"距离"判断在哪个区域 */
    abs_err = (pixel_error < 0.0f) ? -pixel_error : pixel_error;

    if (abs_err <= CENTER_THRESHOLD_PX)   // 中心区：角度最小，让球平缓减速停下
        return CENTER_GAIN;
    if (abs_err >= EDGE_THRESHOLD_PX)     // 边缘区：角度最大，用力拉回防止冲出视野
        return EDGE_GAIN;

    /* 中间区域：在中心力度和边缘力度之间线性过渡，
     * 避免区域交界处力度突变，造成角度跳变、球体抖动 */
    return CENTER_GAIN + (EDGE_GAIN - CENTER_GAIN)
                       * (abs_err - CENTER_THRESHOLD_PX)
                       / (EDGE_THRESHOLD_PX - CENTER_THRESHOLD_PX);
}

void controlByVision(float pixel_error)   // 串级PID视觉控制函数，输入带正负号的像素误差
{
    float actual_velocity = 0.0f;  // 本帧差分得到的原始速度(像素/帧)
    float target_velocity;         // 位置环输出的期望速度(像素/帧)
    float vel_error;               // 速度误差(期望速度 - 实际速度)
    float delta_angle;             // 速度环输出的电机角度(度)
    float pos_gain;                // 本帧按球距中心的距离调度出的角度倍率
    float brake_angle;             // 找回后反向刹车的角度

    /* ============ 0. 能进到这里说明球已重新出现在视野内：恢复串级调控 ============
     * 每帧都要做两件事：
     *   ① 清除"丢失找回"状态(如果之前丢过球)；
     *   ② 记住当前有效误差——它会在球跑丢后用来判断该往哪边推球。
     * 方向约定与串级PID一致："正误差→正角度→球朝中心滚"。 */
    if (lost_frames > 0)               // 上一帧还在"找回"状态 → 球是这一帧刚被找回的
    {
        brake_frames = RECOVERY_BRAKE_FRAMES;   // 开启反向刹车，抵消找球积累的惯性
    }
    Control_BallLost = 0;              // 清除丢失标志(供OLED显示)
    lost_frames      = 0;              // 丢失帧计数清零
    last_known_error = pixel_error;    // 记住球当前在中心的哪一侧

    /* ================== 0.5 反向刹车：抵消找球惯性 ==================
     * 刚找回球的头 RECOVERY_BRAKE_FRAMES 帧，不跑串级PID、也不看死区，
     * 直接朝"找回方向的反向"发力，把找球过程中杆和球积累的惯性抵消掉，
     * 防止球一出现就被惯性甩过头、再次冲出视野。
     * 这里放在死区判断之前：即使球找回时正好在中心附近，也要先刹车。 */
    if (brake_frames > 0)
    {
        brake_angle = -recover_angle * BRAKE_GAIN;  // 反向刹车角度(与找回方向相反)

        brake_frames--;                             // 刹车帧数递减，刹完即止

        if (brake_angle >  MAX_DELTA_ANGLE) brake_angle =  MAX_DELTA_ANGLE; // 限幅
        if (brake_angle < -MAX_DELTA_ANGLE) brake_angle = -MAX_DELTA_ANGLE;

        Control_DeltaAngle     = brake_angle;  // 记录刹车角度，供OLED显示
        Control_TargetVelocity = 0.0f;         // 刹车期间不走位置环
        Control_ActualVelocity = 0.0f;
        angelup(brake_angle);                  // 发送反向刹车指令
        return;                                // 刹车期间不进入死区/串级逻辑
    }

    /* ================== ① 死区判断：球已到中心附近，停止调节 ================== */
    if (pixel_error > -DEAD_BAND_PIXELS && pixel_error < DEAD_BAND_PIXELS)
    {
        pos_integral      = 0.0f;  // 清零位置积分，避免球停稳后误差抖动被积分放大
        vel_integral      = 0.0f;  // 清零速度积分
        filtered_velocity = 0.0f;  // 实际速度清零，避免残留速度让下一帧误判
        last_pixel_error  = pixel_error; // 仍刷新"上一帧"，保证出死区时速度是连续差分出的
        velocity_ready    = 0;     // 置为无效：出死区后用连续两帧重新建立速度

        Control_DeltaAngle     = 0.0f; // 死区内不动作
        Control_Pulses         = 0U;   // 死区内不发送脉冲
        Control_TargetVelocity = 0.0f; // 显示值同步清零
        Control_ActualVelocity = 0.0f;
        return;                        // 直接返回，不往下执行
    }

    /* ================== ② 用相邻两帧的误差差分出实际速度(速度环的反馈量) ==================
     * 上一帧误差 - 本帧误差：误差变小 = 球在朝中心运动，差值为正，
     * 所以"正值 = 朝中心运动的速度"。这个速度就是速度环拿来作反馈的量。 */
    if (velocity_ready == 0)
    {
        /* 首帧(或球刚重新出现)：没有上一帧可差分，先只记录本帧误差，
         * 速度按 0 处理，下一帧起才用连续两帧算出真实速度，避免首帧速度跳变。 */
        last_pixel_error  = pixel_error;
        velocity_ready    = 1;
        filtered_velocity = 0.0f;
    }
    else
    {
        /* 有上一帧：差分出原始速度，再做一阶低通滤波，滤掉单帧像素噪声。
         * 滤波后的速度会平缓很多，速度环不会被像素抖动牵着走。 */
        actual_velocity   = last_pixel_error - pixel_error;        // 原始速度
        filtered_velocity = VEL_FILTER * filtered_velocity         // 低通滤波：
                          + (1.0f - VEL_FILTER) * actual_velocity; // 历史+新值加权
        last_pixel_error  = pixel_error;  // 把本帧误差记成"上一帧"，供下一帧继续差分
    }

    /* ================== ③ 位置环(外环)：像素误差 → 期望速度 ==================
     * 球离中心越远，期望速度越大；越靠近中心，期望速度越小，实现"远快近慢"，
     * 到中心附近时期望速度自然趋近 0，这是球能停下来的前提。 */
    pos_integral += pixel_error;                      // 累加误差得到位置积分项
    if (pos_integral >  POS_I_LIMIT) pos_integral =  POS_I_LIMIT; // 积分限幅防饱和
    if (pos_integral < -POS_I_LIMIT) pos_integral = -POS_I_LIMIT;

    /* 位置环输出 = 期望速度。
     * 串级结构下位置环不再需要微分项——速度环本身就起阻尼作用，防止过冲。 */
    target_velocity = KP_POS * pixel_error + KI_POS * pos_integral;

    if (target_velocity >  VEL_LIMIT) target_velocity =  VEL_LIMIT; // 期望速度限幅，防大误差猛冲
    if (target_velocity < -VEL_LIMIT) target_velocity = -VEL_LIMIT;

    /* ================== ④ 速度环(内环)：速度误差 → 电机角度 ==================
     * 速度误差为 0 = 球正以期望的速度朝中心走，此时角度输出很小；
     * 实际速度超过期望(球要冲过头)时速度误差变负 → 电机反向减力 → 刹住，
     * 这就是串级PID能"停得住、不冲过头"的关键所在。 */
    vel_error = target_velocity - filtered_velocity;

    vel_integral += vel_error;                        // 速度积分项(一般用不到，保留结构)
    if (vel_integral >  VEL_I_LIMIT) vel_integral =  VEL_I_LIMIT;
    if (vel_integral < -VEL_I_LIMIT) vel_integral = -VEL_I_LIMIT;

    /* 速度环输出 = 电机角度。
     * 符号说明：速度误差为正(球还需加速朝中心走)→角度为正→angelup朝上推球，
     * 与原来单环"误差为正→角度为正"的符号保持一致，方向不会反。 */
    delta_angle = KP_VEL * vel_error + KI_VEL * vel_integral;

    /* ================== ⑤ 按距离调度调节角度，然后限幅并发送给电机 ==================
     * 根据球距中心的距离调整最终角度：
     *   靠近边缘 → 角度放大(EDGE_GAIN)，球被更用力地拉回，防止冲出视野；
     *   靠近中心 → 角度缩小(CENTER_GAIN)，球平缓减速，配合速度环刹车稳稳停在中间。
     * 倍率由 positionGain() 按误差大小算出，中心区与边缘区之间线性过渡。 */
    pos_gain = positionGain(pixel_error);        // 算出本帧的力度倍率
    delta_angle *= pos_gain;                     // 用该倍率缩放最终角度

    if (delta_angle >  MAX_DELTA_ANGLE) delta_angle =  MAX_DELTA_ANGLE; // 角度限幅
    if (delta_angle < -MAX_DELTA_ANGLE) delta_angle = -MAX_DELTA_ANGLE;

    Control_DeltaAngle     = delta_angle;       // 记录输出角度，供OLED显示
    Control_TargetVelocity = target_velocity;   // 记录期望速度，供OLED显示
    Control_ActualVelocity = filtered_velocity; // 记录实际速度，供OLED显示
    Control_PositionGain   = pos_gain;          // 记录本帧调度倍率，供OLED调试

    angelup(delta_angle);                       // 最后把角度送给电机控制函数执行
}

void controlLostBall(void)   // 球丢失时的找回控制：主动把球推回视野，不让它脱管
{
    /* 刚丢失的第一帧：根据最后已知误差确定初始找回方向。
     * 方向约定与串级PID完全一致——"正误差→正角度→球朝中心滚"，
     * 所以找回时推球的方向与最后已知误差同号，等于沿用控制器原来的推法，
     * 把跑出视野的球再推回中心、推回视野内。 */
    if (lost_frames == 0)
    {
        if (last_known_error > 0.5f)          // 球当时在中心正方向一侧
            recover_angle =  RECOVERY_ANGLE;  // 朝正方向推(球会朝中心滚)
        else if (last_known_error < -0.5f)    // 球当时在中心负方向一侧
            recover_angle = -RECOVERY_ANGLE;  // 朝负方向推
        else                                  // 误差未知(例如在中心附近直接丢的)
            recover_angle =  RECOVERY_ANGLE;  // 先往一个方向试，找不到再扫描
    }

    lost_frames++;                            // 每收到一帧K230数据，丢失计数+1

    /* 同一个方向盲推了 RECOVERY_SWEEP_FRAMES 帧还没找到球，
     * 说明这个方向可能推反了，就反向扫描一次，避免一直朝错误方向推导致丢球到底。 */
    if (lost_frames % RECOVERY_SWEEP_FRAMES == 0)
    {
        recover_angle = -recover_angle;
    }

    /* 找回期间不跑串级PID，只发固定角度盲推；
     * 同时清空串级状态，等球回来后能干净地重新进入串级调控。 */
    pos_integral      = 0.0f;
    vel_integral      = 0.0f;
    filtered_velocity = 0.0f;
    velocity_ready    = 0;

    Control_BallLost       = 1;              // 置丢失标志(供OLED显示)
    Control_DeltaAngle     = recover_angle;  // 记录输出角度，供OLED显示
    Control_TargetVelocity = 0.0f;           // 找回期间没有速度概念，显示清0
    Control_ActualVelocity = 0.0f;
    angelup(recover_angle);                  // 发送给电机执行
}
