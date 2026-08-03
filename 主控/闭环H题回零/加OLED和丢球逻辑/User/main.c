#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"

/* 电机：Emm固件，1.8度，256细分。 */
#define PULSES_PER_REV       51200.0f
#define MAX_ANGLE_DEG            7.0f
#define MOTOR_SPEED_RPM         60U
#define MOTOR_ACCEL            240U

/* 位置PID控制。球左正右负，电机CW降低右端。 */
#define KP                       0.05f
#define KI                       0.03f
#define KD                       0.013f
#define I_ANGLE_LIMIT             2.0f
#define X_FILTER                 0.50f
#define V_FILTER                 0.35f
#define MAX_ANGLE_STEP           1.2f
#define MIN_ANGLE_CHANGE         0.1f
#define VISION_TIMEOUT_MS      150U
#define LOST_CONFIRM_MS         40U
#define RECOVERY_SWEEP_MS     1200U
#define RECOVERY_TIMEOUT_MS   4800U
#define RECOVERY_ANGLE_DEG       4.0f
#define OLED_REFRESH_MS        200U

static volatile uint32_t ms;
static float filtered_x;
static float last_x;
static float ball_speed;
static float integral_angle;
static float motor_angle;
static uint8_t filter_ready;
static uint8_t motor_is_zero;

static void OLED_ShowStatus(int16_t x, uint8_t valid)
{
    OLED_ShowSignedNum(1, 3, x, 4);
    OLED_ShowSignedNum(2, 3, (int32_t)ball_speed, 4);
    OLED_ShowSignedNum(3, 3, (int32_t)(motor_angle * 10.0f), 3);
    OLED_ShowString(4, 3, valid ? "OK  " : "LOST");
}

static float limit(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Time_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    tim.TIM_Prescaler = 7200 - 1;
    tim.TIM_Period = 10 - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        ++ms;
    }
}

static void Motor_Enable(void)
{
    const uint8_t cmd[] = {0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B};
    MotorSerial_SendArray(cmd, sizeof(cmd));
}

static void Motor_GoZero(void)
{
    const uint8_t cmd[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
    MotorSerial_SendArray(cmd, sizeof(cmd));
    motor_angle = 0.0f;
    motor_is_zero = 1;
}

static void Motor_MoveAbsolute(float angle)
{
    uint8_t cmd[13];
    uint32_t pulses;
    uint8_t direction;

    angle = limit(angle, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    direction = (angle >= 0.0f) ? 0x01 : 0x00;
    pulses = (uint32_t)(absolute(angle) * PULSES_PER_REV / 360.0f + 0.5f);

    cmd[0] = 0x01;
    cmd[1] = 0xFD;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(MOTOR_SPEED_RPM >> 8);
    cmd[4] = (uint8_t)MOTOR_SPEED_RPM;
    cmd[5] = MOTOR_ACCEL;
    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)pulses;
    cmd[10] = 0x01; /* 相对坐标零点的绝对位置 */
    cmd[11] = 0x00;
    cmd[12] = 0x6B;
    MotorSerial_SendArray(cmd, sizeof(cmd));

    motor_angle = angle;
    motor_is_zero = (pulses == 0U);
}

static void Control_Reset(float x)
{
    filtered_x = x;
    last_x = x;
    ball_speed = 0.0f;
    integral_angle = 0.0f;
    filter_ready = 0;
}

static void Control_Update(int16_t x, float dt)
{
    float speed;
    float target;
    float change;

    if (!filter_ready)
    {
        Control_Reset((float)x);
        filter_ready = 1;
        return;
    }

    filtered_x += X_FILTER * ((float)x - filtered_x);
    speed = (filtered_x - last_x) / dt;
    last_x = filtered_x;
    ball_speed += V_FILTER * (limit(speed, -3000.0f, 3000.0f) - ball_speed);

    /* 积分输出直接以角度累加并限幅，消除静差且防止积分饱和。 */
    integral_angle += KI * filtered_x * dt;
    integral_angle = limit(integral_angle, -I_ANGLE_LIMIT, I_ANGLE_LIMIT);

    /* KD会在球速度过快时产生反向小角度，提前刹车。 */
    target = KP * filtered_x + integral_angle + KD * ball_speed;
    if ((absolute(filtered_x) < 5.0f) && (absolute(ball_speed) < 20.0f))
    {
        target = 0.0f;
        integral_angle = 0.0f;
    }

    target = limit(target, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    change = limit(target - motor_angle, -MAX_ANGLE_STEP, MAX_ANGLE_STEP);
    target = motor_angle + change;

    if (absolute(target - motor_angle) >= MIN_ANGLE_CHANGE)
        Motor_MoveAbsolute(target);
}

int main(void)
{
    VisionSample vision;
    uint32_t last_sequence = 0;
    uint32_t last_vision_ms = 0;
    uint32_t last_valid_ms = 0;
    uint32_t recovery_start_ms = 0;
    uint32_t recovery_sweep_ms = 0;
    uint32_t last_sample_ms = 0;
    uint32_t last_oled_ms = 0;
    uint32_t dt_ms;
    int16_t last_valid_x = 0;
    uint8_t have_valid_position = 0;
    uint8_t recovering = 0;
    uint8_t recovery_attempted = 0;
    float recovery_angle = RECOVERY_ANGLE_DEG;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Time_Init();
    Serial_Init();
    OLED_Init();
    OLED_ShowString(1, 1, "X:");
    OLED_ShowString(2, 1, "V:");
    OLED_ShowString(3, 1, "A:");
    OLED_ShowString(4, 1, "S:");
    OLED_ShowStatus(0, 0);

    Delay_ms(500);
    Motor_Enable();
    Delay_ms(100);
    Motor_GoZero();
    Delay_ms(2000);
    Control_Reset(0.0f);

    while (1)
    {
        Vision_GetLatest(&vision);

        if (vision.sequence != last_sequence)
        {
            dt_ms = ms - last_sample_ms;
            last_sample_ms = ms;
            last_sequence = vision.sequence;
            last_vision_ms = ms;

            if (vision.valid && dt_ms >= 5U && dt_ms <= 100U)
            {
                recovery_attempted = 0;
                if (recovering)
                {
                    Motor_MoveAbsolute(0.0f);
                    Control_Reset((float)vision.x);
                    recovering = 0;
                }
                Control_Update(vision.x, (float)dt_ms * 0.001f);
                last_valid_x = vision.x;
                last_valid_ms = ms;
                have_valid_position = 1;
            }
        }

        if ((uint32_t)(ms - last_oled_ms) >= OLED_REFRESH_MS)
        {
            OLED_ShowStatus(vision.x,
                            vision.valid &&
                            ((uint32_t)(ms - last_vision_ms) <=
                             VISION_TIMEOUT_MS));
            last_oled_ms = ms;
        }

        /* 整条K230串口中断：不判断方向，立即回零。 */
        if ((uint32_t)(ms - last_vision_ms) > VISION_TIMEOUT_MS)
        {
            if (!motor_is_zero)
                Motor_GoZero();
            Control_Reset(0.0f);
            recovering = 0;
            continue;
        }

        /* K230仍在发包但valid=0：在正负找回角度之间往复扫描。 */
        if (!vision.valid)
        {
            if (!recovering && !recovery_attempted &&
                (uint32_t)(ms - last_valid_ms) >= LOST_CONFIRM_MS)
            {
                /* 有历史位置时先向中心方向推；未知位置时从正方向开始。 */
                recovery_angle = (have_valid_position && last_valid_x < 0) ?
                                 -RECOVERY_ANGLE_DEG : RECOVERY_ANGLE_DEG;
                Motor_MoveAbsolute(recovery_angle);
                recovering = 1;
                recovery_attempted = 1;
                recovery_start_ms = ms;
                recovery_sweep_ms = ms;
            }

            /* 固定间隔反向摆动；超时仍未找回则停止并回零。 */
            if (recovering &&
                (uint32_t)(ms - recovery_start_ms) >=
                RECOVERY_TIMEOUT_MS)
            {
                Motor_GoZero();
                Control_Reset(0.0f);
                recovering = 0;
                have_valid_position = 0;
            }
            else if (recovering &&
                     (uint32_t)(ms - recovery_sweep_ms) >=
                     RECOVERY_SWEEP_MS)
            {
                recovery_angle = -recovery_angle;
                Motor_MoveAbsolute(recovery_angle);
                recovery_sweep_ms = ms;
            }
        }
    }
}
