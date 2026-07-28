#include "gyro.h"
#include <string.h>

/*============================================================================
 * 内部数据存储
 *===========================================================================*/

static Gyro_Angle  g_angle  = {0};
static Gyro_Rate   g_rate   = {0};
static Gyro_Accel  g_accel  = {0};

/*============================================================================
 * 发送辅助函数
 *===========================================================================*/

static void Gyro_SendByte(uint8_t data)
{
    while (DL_UART_isBusy(Gyro_INST));
    DL_UART_Main_transmitData(Gyro_INST, data);
}

static void Gyro_SendBytes(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        Gyro_SendByte(data[i]);
    }
}

/*============================================================================
 * 数据解析函数：接收0x5A开头的数据帧
 * 支持：角速度(0xAA)、角度(0xBB)、加速度(0xCC)、四元数(0xDD)
 *===========================================================================*/
void Gyro_ParseByte(uint8_t ucData)
{
    static uint8_t ucRxBuffer[11];
    static uint8_t ucRxCnt = 0;
    uint8_t sum = 0;

    /* 缓存数据 */
    ucRxBuffer[ucRxCnt++] = ucData;

    /* 帧头校验 */
    if (ucRxBuffer[0] != 0x5A)
    {
        ucRxCnt = 0;
        return;
    }

    /* 等待完整帧 (11字节) */
    if (ucRxCnt < 11) return;

    /* 根据TYPE计算校验和 */
    switch (ucRxBuffer[1])
    {
        case 0xAA:  /* 角速度 */
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum == ucRxBuffer[10])
            {
                short wx = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short wy = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short wz = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                g_rate.wx = (float)wx / 32768.0f * 2000.0f;
                g_rate.wy = (float)wy / 32768.0f * 2000.0f;
                g_rate.wz = (float)wz / 32768.0f * 2000.0f;
            }
            break;

        case 0xBB:  /* 角度 */
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum == ucRxBuffer[10])
            {
                short roll  = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short pitch = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short yaw   = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                g_angle.Roll  = (float)roll  / 32768.0f * 180.0f;
                g_angle.Pitch = (float)pitch / 32768.0f * 180.0f;
                g_angle.Yaw   = (float)yaw   / 32768.0f * 180.0f;
            }
            break;

        case 0xCC:  /* 加速度 */
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum == ucRxBuffer[10])
            {
                short ax = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short ay = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short az = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                const float G = 9.8f;
                g_accel.ax = (float)ax / 32768.0f * 16.0f * G;
                g_accel.ay = (float)ay / 32768.0f * 16.0f * G;
                g_accel.az = (float)az / 32768.0f * 16.0f * G;
            }
            break;

        case 0xDD:  /* 四元数 (暂不存储) */
        case 0xEE:  /* 寄存器 (暂不处理) */
            break;

        default:
            break;
    }

    /* 解析完成，复位计数器 */
    ucRxCnt = 0;
}

/*============================================================================
 * 初始化
 *===========================================================================*/
void Gyro_Init(void)
{
    /* 使能 UART 中断 */
    NVIC_ClearPendingIRQ(Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(Gyro_INST_INT_IRQN);

    /* 注意：不在这里发送校准命令，避免阻塞 */
    /* 需要校准时手动调用 Gyro_CaliYaw() */
}

/*============================================================================
 * 角度获取接口
 *===========================================================================*/
float Gyro_GetYaw(void)   { return g_angle.Yaw; }
float Gyro_GetRoll(void)  { return g_angle.Roll; }
float Gyro_GetPitch(void) { return g_angle.Pitch; }

/*============================================================================
 * 角速度获取接口
 *===========================================================================*/
float Gyro_GetRateX(void) { return g_rate.wx; }
float Gyro_GetRateY(void) { return g_rate.wy; }
float Gyro_GetRateZ(void) { return g_rate.wz; }

/*============================================================================
 * 加速度获取接口
 *===========================================================================*/
float Gyro_GetAccelX(void) { return g_accel.ax; }
float Gyro_GetAccelY(void) { return g_accel.ay; }
float Gyro_GetAccelZ(void) { return g_accel.az; }

/*============================================================================
 * 校准命令
 *===========================================================================*/

/* 解锁指令 */
static uint8_t cmd_unlock[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
/* Z轴角度归零指令 */
static uint8_t cmd_yaw_zero[5] = {0x55, 0xAA, 0x0A, 0x04, 0x00};
/* 保存指令 */
static uint8_t cmd_save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};
/* 零偏校准指令 */
static uint8_t cmd_bias_cal[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};

/**
 * @brief Z轴角度归零
 */
void Gyro_CaliYaw(void)
{
    Gyro_SendBytes(cmd_unlock, 5);
    delay_ms(100);
    Gyro_SendBytes(cmd_yaw_zero, 5);
    delay_ms(100);
    Gyro_SendBytes(cmd_save, 5);
}

/**
 * @brief 零偏校准（校准过程中请勿移动！）
 */
void Gyro_CaliBias(void)
{
    Gyro_SendBytes(cmd_unlock, 5);
    delay_ms(100);
    Gyro_SendBytes(cmd_bias_cal, 5);
    delay_ms(6000);  /* 等待6秒校准完成 */
    Gyro_SendBytes(cmd_save, 5);
}
