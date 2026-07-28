/**
  ******************************************************************************
  * @file    GyroUART.c
  * @brief   串口陀螺仪驱动模块
  * @note    适用于六轴陀螺仪模块（串口通信版本）
  *          协议: 0x5A + TYPE + DATA(8字节) + SUM(共11字节)
  ******************************************************************************
  */
#include "GyroUART.h"
#include "bsp_uart.h"

/* 全局数据变量定义 */
GyroData_t stcGyro = {0};
AngleData_t stcAngle = {0};
AccelData_t stcAccel = {0};
QuatData_t stcQuat = {0};

/* 命令定义 */
static uint8_t Key[5]      = {0x55, 0xAA, 0x13, 0x8E, 0x5F};  // 解锁指令
static uint8_t Yaw_Zero[5] = {0x55, 0xAA, 0x0A, 0x04, 0x00};  // Z轴角度归零
static uint8_t Save[5]     = {0x55, 0xAA, 0x00, 0x00, 0x00};  // 保存指令
static uint8_t BIAS_CAL[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};  // 零偏校准

/**
  * @brief  串口陀螺仪初始化
  * @param  无
  * @retval 无
  */
void GyroUART_Init(void)
{
    /* 清零数据 */
    stcGyro.wx = 0;
    stcGyro.wy = 0;
    stcGyro.wz = 0;
    stcAngle.Roll = 0;
    stcAngle.Pitch = 0;
    stcAngle.Yaw = 0;
    stcAccel.ax = 0;
    stcAccel.ay = 0;
    stcAccel.az = 0;
}

/**
  * @brief  串口数据解析函数
  * @param  ucData: 接收到的单字节数据
  * @retval 无
  * @note   需要在串口中断中调用此函数
  */
void GyroUART_ProcessByte(uint8_t ucData)
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

    /* 等待完整帧(11字节) */
    if (ucRxCnt < 11) return;

    /* 根据TYPE计算校验和 */
    switch (ucRxBuffer[1])
    {
        case 0xAA:  // 角速度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }

            /* 解析角速度 */
            {
                short wx = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short wy = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short wz = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                stcGyro.wx = (float)wx / 32768.0f * 2000.0f;  // °/s
                stcGyro.wy = (float)wy / 32768.0f * 2000.0f;
                stcGyro.wz = (float)wz / 32768.0f * 2000.0f;
            }
            break;

        case 0xBB:  // 角度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }

            /* 解析角度 */
            {
                short roll  = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short pitch = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short yaw   = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                stcAngle.Roll  = (float)roll  / 32768.0f * 180.0f;  // °
                stcAngle.Pitch = (float)pitch / 32768.0f * 180.0f;
                stcAngle.Yaw   = (float)yaw   / 32768.0f * 180.0f;
            }
            break;

        case 0xCC:  // 加速度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }

            /* 解析加速度 */
            {
                short ax = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short ay = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short az = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);

                const float G = 9.8f;
                stcAccel.ax = (float)ax / 32768.0f * 16.0f * G;  // m/s²
                stcAccel.ay = (float)ay / 32768.0f * 16.0f * G;
                stcAccel.az = (float)az / 32768.0f * 16.0f * G;
            }
            break;

        case 0xDD:  // 四元数
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];

            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }

            /* 解析四元数 */
            {
                short q0 = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short q1 = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short q2 = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
                short q3 = (short)((ucRxBuffer[9] << 8) | ucRxBuffer[8]);

                stcQuat.q0 = (float)q0 / 32768.0f;
                stcQuat.q1 = (float)q1 / 32768.0f;
                stcQuat.q2 = (float)q2 / 32768.0f;
                stcQuat.q3 = (float)q3 / 32768.0f;
            }
            break;

        default:
            ucRxCnt = 0;
            return;
    }

    /* 解析成功，复位接收计数器 */
    ucRxCnt = 0;
}

/*============================================================================
 * 数据获取接口函数
 *===========================================================================*/

float GyroUART_GetYaw(void)   { return stcAngle.Yaw; }
float GyroUART_GetRoll(void)  { return stcAngle.Roll; }
float GyroUART_GetPitch(void) { return stcAngle.Pitch; }

float GyroUART_GetGyroX(void) { return stcGyro.wx; }
float GyroUART_GetGyroY(void) { return stcGyro.wy; }
float GyroUART_GetGyroZ(void) { return stcGyro.wz; }

float GyroUART_GetAccelX(void) { return stcAccel.ax; }
float GyroUART_GetAccelY(void) { return stcAccel.ay; }
float GyroUART_GetAccelZ(void) { return stcAccel.az; }

/*============================================================================
 * 命令发送函数
 *===========================================================================*/

/**
  * @brief  发送解锁命令
  * @param  无
  * @retval 无
  */
void GyroUART_SendUnlock(void)
{
    UART_SendBytes(Key, 5);
}

/**
  * @brief  发送Z轴角度归零命令
  * @param  无
  * @retval 无
  */
void GyroUART_SendCaliYaw(void)
{
    UART_SendBytes(Key, 5);
    for (volatile uint32_t i = 0; i < 1000000; i++);
    UART_SendBytes(Yaw_Zero, 5);
    for (volatile uint32_t i = 0; i < 1000000; i++);
    UART_SendBytes(Save, 5);
}

/**
  * @brief  发送零偏校准命令
  * @param  无
  * @retval 无
  * @note   校准过程中请勿移动设备，约需6秒
  */
void GyroUART_SendCaliBias(void)
{
    UART_SendBytes(Key, 5);
    for (volatile uint32_t i = 0; i < 1000000; i++);
    UART_SendBytes(BIAS_CAL, 5);
    for (volatile uint32_t i = 0; i < 60000000; i++);  // 等待6秒
    UART_SendBytes(Save, 5);
}

/**
  * @brief  发送保存命令
  * @param  无
  * @retval 无
  */
void GyroUART_SendSave(void)
{
    UART_SendBytes(Save, 5);
}
