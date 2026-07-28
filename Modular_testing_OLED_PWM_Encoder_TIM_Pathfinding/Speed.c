#include "Speed.h"

/**
 * @brief  更新速度数据
 * @param  data 速度数据结构体指针
 * @param  encoder_count 编码器增量值
 * @param  dir 方向系数（1 或 -1）
 */
void Speed_Update(Speed_Data *data, int16_t encoder_count, int8_t dir)
{
    /* 保存原始脉冲数 */
    data->pulse_count = encoder_count * dir;

    /* 计算转速（RPM） */
    data->rpm = (float)data->pulse_count * PULSE_TO_RPM;
}
