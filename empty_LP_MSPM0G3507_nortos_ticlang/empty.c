/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "ti_msp_dl_config.h"
#include "OLED.h"

/* 编码器位置计数变量 */
volatile int32_t encoder_position = 0;
volatile uint32_t interrupt_count = 0;  /* 调试：中断触发计数 */

/* 双通道时序法判断方向 */
volatile uint32_t last_a_time = 0;  /* A 相上次捕获时间 */
volatile uint32_t last_b_time = 0;  /* B 相上次捕获时间 */

int main(void)
{
    /* 1. 初始化系统 */
    SYSCFG_DL_init();

    /* 2. 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Encoder:");

    /* 3. 使能定时器中断 */
    NVIC_EnableIRQ(Encoder2_INST_INT_IRQN);

    /* 4. 启动定时器 */
    DL_TimerG_startCounter(Encoder2_INST);

    while (1)
    {
        /* OLED 显示编码器位置 */
        OLED_ShowSignedNum(2, 1, encoder_position, 8);

        /* 调试：显示中断触发次数 */
        OLED_ShowNum(3, 1, interrupt_count, 8);
    }
}

/*
 * 中断服务函数 - 双通道时序法判断方向
 *
 * 原理：
 *   正转时：A 上升沿先到 → 短时间后 B 上升沿到
 *   反转时：B 上升沿先到 → 短时间后 A 上升沿到
 *
 *   A 上升沿时：如果 B 刚触发过（时间差小）→ B 先到 → 反转
 *   B 上升沿时：如果 A 刚触发过（时间差小）→ A 先到 → 正转
 */
void Encoder2_INST_IRQHandler(void)
{
    uint32_t now = DL_TimerG_getTimerCount(Encoder2_INST);

    switch (DL_TimerG_getPendingInterrupt(Encoder2_INST))
    {
        case DL_TIMERG_IIDX_CC0_UP:  /* A 相上升沿 */
            interrupt_count++;
            /* 如果 B 刚刚触发过（时间差 < 阈值），说明 B 先到 → 反转 */
            if ((now - last_b_time) < 50000)
                encoder_position--;
            else
                encoder_position++;
            last_a_time = now;
            break;

        case DL_TIMERG_IIDX_CC1_UP:  /* B 相上升沿 */
            /* 如果 A 刚刚触发过（时间差 < 阈值），说明 A 先到 → 正转 */
            if ((now - last_a_time) < 50000)
                encoder_position++;
            else
                encoder_position--;
            last_b_time = now;
            break;

        default:
            break;
    }
}
