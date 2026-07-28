/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             16000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_8
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM19)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM29)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM29_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_0_C2_PORT                                                 GPIOB
#define GPIO_PWM_0_C2_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_0_C2_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_PWM_0_C2_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA0_CCP2
#define GPIO_PWM_0_C2_IDX                                    DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_0_C3_PORT                                                 GPIOB
#define GPIO_PWM_0_C3_PIN                                         DL_GPIO_PIN_26
#define GPIO_PWM_0_C3_IOMUX                                      (IOMUX_PINCM57)
#define GPIO_PWM_0_C3_IOMUX_FUNC                     IOMUX_PINCM57_PF_TIMA0_CCP3
#define GPIO_PWM_0_C3_IDX                                    DL_TIMER_CC_3_INDEX



/* Defines for PID_TIM */
#define PID_TIM_INST                                                     (TIMG0)
#define PID_TIM_INST_IRQHandler                                 TIMG0_IRQHandler
#define PID_TIM_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define PID_TIM_INST_LOAD_VALUE                                           (999U)
/* Defines for Pathfinding_TIM */
#define Pathfinding_TIM_INST                                             (TIMA1)
#define Pathfinding_TIM_INST_IRQHandler                         TIMA1_IRQHandler
#define Pathfinding_TIM_INST_INT_IRQN                           (TIMA1_INT_IRQn)
#define Pathfinding_TIM_INST_LOAD_VALUE                                  (3999U)




/* Defines for OLED */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 400000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                         DL_GPIO_PIN_10
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM21)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM21_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                         DL_GPIO_PIN_11
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM22)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM22_PF_I2C0_SCL


/* Defines for Gyro */
#define Gyro_INST                                                          UART2
#define Gyro_INST_FREQUENCY                                             40000000
#define Gyro_INST_IRQHandler                                    UART2_IRQHandler
#define Gyro_INST_INT_IRQN                                        UART2_INT_IRQn
#define GPIO_Gyro_RX_PORT                                                  GPIOB
#define GPIO_Gyro_TX_PORT                                                  GPIOB
#define GPIO_Gyro_RX_PIN                                          DL_GPIO_PIN_16
#define GPIO_Gyro_TX_PIN                                          DL_GPIO_PIN_15
#define GPIO_Gyro_IOMUX_RX                                       (IOMUX_PINCM33)
#define GPIO_Gyro_IOMUX_TX                                       (IOMUX_PINCM32)
#define GPIO_Gyro_IOMUX_RX_FUNC                        IOMUX_PINCM33_PF_UART2_RX
#define GPIO_Gyro_IOMUX_TX_FUNC                        IOMUX_PINCM32_PF_UART2_TX
#define Gyro_BAUD_RATE                                                  (115200)
#define Gyro_IBRD_40_MHZ_115200_BAUD                                        (21)
#define Gyro_FBRD_40_MHZ_115200_BAUD                                        (45)
/* Defines for HC */
#define HC_INST                                                            UART1
#define HC_INST_FREQUENCY                                               40000000
#define HC_INST_IRQHandler                                      UART1_IRQHandler
#define HC_INST_INT_IRQN                                          UART1_INT_IRQn
#define GPIO_HC_RX_PORT                                                    GPIOB
#define GPIO_HC_TX_PORT                                                    GPIOB
#define GPIO_HC_RX_PIN                                             DL_GPIO_PIN_7
#define GPIO_HC_TX_PIN                                             DL_GPIO_PIN_6
#define GPIO_HC_IOMUX_RX                                         (IOMUX_PINCM24)
#define GPIO_HC_IOMUX_TX                                         (IOMUX_PINCM23)
#define GPIO_HC_IOMUX_RX_FUNC                          IOMUX_PINCM24_PF_UART1_RX
#define GPIO_HC_IOMUX_TX_FUNC                          IOMUX_PINCM23_PF_UART1_TX
#define HC_BAUD_RATE                                                    (115200)
#define HC_IBRD_40_MHZ_115200_BAUD                                          (21)
#define HC_FBRD_40_MHZ_115200_BAUD                                          (45)





/* Port definition for Pin Group HC05 */
#define HC05_PORT                                                        (GPIOB)

/* Defines for STATE: GPIOB.23 with pinCMx 51 on package pin 22 */
#define HC05_STATE_PIN                                          (DL_GPIO_PIN_23)
#define HC05_STATE_IOMUX                                         (IOMUX_PINCM51)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.24 with pinCMx 54 on package pin 25 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_24)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM54)
/* Defines for AIN2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_25)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM55)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOB)

/* Defines for BIN1: GPIOB.20 with pinCMx 48 on package pin 19 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_20)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM48)
/* Defines for BIN2: GPIOB.25 with pinCMx 56 on package pin 27 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_25)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM56)
/* Port definition for Pin Group Encoders */
#define Encoders_PORT                                                    (GPIOA)

/* Defines for E1: GPIOA.26 with pinCMx 59 on package pin 30 */
// pins affected by this interrupt request:["E1","E2"]
#define Encoders_INT_IRQN                                       (GPIOA_INT_IRQn)
#define Encoders_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Encoders_E1_IIDX                                    (DL_GPIO_IIDX_DIO26)
#define Encoders_E1_PIN                                         (DL_GPIO_PIN_26)
#define Encoders_E1_IOMUX                                        (IOMUX_PINCM59)
/* Defines for E2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define Encoders_E2_IIDX                                    (DL_GPIO_IIDX_DIO27)
#define Encoders_E2_PIN                                         (DL_GPIO_PIN_27)
#define Encoders_E2_IOMUX                                        (IOMUX_PINCM60)
/* Defines for L3: GPIOA.30 with pinCMx 5 on package pin 37 */
#define Pathfinding_L3_PORT                                              (GPIOA)
#define Pathfinding_L3_PIN                                      (DL_GPIO_PIN_30)
#define Pathfinding_L3_IOMUX                                      (IOMUX_PINCM5)
/* Defines for L2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define Pathfinding_L2_PORT                                              (GPIOB)
#define Pathfinding_L2_PIN                                       (DL_GPIO_PIN_1)
#define Pathfinding_L2_IOMUX                                     (IOMUX_PINCM13)
/* Defines for L1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define Pathfinding_L1_PORT                                              (GPIOB)
#define Pathfinding_L1_PIN                                       (DL_GPIO_PIN_0)
#define Pathfinding_L1_IOMUX                                     (IOMUX_PINCM12)
/* Defines for MC: GPIOA.7 with pinCMx 14 on package pin 49 */
#define Pathfinding_MC_PORT                                              (GPIOA)
#define Pathfinding_MC_PIN                                       (DL_GPIO_PIN_7)
#define Pathfinding_MC_IOMUX                                     (IOMUX_PINCM14)
/* Defines for R1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define Pathfinding_R1_PORT                                              (GPIOB)
#define Pathfinding_R1_PIN                                      (DL_GPIO_PIN_10)
#define Pathfinding_R1_IOMUX                                     (IOMUX_PINCM27)
/* Defines for R2: GPIOB.14 with pinCMx 31 on package pin 2 */
#define Pathfinding_R2_PORT                                              (GPIOB)
#define Pathfinding_R2_PIN                                      (DL_GPIO_PIN_14)
#define Pathfinding_R2_IOMUX                                     (IOMUX_PINCM31)
/* Defines for R3: GPIOB.11 with pinCMx 28 on package pin 63 */
#define Pathfinding_R3_PORT                                              (GPIOB)
#define Pathfinding_R3_PIN                                      (DL_GPIO_PIN_11)
#define Pathfinding_R3_IOMUX                                     (IOMUX_PINCM28)
/* Port definition for Pin Group key */
#define key_PORT                                                         (GPIOB)

/* Defines for K1: GPIOB.17 with pinCMx 43 on package pin 14 */
// pins affected by this interrupt request:["K1","K2"]
#define key_INT_IRQN                                            (GPIOB_INT_IRQn)
#define key_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define key_K1_IIDX                                         (DL_GPIO_IIDX_DIO17)
#define key_K1_PIN                                              (DL_GPIO_PIN_17)
#define key_K1_IOMUX                                             (IOMUX_PINCM43)
/* Defines for K2: GPIOB.18 with pinCMx 44 on package pin 15 */
#define key_K2_IIDX                                         (DL_GPIO_IIDX_DIO18)
#define key_K2_PIN                                              (DL_GPIO_PIN_18)
#define key_K2_IOMUX                                             (IOMUX_PINCM44)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PID_TIM_init(void);
void SYSCFG_DL_Pathfinding_TIM_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_Gyro_init(void);
void SYSCFG_DL_HC_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
