/*
 * Copyright (c) 2021, Texas Instruments Incorporated - http://www.ti.com
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
 *  DO NOT EDIT - This file is generated for the MSPM0G150X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G150X

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



/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA0
#define PWM_1_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 2 */
#define GPIO_PWM_1_C2_PORT                                                 GPIOA
#define GPIO_PWM_1_C2_PIN                                          DL_GPIO_PIN_7
#define GPIO_PWM_1_C2_IOMUX                                      (IOMUX_PINCM14)
#define GPIO_PWM_1_C2_IOMUX_FUNC                     IOMUX_PINCM14_PF_TIMA0_CCP2
#define GPIO_PWM_1_C2_IDX                                    DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_1_C2_CMPL_PORT                                            GPIOA
#define GPIO_PWM_1_C2_CMPL_PIN                                     DL_GPIO_PIN_6
#define GPIO_PWM_1_C2_CMPL_IOMUX                                 (IOMUX_PINCM11)
#define GPIO_PWM_1_C2_CMPL_IOMUX_FUNC           IOMUX_PINCM11_PF_TIMA0_CCP2_CMPL

/* GPIO defines for channel 3 */
#define GPIO_PWM_1_C3_PORT                                                 GPIOA
#define GPIO_PWM_1_C3_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_1_C3_IOMUX                                       (IOMUX_PINCM9)
#define GPIO_PWM_1_C3_IOMUX_FUNC                      IOMUX_PINCM9_PF_TIMA0_CCP3
#define GPIO_PWM_1_C3_IDX                                    DL_TIMER_CC_3_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_1_C3_CMPL_PORT                                            GPIOB
#define GPIO_PWM_1_C3_CMPL_PIN                                     DL_GPIO_PIN_3
#define GPIO_PWM_1_C3_CMPL_IOMUX                                 (IOMUX_PINCM16)
#define GPIO_PWM_1_C3_CMPL_IOMUX_FUNC           IOMUX_PINCM16_PF_TIMA0_CCP3_CMPL



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (2559U)
#define TIMER_0_INST_PUB_0_CH                                                (1)
#define TIMER_0_INST_PUB_1_CH                                                (2)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMG0)
#define TIMER_1_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                           (999U)




/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_0_REF_VOLTAGE                                          -1 // VDDA cannot be determined
#define ADC12_0_INST_SUB_CH                                                  (1)

/* Defines for ADC12_1 */
#define ADC12_1_INST                                                        ADC1
#define ADC12_1_INST_IRQHandler                                  ADC1_IRQHandler
#define ADC12_1_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define ADC12_1_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_1_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_1_ADCMEM_0_REF_VOLTAGE                                          -1 // VDDA cannot be determined
#define ADC12_1_ADCMEM_1                                      DL_ADC12_MEM_IDX_1
#define ADC12_1_ADCMEM_1_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_1_ADCMEM_1_REF_VOLTAGE                                          -1 // VDDA cannot be determined
#define ADC12_1_ADCMEM_2                                      DL_ADC12_MEM_IDX_2
#define ADC12_1_ADCMEM_2_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_1_ADCMEM_2_REF_VOLTAGE                                          -1 // VDDA cannot be determined
#define ADC12_1_ADCMEM_3                                      DL_ADC12_MEM_IDX_3
#define ADC12_1_ADCMEM_3_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_1_ADCMEM_3_REF_VOLTAGE                                          -1 // VDDA cannot be determined
#define ADC12_1_INST_SUB_CH                                                  (2)
#define GPIO_ADC12_1_C6_PORT                                               GPIOB
#define GPIO_ADC12_1_C6_PIN                                       DL_GPIO_PIN_19
#define GPIO_ADC12_1_C5_PORT                                               GPIOB
#define GPIO_ADC12_1_C5_PIN                                       DL_GPIO_PIN_18
#define GPIO_ADC12_1_C4_PORT                                               GPIOB
#define GPIO_ADC12_1_C4_PIN                                       DL_GPIO_PIN_17



/* Defines for COMP_0 */
#define COMP_0_INST                                                        COMP0
#define COMP_0_INST_INT_IRQN                                      COMP0_INT_IRQn

/* Defines for COMP_0 DACCODE0 */
#define COMP_0_DACCODE0                                                    (191)

#define COMP_0_INST_PUB_CH                                                   (3)

/* GPIO configuration for COMP_0 */


/* Defines for OPA_0 */
#define OPA_0_INST                                                          OPA0
#define GPIO_OPA_0_IN1POS_PORT                                             GPIOA
#define GPIO_OPA_0_IN1POS_PIN                                     DL_GPIO_PIN_25
#define GPIO_OPA_0_IOMUX_IN1POS                                  (IOMUX_PINCM55)
#define GPIO_OPA_0_IOMUX_IN1POS_FUNC                IOMUX_PINCM55_PF_UNCONNECTED
#define GPIO_OPA_0_IN1NEG_PORT                                             GPIOA
#define GPIO_OPA_0_IN1NEG_PIN                                     DL_GPIO_PIN_24
#define GPIO_OPA_0_IOMUX_IN1NEG                                  (IOMUX_PINCM54)
#define GPIO_OPA_0_IOMUX_IN1NEG_FUNC                IOMUX_PINCM54_PF_UNCONNECTED
#define GPIO_OPA_0_OUT_PORT                                                GPIOA
#define GPIO_OPA_0_OUT_PIN                                        DL_GPIO_PIN_22
#define GPIO_OPA_0_IOMUX_OUT                                     (IOMUX_PINCM47)
#define GPIO_OPA_0_IOMUX_OUT_FUNC                   IOMUX_PINCM47_PF_UNCONNECTED
/* Defines for OPA_1 */
#define OPA_1_INST                                                          OPA1
#define GPIO_OPA_1_IN1POS_PORT                                             GPIOA
#define GPIO_OPA_1_IN1POS_PIN                                     DL_GPIO_PIN_18
#define GPIO_OPA_1_IOMUX_IN1POS                                  (IOMUX_PINCM40)
#define GPIO_OPA_1_IOMUX_IN1POS_FUNC                IOMUX_PINCM40_PF_UNCONNECTED
#define GPIO_OPA_1_IN1NEG_PORT                                             GPIOA
#define GPIO_OPA_1_IN1NEG_PIN                                     DL_GPIO_PIN_17
#define GPIO_OPA_1_IOMUX_IN1NEG                                  (IOMUX_PINCM39)
#define GPIO_OPA_1_IOMUX_IN1NEG_FUNC                IOMUX_PINCM39_PF_UNCONNECTED
#define GPIO_OPA_1_OUT_PORT                                                GPIOA
#define GPIO_OPA_1_OUT_PIN                                        DL_GPIO_PIN_16
#define GPIO_OPA_1_IOMUX_OUT                                     (IOMUX_PINCM38)
#define GPIO_OPA_1_IOMUX_OUT_FUNC                   IOMUX_PINCM38_PF_UNCONNECTED



/* Port definition for Pin Group GPIO_TEMP */
#define GPIO_TEMP_PORT                                                   (GPIOB)

/* Defines for PIN_3: GPIOB.2 with pinCMx 15 on package pin 14 */
// pins affected by this interrupt request:["PIN_3"]
#define GPIO_TEMP_INT_IRQN                                      (GPIOB_INT_IRQn)
#define GPIO_TEMP_INT_IIDX                      (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_TEMP_PIN_3_IIDX                                 (DL_GPIO_IIDX_DIO2)
#define GPIO_TEMP_PIN_3_PIN                                      (DL_GPIO_PIN_2)
#define GPIO_TEMP_PIN_3_IOMUX                                    (IOMUX_PINCM15)
/* Port definition for Pin Group GPIO_GRP_0 */
#define GPIO_GRP_0_PORT                                                  (GPIOB)

/* Defines for PIN_0: GPIOB.16 with pinCMx 33 on package pin 26 */
#define GPIO_GRP_0_PIN_0_PIN                                    (DL_GPIO_PIN_16)
#define GPIO_GRP_0_PIN_0_IOMUX                                   (IOMUX_PINCM33)
/* Defines for PIN_4: GPIOB.15 with pinCMx 32 on package pin 25 */
#define GPIO_GRP_0_PIN_4_PIN                                    (DL_GPIO_PIN_15)
#define GPIO_GRP_0_PIN_4_IOMUX                                   (IOMUX_PINCM32)
/* Port definition for Pin Group GPIO_GRP_1 */
#define GPIO_GRP_1_PORT                                                  (GPIOA)

/* Defines for PIN_1: GPIOA.26 with pinCMx 59 on package pin 46 */
#define GPIO_GRP_1_PIN_1_PIN                                    (DL_GPIO_PIN_26)
#define GPIO_GRP_1_PIN_1_IOMUX                                   (IOMUX_PINCM59)
/* Defines for PIN_2: GPIOA.27 with pinCMx 60 on package pin 47 */
#define GPIO_GRP_1_PIN_2_PIN                                    (DL_GPIO_PIN_27)
#define GPIO_GRP_1_PIN_2_IOMUX                                   (IOMUX_PINCM60)
/* Defines for PIN_5: GPIOA.31 with pinCMx 6 on package pin 5 */
#define GPIO_GRP_1_PIN_5_PIN                                    (DL_GPIO_PIN_31)
#define GPIO_GRP_1_PIN_5_IOMUX                                    (IOMUX_PINCM6)
/* Defines for PIN_6: GPIOA.5 with pinCMx 10 on package pin 11 */
#define GPIO_GRP_1_PIN_6_PIN                                     (DL_GPIO_PIN_5)
#define GPIO_GRP_1_PIN_6_IOMUX                                   (IOMUX_PINCM10)
/* Defines for PIN_7: GPIOA.8 with pinCMx 19 on package pin 16 */
#define GPIO_GRP_1_PIN_7_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_GRP_1_PIN_7_IOMUX                                   (IOMUX_PINCM19)
/* Port definition for Pin Group GPIO_GRP_2 */
#define GPIO_GRP_2_PORT                                                  (GPIOA)

/* Defines for LED2: GPIOA.13 with pinCMx 35 on package pin 28 */
#define GPIO_GRP_2_LED2_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_GRP_2_LED2_IOMUX                                    (IOMUX_PINCM35)
/* Defines for LED3: GPIOA.14 with pinCMx 36 on package pin 29 */
#define GPIO_GRP_2_LED3_PIN                                     (DL_GPIO_PIN_14)
#define GPIO_GRP_2_LED3_IOMUX                                    (IOMUX_PINCM36)
#define GPIOA_EVENT_SUBSCRIBER_1_CHANNEL                                     (3)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_ADC12_1_init(void);
void SYSCFG_DL_COMP_0_init(void);
void SYSCFG_DL_OPA_0_init(void);
void SYSCFG_DL_OPA_1_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
