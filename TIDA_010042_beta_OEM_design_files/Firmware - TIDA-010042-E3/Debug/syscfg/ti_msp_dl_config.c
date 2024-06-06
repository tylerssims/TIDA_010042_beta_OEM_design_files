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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G150X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_1Backup;
DL_TimerA_backupConfig gTIMER_0Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_1_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_TIMER_1_init();
    SYSCFG_DL_ADC12_0_init();
    SYSCFG_DL_ADC12_1_init();
    SYSCFG_DL_COMP_0_init();
    SYSCFG_DL_OPA_0_init();
    SYSCFG_DL_OPA_1_init();
    /* Ensure backup structures have no valid state */
	gPWM_1Backup.backupRdy 	= false;
	gTIMER_0Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_1_INST, &gPWM_1Backup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_0_INST, &gTIMER_0Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_1_INST, &gPWM_1Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_0_INST, &gTIMER_0Backup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_1_INST);
    DL_TimerA_reset(TIMER_0_INST);
    DL_TimerG_reset(TIMER_1_INST);
    DL_ADC12_reset(ADC12_0_INST);
    DL_ADC12_reset(ADC12_1_INST);
    DL_COMP_reset(COMP_0_INST);
    DL_OPA_reset(OPA_0_INST);
    DL_OPA_reset(OPA_1_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_1_INST);
    DL_TimerA_enablePower(TIMER_0_INST);
    DL_TimerG_enablePower(TIMER_1_INST);
    DL_ADC12_enablePower(ADC12_0_INST);
    DL_ADC12_enablePower(ADC12_1_INST);
    DL_COMP_enablePower(COMP_0_INST);
    DL_OPA_enablePower(OPA_0_INST);
    DL_OPA_enablePower(OPA_1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    const uint8_t unusedPinIndexes[] =
    {
        IOMUX_PINCM1, IOMUX_PINCM2, IOMUX_PINCM3, IOMUX_PINCM7,
        IOMUX_PINCM8, IOMUX_PINCM20, IOMUX_PINCM21, IOMUX_PINCM22,
        IOMUX_PINCM23, IOMUX_PINCM24, IOMUX_PINCM25, IOMUX_PINCM26,
        IOMUX_PINCM31, IOMUX_PINCM34, IOMUX_PINCM37, IOMUX_PINCM46,
        IOMUX_PINCM48, IOMUX_PINCM52, IOMUX_PINCM53
    };

    for(int i = 0; i < sizeof(unusedPinIndexes)/sizeof(unusedPinIndexes[0]); i++)
    {
        DL_GPIO_initDigitalOutput(unusedPinIndexes[i]);
    }

    DL_GPIO_clearPins(GPIOA,
        (DL_GPIO_PIN_0 | DL_GPIO_PIN_1 | DL_GPIO_PIN_28 | DL_GPIO_PIN_2 |
        DL_GPIO_PIN_3 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11 |
        DL_GPIO_PIN_12 | DL_GPIO_PIN_15 | DL_GPIO_PIN_21 | DL_GPIO_PIN_23));
    DL_GPIO_enableOutput(GPIOA,
        (DL_GPIO_PIN_0 | DL_GPIO_PIN_1 | DL_GPIO_PIN_28 | DL_GPIO_PIN_2 |
        DL_GPIO_PIN_3 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11 |
        DL_GPIO_PIN_12 | DL_GPIO_PIN_15 | DL_GPIO_PIN_21 | DL_GPIO_PIN_23));
    DL_GPIO_clearPins(GPIOB,
        (DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9 |
        DL_GPIO_PIN_14 | DL_GPIO_PIN_20 | DL_GPIO_PIN_24));
    DL_GPIO_enableOutput(GPIOB,
        (DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9 |
        DL_GPIO_PIN_14 | DL_GPIO_PIN_20 | DL_GPIO_PIN_24));

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_1_C2_IOMUX,GPIO_PWM_1_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C2_PORT, GPIO_PWM_1_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_1_C2_CMPL_IOMUX,GPIO_PWM_1_C2_CMPL_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C2_CMPL_PORT, GPIO_PWM_1_C2_CMPL_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_1_C3_IOMUX,GPIO_PWM_1_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C3_PORT, GPIO_PWM_1_C3_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_1_C3_CMPL_IOMUX,GPIO_PWM_1_C3_CMPL_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C3_CMPL_PORT, GPIO_PWM_1_C3_CMPL_PIN);

    DL_GPIO_initDigitalInput(GPIO_TEMP_PIN_3_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_PIN_0_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_PIN_4_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_5_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_6_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_7_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_2_LED2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRP_2_LED3_IOMUX);

    DL_GPIO_configSubscriber(GPIOA, DL_GPIO_SUBSCRIBER_INDEX_1,
        DL_GPIO_SUBSCRIBER_OUT_POLICY_TOGGLE,
        DL_GPIO_SUBSCRIBER1_PIN_26);
    DL_GPIO_setSubscriberChanID(GPIOA, DL_GPIO_SUBSCRIBER_INDEX_1, GPIOA_EVENT_SUBSCRIBER_1_CHANNEL);
    DL_GPIO_enableSubscriber(GPIOA, DL_GPIO_SUBSCRIBER_INDEX_1);
    DL_GPIO_clearPins(GPIOA, GPIO_GRP_1_PIN_1_PIN |
		GPIO_GRP_1_PIN_2_PIN |
		GPIO_GRP_1_PIN_5_PIN |
		GPIO_GRP_1_PIN_6_PIN |
		GPIO_GRP_1_PIN_7_PIN |
		GPIO_GRP_2_LED2_PIN |
		GPIO_GRP_2_LED3_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRP_1_PIN_1_PIN |
		GPIO_GRP_1_PIN_2_PIN |
		GPIO_GRP_1_PIN_5_PIN |
		GPIO_GRP_1_PIN_6_PIN |
		GPIO_GRP_1_PIN_7_PIN |
		GPIO_GRP_2_LED2_PIN |
		GPIO_GRP_2_LED3_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_GRP_0_PIN_4_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_GRP_0_PIN_0_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_GRP_0_PIN_0_PIN |
		GPIO_GRP_0_PIN_4_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_2_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_TEMP_PIN_3_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_TEMP_PIN_3_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);

    /* Check that SYSPLL is disabled before configuration */
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_SYSPLL_OFF))
           != (DL_SYSCTL_CLK_STATUS_SYSPLL_OFF))
        {
            ;
        }
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);



    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

}


/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_1ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_1Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 320,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_1_init(void) {

    DL_TimerA_setClockConfig(
        PWM_1_INST, (DL_TimerA_ClockConfig *) &gPWM_1ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_1_INST, (DL_TimerA_PWMConfig *) &gPWM_1Config);

    DL_TimerA_setCaptureCompareValue(PWM_1_INST, 320, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(PWM_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_DEAD_BAND,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptureCompareValue(PWM_1_INST, 320, DL_TIMER_CC_3_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(PWM_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_DEAD_BAND,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setDeadBand(PWM_1_INST, 2, 2, DL_TIMER_DEAD_BAND_MODE_0);
    DL_TimerA_enableClock(PWM_1_INST);


    DL_TimerA_enableInterrupt(PWM_1_INST , DL_TIMER_INTERRUPT_ZERO_EVENT);

    DL_TimerA_setCCPDirection(PWM_1_INST , DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT );

}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (32us * 80000000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerA_setClockConfig(TIMER_0_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerA_initTimerMode(TIMER_0_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerA_enableClock(TIMER_0_INST);

    DL_TimerA_enableEvent(TIMER_0_INST, DL_TIMER_EVENT_ROUTE_1, (DL_TIMER_EVENT_ZERO_EVENT));

    DL_TimerA_setPublisherChanID(TIMER_0_INST, DL_TIMER_PUBLISHER_INDEX_0, TIMER_0_INST_PUB_0_CH);

    DL_TimerA_enableEvent(TIMER_0_INST, DL_TIMER_EVENT_ROUTE_2, (DL_TIMER_EVENT_ZERO_EVENT));

    DL_TimerA_setPublisherChanID(TIMER_0_INST, DL_TIMER_PUBLISHER_INDEX_1, TIMER_0_INST_PUB_1_CH);


}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (5000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   500000 Hz = 5000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_1ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 9U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_1_INST_LOAD_VALUE = (2ms * 500000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_1TimerConfig = {
    .period     = TIMER_1_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_1_init(void) {

    DL_TimerG_setClockConfig(TIMER_1_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_1ClockConfig);

    DL_TimerG_initTimerMode(TIMER_1_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_1TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_1_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_1_INST);




}


/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void)
{
    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_initSingleSample(ADC12_0_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_13, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC12_0_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC12_0_INST,6);
    DL_ADC12_setSampleTime1(ADC12_0_INST,0);
    DL_ADC12_setSubscriberChanID(ADC12_0_INST,ADC12_0_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,(DL_ADC12_INTERRUPT_INIFG
		 | DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED
		 | DL_ADC12_INTERRUPT_WINDOW_COMP_HIGH
		 | DL_ADC12_INTERRUPT_WINDOW_COMP_LOW));
    DL_ADC12_enableInterrupt(ADC12_0_INST,(DL_ADC12_INTERRUPT_INIFG
		 | DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED
		 | DL_ADC12_INTERRUPT_WINDOW_COMP_HIGH
		 | DL_ADC12_INTERRUPT_WINDOW_COMP_LOW));
    DL_ADC12_enableConversions(ADC12_0_INST);
}
/* ADC12_1 Initialization */
static const DL_ADC12_ClockConfig gADC12_1ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_1_init(void)
{
    DL_ADC12_setClockConfig(ADC12_1_INST, (DL_ADC12_ClockConfig *) &gADC12_1ClockConfig);

    DL_ADC12_initSeqSample(ADC12_1_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_03, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_1_INST, ADC12_1_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_13, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC12_1_INST, ADC12_1_ADCMEM_1,
        DL_ADC12_INPUT_CHAN_6, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC12_1_INST, ADC12_1_ADCMEM_2,
        DL_ADC12_INPUT_CHAN_5, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC12_1_INST, ADC12_1_ADCMEM_3,
        DL_ADC12_INPUT_CHAN_4, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC12_1_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC12_1_INST,0);
    DL_ADC12_setSampleTime1(ADC12_1_INST,6);
    DL_ADC12_setSubscriberChanID(ADC12_1_INST,ADC12_1_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_1_INST,(DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED));
    DL_ADC12_enableInterrupt(ADC12_1_INST,(DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC12_1_INST);
}

/* COMP_0 Initialization */
static const DL_COMP_Config gCOMP_0Config = {
    .channelEnable = DL_COMP_ENABLE_CHANNEL_NEG,
    .mode          = DL_COMP_MODE_FAST,
    .negChannel    = DL_COMP_IMSEL_CHANNEL_6,
    .posChannel    = DL_COMP_IPSEL_CHANNEL_0,
    .hysteresis    = DL_COMP_HYSTERESIS_10,
    .polarity      = DL_COMP_POLARITY_NON_INV
};
static const DL_COMP_RefVoltageConfig gCOMP_0VRefConfig = {
    .mode           = DL_COMP_REF_MODE_STATIC,
    .source         = DL_COMP_REF_SOURCE_VDDA_DAC,
    .terminalSelect = DL_COMP_REF_TERMINAL_SELECT_POS,
    .controlSelect  = DL_COMP_DAC_CONTROL_SW,
    .inputSelect    = DL_COMP_DAC_INPUT_DACCODE0
};

SYSCONFIG_WEAK void SYSCFG_DL_COMP_0_init(void)
{
    DL_COMP_init(COMP_0_INST, (DL_COMP_Config *) &gCOMP_0Config);
    DL_COMP_enableOutputFilter(COMP_0_INST,DL_COMP_FILTER_DELAY_1200);
    DL_COMP_refVoltageInit(COMP_0_INST, (DL_COMP_RefVoltageConfig *) &gCOMP_0VRefConfig);
    DL_COMP_setDACCode0(COMP_0_INST, COMP_0_DACCODE0);
    DL_COMP_enableEvent(COMP_0_INST, (DL_COMP_EVENT_OUTPUT_EDGE
		 | DL_COMP_EVENT_OUTPUT_EDGE_INV ));
    DL_COMP_setPublisherChanID(COMP_0_INST, COMP_0_INST_PUB_CH);
    DL_COMP_enableInterrupt(COMP_0_INST, (DL_COMP_INTERRUPT_OUTPUT_EDGE
		 | DL_COMP_INTERRUPT_OUTPUT_EDGE_INV));

    DL_COMP_enable(COMP_0_INST);

}


static const DL_OPA_Config gOPA_0Config0 = {
    .pselChannel    = DL_OPA_PSEL_IN1_POS,
    .nselChannel    = DL_OPA_NSEL_IN1_NEG,
    .mselChannel    = DL_OPA_MSEL_OPEN,
    .gain           = DL_OPA_GAIN_N0_P1,
    .outputPinState = DL_OPA_OUTPUT_PIN_ENABLED,
    .choppingMode   = DL_OPA_CHOPPING_MODE_STANDARD,
};

SYSCONFIG_WEAK void SYSCFG_DL_OPA_0_init(void)
{
    DL_OPA_init(OPA_0_INST, (DL_OPA_Config *) &gOPA_0Config0);
    DL_OPA_enableRailToRailInput(OPA_0_INST);
    DL_OPA_setGainBandwidth(OPA_0_INST, DL_OPA_GBW_HIGH);

    DL_OPA_enable(OPA_0_INST);
}
static const DL_OPA_Config gOPA_1Config0 = {
    .pselChannel    = DL_OPA_PSEL_IN1_POS,
    .nselChannel    = DL_OPA_NSEL_IN1_NEG,
    .mselChannel    = DL_OPA_MSEL_OPEN,
    .gain           = DL_OPA_GAIN_N0_P1,
    .outputPinState = DL_OPA_OUTPUT_PIN_ENABLED,
    .choppingMode   = DL_OPA_CHOPPING_MODE_STANDARD,
};

SYSCONFIG_WEAK void SYSCFG_DL_OPA_1_init(void)
{
    DL_OPA_init(OPA_1_INST, (DL_OPA_Config *) &gOPA_1Config0);
    DL_OPA_enableRailToRailInput(OPA_1_INST);
    DL_OPA_setGainBandwidth(OPA_1_INST, DL_OPA_GBW_HIGH);

    DL_OPA_enable(OPA_1_INST);
}

