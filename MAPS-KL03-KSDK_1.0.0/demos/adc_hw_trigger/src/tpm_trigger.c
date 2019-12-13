/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "adc_hw_trigger.h"
#include "fsl_tpm_driver.h"
#include "fsl_gpio_hal.h"
#include "fsl_port_hal.h"

extern const uint32_t gSimBaseAddr[];
static tpm_state_t gTpmState;

/*!
 * @Brief enable the trigger source of LPTimer
 */
void init_trigger_source(uint32_t adcInstance)
{
    uint32_t freqUs, freqTpm;
    tpm_general_config_t tpmConfig =
    {
        .isDBGMode = true,
        .isGlobalTimeBase = false,
        .isTriggerMode = false,
        .triggerSource = kTpmTpm0Trig,
        .isCountReloadOnTrig = true, /* reload on overflow */
    };

    /* Get the required period */
    freqUs = 1000000U/(INPUT_SIGNAL_FREQ*NR_SAMPLES)*2;

    /* init the TPM driver */
    TPM_DRV_Init(0, &tpmConfig, &gTpmState);
    /* set clock source to be IRC */
    TPM_DRV_SetClock(0, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy8);
    /* retrive the final TPM counting clock freq */
    freqTpm = TPM_DRV_GetClock(0);
    /* start TPM counter */
    freqTpm *= freqUs; /* convert to KHz */
    freqTpm /= 1000 * 1000; /* convert to ms */
    TPM_DRV_CounterStart(0, kTpmCountingUp, freqTpm, false);

    /* Configure SIM for ADC hw trigger source selection */
    SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBaseAddr[0], adcInstance, true);
    SIM_HAL_SetAdcPreTriggerMode(gSimBaseAddr[0], adcInstance, kSimAdcPretrgselA);
    SIM_HAL_SetAdcTriggerMode(gSimBaseAddr[0], adcInstance, kSimAdcTrgSelTpm0);
}

/*!
 * @Brief disable the trigger source
 */
void deinit_trigger_source(uint32_t adcInstance)
{
    TPM_DRV_CounterStop(0);
    TPM_DRV_Deinit(0);
}
