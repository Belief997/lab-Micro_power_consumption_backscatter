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

#include "fsl_tpm_driver.h"
#include "fsl_tpm_common.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! Stores TPM clock source setting */
static tpm_clock_mode_t s_tpmClockSource = kTpmClockSourceNoneClk;
/*! Stores the external clock frequency incase TPM module uses the external clock */
static uint32_t s_tpmExtClock;
static tpm_state_t *volatile g_tpmStatePtr[HW_TPM_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*See fsl_tpm_driver.h for documentation of this function.*/
void TPM_DRV_Init(uint8_t instance, tpm_general_config_t * info, tpm_state_t *userStatePtr)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    /*Enable TPM clock*/
    CLOCK_SYS_EnableTpmClock(instance);

    TPM_HAL_Reset(tpmBaseAddr, instance);

    /*trigger mode*/
    TPM_HAL_SetTriggerMode(tpmBaseAddr, info->isTriggerMode);
    TPM_HAL_SetStopOnOverflowMode(tpmBaseAddr, info->isStopCountOnOveflow);
    TPM_HAL_SetReloadOnTriggerMode(tpmBaseAddr, info->isCountReloadOnTrig);

    /*trigger source*/
    TPM_HAL_SetTriggerSrc(tpmBaseAddr, info->triggerSource);

    /*global time base*/
    TPM_HAL_EnableGlobalTimeBase(tpmBaseAddr, info->isGlobalTimeBase);

    /*Debug mode*/
    TPM_HAL_SetDbgMode(tpmBaseAddr, info->isDBGMode);

    /* Internal context */
    g_tpmStatePtr[instance] = userStatePtr;

    userStatePtr->userCallbackFunc = NULL;
    
    NVIC_ClearPendingIRQ(g_tpmIrqId[instance]);
    INT_SYS_EnableIRQ(g_tpmIrqId[instance]);
}

/*see fsl_tpm_hal.h for documentation of this function*/
void TPM_DRV_SetClock(uint8_t instance, tpm_clock_source_t clock, tpm_clock_ps_t clockPs)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    /*Clock prescaler*/
    TPM_HAL_SetClockDiv(tpmBaseAddr, clockPs);

    if (clock == kTpmClockSourcNone)
    {
        s_tpmClockSource = kTpmClockSourceNoneClk;
    }
    else if ((clock == kTpmClockSourceModuleDisable) || (clock == kTpmClockSourceModuleOSCERCLK) ||
             (clock == kTpmClockSourceModuleHighFreq) || (clock == kTpmClockSourceModuleMCGIRCLK))
    {
        CLOCK_HAL_SetSource(g_simBaseAddr[0], kClockTpmSrc, clock - 1);
        s_tpmClockSource = kTpmClockSourceModuleClk;
    }
    else if ((clock == kTpmClockSourceExternalCLKIN0) || (clock == kTpmClockSourceExternalCLKIN1))

    {
        SIM_HAL_SetTpmExternalClkPinSelMode(g_simBaseAddr[0], instance, (sim_tpm_clk_sel_t)(clock - 5));
        s_tpmClockSource = kTpmClockSourceExternalClk;
    }
    else
    {
        s_tpmClockSource = kTpmClockSourceReservedClk;
    }
}

void TPM_DRV_SetExtClockFreq(uint32_t freq)
{
    s_tpmExtClock = freq;
}

uint32_t TPM_DRV_GetClock(uint8_t instance)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];
    uint32_t freq = 0;
    uint32_t clockPs;

    /* Clock prescaler */
    clockPs = (1 << TPM_HAL_GetClockDiv(tpmBaseAddr));

    switch (s_tpmClockSource)
    {
        case kTpmClockSourceModuleClk:
            freq = CLOCK_SYS_GetTpmFreq(0) / clockPs;
            break;
        case kTpmClockSourceExternalClk:
            freq = s_tpmExtClock / clockPs;
            break;
        default:
            break;
    }

    return freq;
}

void TPM_DRV_SetTimeOverflowIntCmd(uint32_t instance, bool overflowEnable)
{
    if (overflowEnable)
    {
        TPM_HAL_EnableTimerOverflowInt(g_tpmBaseAddr[instance]);
    }
    else
    {
        TPM_HAL_DisableTimerOverflowInt(g_tpmBaseAddr[instance]);
    }
}

void TPM_DRV_SetChnIntCmd(uint32_t instance, uint8_t channelNum, bool enable)
{
    if (enable)
    {
        TPM_HAL_EnableChnInt(g_tpmBaseAddr[instance], channelNum);
    }
    else
    {
        TPM_HAL_DisableChnInt(g_tpmBaseAddr[instance], channelNum);
    }
}

/*See fsl_tpm_driver.h for documentation of this function.*/
void TPM_DRV_Deinit(uint8_t instance)
{
    TPM_HAL_Reset(g_tpmBaseAddr[instance], instance);
    
    /* Cleared state pointer */
    g_tpmStatePtr[instance] = NULL;

    INT_SYS_DisableIRQ(g_tpmIrqId[instance]);

    /* disable clock for TPM.*/
    CLOCK_SYS_DisableTpmClock(instance);
}

/*See fsl_tpm_driver.h for documentation of this function.*/
void TPM_DRV_PwmStop(uint8_t instance, tpm_pwm_param_t *param, uint8_t channel)
{
    assert((param->mode == kTpmEdgeAlignedPWM) || (param->mode == kTpmCenterAlignedPWM));
    assert(instance < HW_TPM_INSTANCE_COUNT);
    assert(channel < FSL_FEATURE_TPM_CHANNEL_COUNTn(instance));

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    /* Set clock source to none to disable counter */
    TPM_HAL_SetClockMode(tpmBaseAddr, kTpmClockSourceNoneClk);

    TPM_HAL_DisableChn(tpmBaseAddr, channel);
}

/*See fsl_tpm_driver.h for documentation of this function.*/
bool TPM_DRV_PwmStart(uint8_t instance, tpm_pwm_param_t *param, uint8_t channel)
{
    uint32_t freq;
    uint16_t uMod, uCnv;

    assert(instance < HW_TPM_INSTANCE_COUNT);
    assert(param->uDutyCyclePercent <= 100);
    assert(channel < FSL_FEATURE_TPM_CHANNEL_COUNTn(instance));

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    if (s_tpmClockSource == kTpmClockSourceNoneClk)
    {
        return false;
    }

    freq = TPM_DRV_GetClock(instance);

    /* When switching mode, disable channel first  */
    TPM_HAL_DisableChn(tpmBaseAddr, channel);

    /* Set the requested PWM mode */
    TPM_HAL_EnablePwmMode(tpmBaseAddr, param, channel);

    switch(param->mode)
    {
        case kTpmEdgeAlignedPWM:
            uMod = freq / param->uFrequencyHZ - 1;
            uCnv = uMod * param->uDutyCyclePercent / 100;
            /* For 100% duty cycle */
            if(uCnv >= uMod)
            {
                uCnv = uMod + 1;
            }
            TPM_HAL_SetMod(tpmBaseAddr, uMod);
            TPM_HAL_SetChnCountVal(tpmBaseAddr, channel, uCnv);
            break;
        case kTpmCenterAlignedPWM:
            uMod = freq / (param->uFrequencyHZ * 2);
            uCnv = uMod * param->uDutyCyclePercent / 100;
            /* For 100% duty cycle */
            if(uCnv >= uMod)
            {
                uCnv = uMod + 1;
            }
            TPM_HAL_SetMod(tpmBaseAddr, uMod);
            TPM_HAL_SetChnCountVal(tpmBaseAddr, channel, uCnv);
            break;
        default:
            assert(0);
            break;
    }

    /* Set the TPM clock */
    TPM_HAL_SetClockMode(tpmBaseAddr, s_tpmClockSource);

    return true;
}

void TPM_DRV_CounterStart(uint8_t instance, tpm_counting_mode_t countMode, uint32_t countFinalVal,
                                 bool enableOverflowInt)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];
    uint32_t channel = 0;

    /* Set clock source to none to disable counter */
    TPM_HAL_SetClockMode(g_tpmBaseAddr[instance], kTpmClockSourceNoneClk);

    /* Clear the overflow flag */
    TPM_HAL_ClearTimerOverflowFlag(tpmBaseAddr);
    TPM_HAL_SetMod(tpmBaseAddr, countFinalVal);
    TPM_HAL_ClearCounter(tpmBaseAddr);

    /* Use TPM as counter, turn off all the channels */
    for (channel = 0; channel < FSL_FEATURE_TPM_CHANNEL_COUNTn(instance); channel++)
    {
        TPM_HAL_DisableChn(tpmBaseAddr, channel);
    }

    if (countMode == kTpmCountingUp)
    {
        TPM_HAL_SetCpwms(tpmBaseAddr, 0);
    }
    else if (countMode == kTpmCountingUpDown)
    {
        TPM_HAL_SetCpwms(tpmBaseAddr, 1);
    }

    /* Activate interrupts if required */
    TPM_DRV_SetTimeOverflowIntCmd(instance, enableOverflowInt);

    /* Set the TPM clock */
    TPM_HAL_SetClockMode(tpmBaseAddr, s_tpmClockSource);
}

void TPM_DRV_CounterStop(uint8_t instance)
{
    TPM_HAL_SetCpwms(g_tpmBaseAddr[instance], 0);

    /* Set clock source to none to disable counter */
    TPM_HAL_SetClockMode(g_tpmBaseAddr[instance], kTpmClockSourceNoneClk);

    /* Disable the overflow interrupt */
    TPM_DRV_SetTimeOverflowIntCmd(instance, false);
}

uint32_t TPM_DRV_CounterRead(uint8_t instance)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    return TPM_HAL_GetCounterVal(g_tpmBaseAddr[instance]);
}

void TPM_DRV_InputCaptureEnable(uint8_t instance, uint8_t channel, tpm_input_capture_mode_t mode,
                                         uint32_t countFinalVal, bool intEnable)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    /* Set clock source to none to disable counter */
    TPM_HAL_SetClockMode(g_tpmBaseAddr[instance], kTpmClockSourceNoneClk);

    TPM_HAL_DisableChn(tpmBaseAddr, channel);
    TPM_HAL_ClearChnInt(tpmBaseAddr, channel);
    TPM_HAL_ClearCounter(tpmBaseAddr);
    TPM_HAL_SetCpwms(tpmBaseAddr, 0);
    TPM_HAL_SetMod(tpmBaseAddr, countFinalVal);
    TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, channel, (mode << BP_TPM_CnSC_ELSA));

    TPM_DRV_SetChnIntCmd(instance, channel, intEnable);

    /* Set the TPM clock */
    TPM_HAL_SetClockMode(tpmBaseAddr, s_tpmClockSource);
}

uint32_t TPM_DRV_GetChnVal(uint8_t instance, uint8_t channel)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    return TPM_HAL_GetChnCountVal(g_tpmBaseAddr[instance], channel);
}

void TPM_DRV_OutputCompareEnable(uint8_t instance, uint8_t channel, tpm_output_compare_mode_t mode,
                                            uint32_t countFinalVal, uint32_t matchVal, bool intEnable)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];
    uint32_t cmpMode = 0;

    /* Set clock source to none to disable counter */
    TPM_HAL_SetClockMode(g_tpmBaseAddr[instance], kTpmClockSourceNoneClk);

    TPM_HAL_DisableChn(tpmBaseAddr, channel);
    TPM_HAL_ClearChnInt(tpmBaseAddr, channel);
    TPM_HAL_ClearCounter(tpmBaseAddr);
    TPM_HAL_SetCpwms(tpmBaseAddr, 0);
    TPM_HAL_SetMod(tpmBaseAddr, countFinalVal);

    if ((mode == kTpmHighPulseOutput) || (mode == kTpmLowPulseOutput))
    {
        cmpMode = ((uint32_t)mode - 3) << BP_TPM_CnSC_ELSA;
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, channel,
                                     ((0x3 << BP_TPM_CnSC_MSA) | cmpMode));
    }
    else
    {
        cmpMode = mode << BP_TPM_CnSC_ELSA;
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, channel,
                                     ((0x1 << BP_TPM_CnSC_MSA) | cmpMode));
    }
    TPM_HAL_SetChnCountVal(tpmBaseAddr, channel, matchVal);

    TPM_DRV_SetChnIntCmd(instance, channel, intEnable);

    /* Set the TPM clock */
    TPM_HAL_SetClockMode(tpmBaseAddr, s_tpmClockSource);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : TPM_DRV_InstallCallback
 * Description   : Install the user-defined callback in TPM module.
 * When an TPM interrupt request is served, the callback will be executed 
 * inside the ISR. 
 *
 *END*************************************************************************/
void TPM_DRV_InstallCallback(uint32_t instance, tpm_callback_t userCallback)
{
    assert(instance < HW_TPM_INSTANCE_COUNT);

    /* Fill callback function into state structure. */
    g_tpmStatePtr[instance]->userCallbackFunc = userCallback;
    
    return;
}

/*!
 * @brief Pass IRQ control to TPM driver.
 *
 * @param info The TPM peripheral instance info
 */
void TPM_DRV_IRQHandler(uint8_t instance)
{
    uint16_t status = 0;
    uint16_t channel;
    uint32_t tpmBaseAddr = g_tpmBaseAddr[instance];

    /* Clear the status flags for the interrupts enabled */
    if (TPM_HAL_IsOverflowIntEnabled(tpmBaseAddr))
    {
        status = (1 << BP_TPM_STATUS_TOF);
    }

    for (channel = 0; channel < FSL_FEATURE_TPM_CHANNEL_COUNTn(instance); channel++)
    {
        if (TPM_HAL_IsChnIntEnabled(tpmBaseAddr, channel))
        {
            status |= (1u << channel);
        }
    }

    if (g_tpmStatePtr[instance])
    {
        if (g_tpmStatePtr[instance]->userCallbackFunc)
        {
            /* Execute user-defined callback function. */
            (*(g_tpmStatePtr[instance]->userCallbackFunc))();
        }
    }
    
    TPM_HAL_ClearStatusReg(tpmBaseAddr, status);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

