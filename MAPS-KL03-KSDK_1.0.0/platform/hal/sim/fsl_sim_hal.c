/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "fsl_sim_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_SetSource
 * Description   : Set clock source setting 
 * This function will set the settings for specified clock source. Each clock 
 * source has its clock selection settings. Refer to reference manual for 
 * details of settings for each clock source. Refer to clock_source_names_t 
 * for clock sources.
 * 
 *END**************************************************************************/
sim_hal_status_t CLOCK_HAL_SetSource(uint32_t baseAddr,
                                     clock_source_names_t clockSource,
                                     uint8_t setting)
{
    sim_hal_status_t status = kSimHalSuccess;
    assert(clockSource < kClockSourceMax);

    switch (clockSource)
    {
    case kClockTpmSrc:                   /* TPMSRC*/
        BW_SIM_SOPT2_TPMSRC(baseAddr, setting);
        break;


    case kClockOsc32kSel:                /* OSC32KSEL*/
        BW_SIM_SOPT1_OSC32KSEL(baseAddr, setting);
        break;

    case kClockOsc32kOut:                /* OSC32KOUT*/
        BW_SIM_SOPT1_OSC32KOUT(baseAddr, setting);
        break;



    case kClockClkoutSel:                /* CLKOUTSEL*/
        BW_SIM_SOPT2_CLKOUTSEL(baseAddr, setting);
        break;

    case kClockRtcClkoutSel:                /* RTCCLKOUTSEL*/
        BW_SIM_SOPT2_RTCCLKOUTSEL(baseAddr, setting);
        break;
    case kClockLpuart0Src:                /* LPUART0SRC */
        BW_SIM_SOPT2_LPUART0SRC(baseAddr, setting);
        break;
    default:
        status = kSimHalNoSuchClockSrc;
        break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_GetSource
 * Description   : Get clock source setting
 * This function will get the settings for specified clock source. Each clock 
 * source has its clock selection settings. Refer to reference manual for 
 * details of settings for each clock source. Refer to clock_source_names_t
 * for clock sources.
 * 
 *END**************************************************************************/
sim_hal_status_t CLOCK_HAL_GetSource(uint32_t baseAddr,
                                     clock_source_names_t clockSource,
                                     uint8_t *setting)
{
    sim_hal_status_t status = kSimHalSuccess;
    assert(clockSource < kClockSourceMax);

    switch (clockSource)
    {










    case kClockTpmSrc:                   /* TPMSRC*/
        *setting = BR_SIM_SOPT2_TPMSRC(baseAddr);
        break;


    case kClockOsc32kSel:                /* OSC32KSEL*/
        *setting = BR_SIM_SOPT1_OSC32KSEL(baseAddr);
        break;

    case kClockOsc32kOut:                /* OSC32KOUT*/
        *setting = BR_SIM_SOPT1_OSC32KOUT(baseAddr);
        break;



    case kClockClkoutSel:                /* CLKOUTSEL */
        *setting = BR_SIM_SOPT2_CLKOUTSEL(baseAddr);
        break;

    case kClockRtcClkoutSel:                /* RTCCLKOUTSEL */
        *setting = BR_SIM_SOPT2_RTCCLKOUTSEL(baseAddr);
        break;
    case kClockLpuart0Src:                /* LPUART0SRC */
        *setting = BR_SIM_SOPT2_LPUART0SRC(baseAddr);
        break;
        
    default:
        status = kSimHalNoSuchClockSrc;
        break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_SetDivider
 * Description   : Set clock divider setting
 * This function will set the setting for specified clock divider. Refer to 
 * reference manual for supported clock divider and value range. Refer to 
 * clock_divider_names_t for dividers.
 * 
 *END**************************************************************************/
sim_hal_status_t CLOCK_HAL_SetDivider(uint32_t baseAddr,
                                      clock_divider_names_t clockDivider, 
                                      uint32_t setting)
{
    sim_hal_status_t status = kSimHalSuccess;
    assert(clockDivider < kClockDividerMax);

    switch (clockDivider)
    {
    case kClockDividerOutdiv1:           /* OUTDIV1*/
        BW_SIM_CLKDIV1_OUTDIV1(baseAddr, setting);
        break;



    case kClockDividerOutdiv4:           /* OUTDIV4*/
        BW_SIM_CLKDIV1_OUTDIV4(baseAddr, setting);
        break;






    case kClockDividerSpecial1:          /* special divider 1   */
        break;

    default:
        status = kSimHalNoSuchDivider;
        break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_SetOutDividers
 * Description   : Set all clock out dividers setting at the same time
 * This function will set the setting for all clock out dividers. Refer to 
 * reference manual for supported clock divider and value range. Refer to 
 * clock_divider_names_t for dividers.
 * 
 *END**************************************************************************/
void CLOCK_HAL_SetOutDividers(uint32_t baseAddr, uint32_t outdiv1, uint32_t outdiv2, 
                                 uint32_t outdiv3, uint32_t outdiv4)
{
    uint32_t clkdiv1 = 0;
    
    clkdiv1 |= BF_SIM_CLKDIV1_OUTDIV1(outdiv1);
    clkdiv1 |= BF_SIM_CLKDIV1_OUTDIV4(outdiv4);
    
    HW_SIM_CLKDIV1_WR(baseAddr, clkdiv1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_GetDivider
 * Description   : Get clock divider setting
 * This function will get the setting for specified clock divider. Refer to 
 * reference manual for supported clock divider and value range. Refer to 
 * clock_divider_names_t for dividers.
 * 
 *END**************************************************************************/
sim_hal_status_t CLOCK_HAL_GetDivider(uint32_t baseAddr,
                                      clock_divider_names_t clockDivider,
                                      uint32_t *setting)
{
    sim_hal_status_t status = kSimHalSuccess;
    assert(clockDivider < kClockDividerMax);

    *setting = 0;

    switch (clockDivider)
    {
    case kClockDividerOutdiv1:           /* OUTDIV1*/
        *setting = BR_SIM_CLKDIV1_OUTDIV1(baseAddr);
        break;



    case kClockDividerOutdiv4:           /* OUTDIV4*/
        *setting = BR_SIM_CLKDIV1_OUTDIV4(baseAddr);
        break;






    case kClockDividerSpecial1:          /* special divider 1    */
        *setting = 1;                   
        break;

    default:
        status = kSimHalNoSuchDivider;
        break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcAlternativeTriggerCmd
 * Description   : Set ADCx alternate trigger enable setting
 * This function will enable/disable alternative conversion triggers for ADCx. 
 * 
 *END**************************************************************************/
void SIM_HAL_SetAdcAlternativeTriggerCmd(uint32_t baseAddr, uint8_t instance, bool enable)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT7_ADC0ALTTRGEN(baseAddr, enable ? 1 : 0);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        BW_SIM_SOPT7_ADC1ALTTRGEN(baseAddr, enable ? 1 : 0);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        BW_SIM_SOPT7_ADC2ALTTRGEN(baseAddr, enable ? 1 : 0);
        break;
    case 3:
        BW_SIM_SOPT7_ADC3ALTTRGEN(baseAddr, enable ? 1 : 0);
        break;
#endif
#endif
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcAlternativeTriggerCmd
 * Description   : Get ADCx alternate trigger enable settingg
 * This function will get ADCx alternate trigger enable setting. 
 * 
 *END**************************************************************************/
bool SIM_HAL_GetAdcAlternativeTriggerCmd(uint32_t baseAddr, uint8_t instance)
{
    bool retValue = false;

    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = BR_SIM_SOPT7_ADC0ALTTRGEN(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        retValue = BR_SIM_SOPT7_ADC1ALTTRGEN(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        retValue = BR_SIM_SOPT7_ADC2ALTTRGEN(baseAddr);
        break;
    case 3:
        retValue = BR_SIM_SOPT7_ADC3ALTTRGEN(baseAddr);
        break;
#endif
#endif
    default:
        retValue = false;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcPreTriggerMode
 * Description   : Set ADCx pre-trigger select setting
 * This function will select the ADCx pre-trigger source when alternative
 * triggers are enabled through ADCxALTTRGEN
 * 
 *END**************************************************************************/
void SIM_HAL_SetAdcPreTriggerMode(uint32_t baseAddr, uint8_t instance, sim_pretrgsel_t select)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT7_ADC0PRETRGSEL(baseAddr, select);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        BW_SIM_SOPT7_ADC1PRETRGSEL(baseAddr, select);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        BW_SIM_SOPT7_ADC2PRETRGSEL(baseAddr, select);
        break;
    case 3:
        BW_SIM_SOPT7_ADC3PRETRGSEL(baseAddr, select);
        break;
#endif
#endif
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcPreTriggerMode
 * Description   : Get ADCx pre-trigger select setting
 * This function will get ADCx pre-trigger select setting.
 * 
 *END**************************************************************************/
sim_pretrgsel_t SIM_HAL_GetAdcPreTriggerMode(uint32_t baseAddr, uint8_t instance)
{
    sim_pretrgsel_t retValue = (sim_pretrgsel_t)0;

    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC0PRETRGSEL(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC1PRETRGSEL(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC2PRETRGSEL(baseAddr);
        break;
    case 3:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC3PRETRGSEL(baseAddr);
        break;
#endif
#endif
    default:
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcTriggerMode
 * Description   : Set ADCx trigger select setting
 * This function will select the ADCx trigger source when alternative triggers
 * are enabled through ADCxALTTRGEN
 * 
 *END**************************************************************************/
void SIM_HAL_SetAdcTriggerMode(uint32_t baseAddr, uint8_t instance, sim_trgsel_t select)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT7_ADC0TRGSEL(baseAddr, select);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        BW_SIM_SOPT7_ADC1TRGSEL(baseAddr, select);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        BW_SIM_SOPT7_ADC2TRGSEL(baseAddr, select);
        break;
    case 3:
        BW_SIM_SOPT7_ADC3TRGSEL(baseAddr, select);
        break;
#endif
#endif
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcTriggerMode
 * Description   : Get ADCx trigger select setting 
 * This function will get ADCx trigger select setting.
 * 
 *END**************************************************************************/
sim_pretrgsel_t SIM_HAL_GetAdcTriggerMode(uint32_t baseAddr, uint8_t instance)
{
    sim_pretrgsel_t retValue =(sim_pretrgsel_t)0;

    assert(instance < HW_ADC_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC0TRGSEL(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 1)
    case 1:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC1TRGSEL(baseAddr);
        break;
#if (HW_ADC_INSTANCE_COUNT > 2)
    case 2:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC2TRGSEL(baseAddr);
        break;
    case 3:
        retValue = (sim_pretrgsel_t)BR_SIM_SOPT7_ADC3TRGSEL(baseAddr);
        break;
#endif
#endif
    default:
        break;
    }

    return retValue;
}


#ifdef HW_LPUART_INSTANCE_COUNT
/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetLpUartRxSrcMode
 * Description   : Set LPUARTx receive data source select setting 
 * This function will select the source for the LPUART1 receive data.
 * 
 *END**************************************************************************/
void SIM_HAL_SetLpUartRxSrcMode(uint32_t baseAddr, uint8_t instance, sim_uart_rxsrc_t select)
{
    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT5_LPUART0RXSRC(baseAddr, select);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetLpUartRxSrcMode
 * Description   : Get LPUARTx receive data source select setting 
 * This function will get LPUARTx receive data source select setting.
 * 
 *END**************************************************************************/
sim_uart_rxsrc_t SIM_HAL_GetLpUartRxSrcMode(uint32_t baseAddr, uint8_t instance)
{
    sim_uart_rxsrc_t retValue = (sim_uart_rxsrc_t)0;

    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = (sim_uart_rxsrc_t)BR_SIM_SOPT5_LPUART0RXSRC(baseAddr);
        break;
    default:
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetUartTxSrcMode
 * Description   : Set LPUARTx transmit data source select setting 
 * This function will select the source for the LPUARTx transmit data.
 * 
 *END**************************************************************************/
void SIM_HAL_SetLpUartTxSrcMode(uint32_t baseAddr, uint8_t instance, sim_uart_txsrc_t select)
{
    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT5_LPUART0TXSRC(baseAddr, select);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetLpUartTxSrcMode
 * Description   : Get LPUARTx transmit data source select setting 
 * This function will get LPUARTx transmit data source select setting.
 * 
 *END**************************************************************************/
sim_uart_txsrc_t SIM_HAL_GetLpUartTxSrcMode(uint32_t baseAddr, uint8_t instance)
{
    sim_uart_txsrc_t retValue =(sim_uart_txsrc_t)0;

    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = (sim_uart_txsrc_t)BR_SIM_SOPT5_LPUART0TXSRC(baseAddr);
        break;
    default:
        break;
    }

    return retValue;
}
#endif


#ifdef HW_LPUART_INSTANCE_COUNT
/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetLpUartOpenDrainCmd
 * Description   : Set LPUARTx Open Drain Enable setting 
 * This function will enable/disable the LPUARTx Open Drain.
 * 
 *END**************************************************************************/
void SIM_HAL_SetLpUartOpenDrainCmd(uint32_t baseAddr, uint8_t instance, bool enable)
{
    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT5_LPUART0ODE(baseAddr, enable ? 1 : 0);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetLpUartOpenDrainCmd
 * Description   : Get LPUARTx Open Drain Enable setting 
 * This function will get LPUARTx Open Drain Enable setting.
 * 
 *END**************************************************************************/
bool SIM_HAL_GetLpUartOpenDrainCmd(uint32_t baseAddr, uint8_t instance)
{
    bool retValue = false;

    assert(instance < HW_LPUART_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = BR_SIM_SOPT5_LPUART0ODE(baseAddr);
        break;
    default:
        break;
    }

    return retValue;
}
#endif


/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetTpmExternalClkPinSelMode
 * Description   : Set Timer/PWM x external clock pin select setting 
 * This function will select the source of Timer/PWM x external clock pin select
 * 
 *END**************************************************************************/
void SIM_HAL_SetTpmExternalClkPinSelMode(uint32_t baseAddr,
                                         uint8_t instance,
                                         sim_tpm_clk_sel_t select)
{
    assert (instance < HW_TPM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        BW_SIM_SOPT4_TPM0CLKSEL(baseAddr, select);
        break;
    case 1:
        BW_SIM_SOPT4_TPM1CLKSEL(baseAddr, select);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetTpmExternalClkPinSelMode
 * Description   : Get Timer/PWM x external clock pin select setting
 * This function will get Timer/PWM x external clock pin select setting.
 * 
 *END**************************************************************************/
sim_tpm_clk_sel_t SIM_HAL_GetTpmExternalClkPinSelMode(uint32_t baseAddr, uint8_t instance)
{
    sim_tpm_clk_sel_t retValue = (sim_tpm_clk_sel_t)0;

    assert (instance < HW_TPM_INSTANCE_COUNT);

    switch (instance)
    {
    case 0:
        retValue = (sim_tpm_clk_sel_t)BR_SIM_SOPT4_TPM0CLKSEL(baseAddr);
        break;
    case 1:
        retValue = (sim_tpm_clk_sel_t)BR_SIM_SOPT4_TPM1CLKSEL(baseAddr);
        break;
    default:
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetTpmChSrcMode
 * Description   : Timer/PWM x channel y input capture source select setting 
 * This function will select Timer/PWM x channel y input capture source
 * 
 *END**************************************************************************/
void SIM_HAL_SetTpmChSrcMode(uint32_t baseAddr,
                             uint8_t instance,
                             uint8_t channel,
                             sim_tpm_ch_src_t select)
{
    assert (instance < HW_TPM_INSTANCE_COUNT);

    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            BW_SIM_SOPT4_TPM1CH0SRC(baseAddr, select);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetTpmChSrcMode
 * Description   : Get Timer/PWM x channel y input capture source select setting
 * This function will get Timer/PWM x channel y input capture source select 
 * setting.
 * 
 *END**************************************************************************/
sim_tpm_ch_src_t SIM_HAL_GetTpmChSrcMode(uint32_t baseAddr,
                                         uint8_t instance,
                                         uint8_t channel)
{
    sim_tpm_ch_src_t retValue = (sim_tpm_ch_src_t)0;

    assert (instance < HW_TPM_INSTANCE_COUNT);

    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            retValue = (sim_tpm_ch_src_t)BR_SIM_SOPT4_TPM1CH0SRC(baseAddr);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return retValue;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

