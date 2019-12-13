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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_mcglite_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Table of base addresses for instances. */
extern const uint32_t g_simBaseAddr[];
const uint32_t g_mcgBaseAddr[] = MCG_BASE_ADDRS;
static uint32_t s_rtcClkInFreq = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetFreq
 * Description   : Internal function to get the frequency by clock name
 * This function will get/calculate the clock frequency based on clock name
 * and current configuration of clock generator.
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                                             uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = kClockManagerSuccess;
    int32_t dividerOne, dividerFour;
    uint8_t srcSetting;

    /* branch according to clock name */
    switch(clockName)
    {
    /* osc clock*/
    case kOsc0ErClock:
        *frequency = CPU_XTAL_CLK_HZ;
        break;
    case kOsc32kClock:
        CLOCK_HAL_GetSource(g_simBaseAddr[0], kClockOsc32kSel, &srcSetting);
        switch (srcSetting)
        {
        case 0:
            *frequency = CPU_XTAL_CLK_HZ;
            break;
        case 2:
            *frequency = s_rtcClkInFreq;
            break;
        case 3:
            *frequency = CPU_LPO_CLK_HZ;
            break;
        default:
            *frequency = 0;
            break;
        }
        break;
    /* irc clock*/
    case kIrc48mClock:
        *frequency = CPU_INT_FAST_CLK_HZ;
        break;
    /* RTC 1HZ clock */
    case kRtc1hzClock:
        *frequency = CPU_XTAL1hz_CLK_HZ;
        break;
    /* LPO clcok*/
    case kLpoClock:
        *frequency = CPU_LPO_CLK_HZ;
        break;

    /* mcglite clocks, calling mcg clock functions */
    case kMcgOutClock:
        *frequency = CLOCK_HAL_GetOutClk(g_mcgBaseAddr[0]);
        break;
    case kMcgIrClock:
        *frequency = CLOCK_HAL_GetInternalRefClk(g_mcgBaseAddr[0]);
        break;

    /* system clocks */
    case kCoreClock:
    case kSystemClock:
    case kPlatformClock:
        *frequency = CLOCK_HAL_GetOutClk(g_mcgBaseAddr[0]);
        /* get system clock divider*/
        if (kSimHalSuccess ==
             CLOCK_HAL_GetDivider(g_simBaseAddr[0], kClockDividerOutdiv1,(uint32_t *)&dividerOne))
        {
            /* get the frequency for the specified clock*/
            *frequency = (*frequency) / (dividerOne + 1);
        }
        else
        {
            returnCode = kClockManagerNoSuchDivider;
        }
        break;
    case kBusClock:
    case kFlashClock:
        *frequency = CLOCK_HAL_GetOutClk(g_mcgBaseAddr[0]);
        /* get system clock divider*/
        if (kSimHalSuccess ==
             CLOCK_HAL_GetDivider(g_simBaseAddr[0], kClockDividerOutdiv1,(uint32_t *)&dividerOne))
        {
            if (kSimHalSuccess ==
                 CLOCK_HAL_GetDivider(g_simBaseAddr[0], kClockDividerOutdiv4,(uint32_t *)&dividerFour))
            {
                /* get the frequency for the specified clock*/
                *frequency = (*frequency) / ((dividerOne + 1) * (dividerFour + 1));
            }
        }
        else
        {
            returnCode = kClockManagerNoSuchDivider;
        }
        break;

    /* reserved value*/
    case kReserved:
    default:
        *frequency = 55555;                     /* for testing use purpose*/
        returnCode = kClockManagerNoSuchClockName;
        break;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetPortFreq
 * Description   : Gets the clock frequency for PORT module
 * This function gets the clock frequency for PORT moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetPortFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kBusClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetFtfFreq
 * Description   : Gets the clock frequency for FTF module. (Flash Memory)
 * This function gets the clock frequency for FTF moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetFtfFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kFlashClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetAdcFreq
 * Description   : Gets the clock frequency for ADC module
 * This function gets the clock frequency for ADC moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetAdcFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kOsc0ErClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetCmpFreq
 * Description   : Gets the clock frequency for CMP module
 * This function gets the clock frequency for CMP moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetCmpFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kBusClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetVrefFreq
 * Description   : Gets the clock frequency for VREF module
 * This function gets the clock frequency for VREF moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetVrefFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kBusClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetTpmFreq
 * Description   : Gets the clock frequency for TPM module.
 * This function gets the clock frequency for TPM moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetTpmFreq(uint32_t instance)
{
    uint32_t freq = 0;
    uint8_t setting;
    clock_names_t clockName;

    /* get the LPUART clock source */
    if (kSimHalSuccess !=
          CLOCK_HAL_GetSource(g_simBaseAddr[0], kClockTpmSrc, &setting))
    {
        return freq;
    }

    /* get the correct source name */
    switch ((sim_tpm_clock_src_t)setting)
    {
    case kSimTpmSrcIrc:
        clockName = kIrc48mClock;
        break;
    case kSimTpmSrcOscEr:
        clockName = kOsc0ErClock;
        break;
    case kSimTpmSrcMcgIr:
        clockName = kMcgIrClock;
        break;
    default:
        clockName = kReserved;
        break;
    }

    if (clockName != kReserved)
    {
        /* Get ref clock freq */
        CLOCK_SYS_GetFreq(clockName, &freq);
    }

    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSpiFreq
 * Description   : Gets the clock frequency for SPI module. 
 * This function gets the clock frequency for SPI moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetSpiFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kBusClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetI2cFreq
 * Description   : Gets the clock frequency for I2C module. 
 * This function gets the clock frequency for I2C moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetI2cFreq(uint32_t instance)
{
    uint32_t freq = 0;
    CLOCK_SYS_GetFreq(kSystemClock, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetLpuartFreq
 * Description   : Gets the clock frequency for LPUART module. 
 * This function gets the clock frequency for LPUART moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetLpuartFreq(uint32_t instance)
{
    uint32_t freq = 0;
    uint8_t setting;
    clock_names_t clockName;

    /* get the LPUART clock source */
    if (kSimHalSuccess !=
          CLOCK_HAL_GetSource(g_simBaseAddr[0], kClockLpuart0Src, &setting))
    {
        return freq;
    }

    /* get the correct source name */
    switch ((sim_lpuart0_clock_src_t)setting)
    {
    case kSimLpUart0SrcIrc:
        clockName = kIrc48mClock;
        break;
    case kSimLpUart0SrcOscEr:
        clockName = kOsc0ErClock;
        break;
    case kSimLpUart0SrcMcgIr:
        clockName = kMcgIrClock;
        break;
    default:
        clockName = kReserved;
        break;
    }

    if (clockName != kReserved)
    {
        /* Get ref clock freq */
        CLOCK_SYS_GetFreq(clockName, &freq);
    }

    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetGpioFreq
 * Description   : Gets the clock frequency for GPIO module. 
 * This function gets the clock frequency for GPIO moudle.
 *
 *END**************************************************************************/
uint32_t CLOCK_SYS_GetGpioFreq(uint32_t instance)
{
    uint32_t freq = 0;

    CLOCK_SYS_GetFreq(kPlatformClock, &freq);

    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetRtcClkInFreq
 * Description   : function to set the frequency of the external RTC_CLKIN
 * This function will set the clock frequency of RTC_CLKIN.
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_SetRtcClkInFreq(uint32_t frequency)
{
    s_rtcClkInFreq = frequency;
    return kClockManagerSuccess;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
