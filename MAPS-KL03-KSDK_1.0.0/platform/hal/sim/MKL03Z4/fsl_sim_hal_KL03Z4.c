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
#include "fsl_sim_hal_KL03Z4.h"
#include "fsl_sim_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief CLOCK name config table for KL03 */
const clock_name_config_t kClockNameConfigTable [] =  {
    {false, kSystemClock,     kClockDividerOutdiv1},
    {false, kSystemClock,     kClockDividerOutdiv1},
    {false, kSystemClock,     kClockDividerOutdiv1},
    {true,  kSystemClock,     kClockDividerOutdiv4}, /* fake for flexbus clock */
    {true,  kSystemClock,     kClockDividerOutdiv4},
    {true,  kSystemClock,     kClockDividerOutdiv4}
};

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnablePortClock
 * Description   : Enable the clock for PORT module
 * This function enables the clock for PORT moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnablePortClock(uint32_t baseAddr, uint32_t instance)
{
    switch (instance)
    {
    case 0:
        BW_SIM_SCGC5_PORTA(baseAddr, 1);
        break;
    case 1:
        BW_SIM_SCGC5_PORTB(baseAddr, 1);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisablePortClock
 * Description   : Disable the clock for PORT module
 * This function disables the clock for PORT moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisablePortClock(uint32_t baseAddr, uint32_t instance)
{
    switch (instance)
    {
    case 0:
        BW_SIM_SCGC5_PORTA(baseAddr, 0);
        break;
    case 1:
        BW_SIM_SCGC5_PORTB(baseAddr, 0);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetPortGateCmd
 * Description   : Get the the clock gate state for PORT module
 * This function will get the clock gate state for PORT moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetPortGateCmd(uint32_t baseAddr, uint32_t instance)
{
    bool retValue = false;

    switch (instance)
    {
    case 0:
        retValue =  BR_SIM_SCGC5_PORTA(baseAddr);
        break;
    case 1:
        retValue =  BR_SIM_SCGC5_PORTB(baseAddr);
        break;
    default:
        retValue =  false;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableFtfClock
 * Description   : Enable the clock for FTF module
 * This function enables the clock for FTF moudle.
 *
 *END**************************************************************************/
void SIM_HAL_EnableFtfClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_FTF(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableFtfClock
 * Description   : Disable the clock for FTF module
 * This function disables the clock for FTF moudle.
 *
 *END**************************************************************************/
void SIM_HAL_DisableFtfClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_FTF(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtfGateCmd
 * Description   : Get the the clock gate state for FTF module
 * This function will get the clock gate state for FTF moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetFtfGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC6_FTF(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableAdcClock
 * Description   : Enable the clock for ADC module
 * This function enables the clock for ADC moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableAdcClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_ADC0(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableAdcClock
 * Description   : Disable the clock for ADC module
 * This function disables the clock for ADC moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableAdcClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_ADC0(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcGateCmd
 * Description   : Get the the clock gate state for ADC module
 * This function will get the clock gate state for ADC moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetAdcGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC6_ADC0(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableCmpClock
 * Description   : Enable the clock for CMP module
 * This function enables the clock for CMP moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableCmpClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_CMP(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableCmpClock
 * Description   : Disable the clock for CMP module
 * This function disables the clock for CMP moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableCmpClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_CMP(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetCmpGateCmd
 * Description   : Get the the clock gate state for CMP module
 * This function will get the clock gate state for CMP moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetCmpGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC4_CMP(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableVrefClock
 * Description   : Enable the clock for VREF module
 * This function enables the clock for VREF moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableVrefClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_VREF(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableVrefClock
 * Description   : Disable the clock for VREF module
 * This function disables the clock for VREF moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableVrefClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_VREF(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetVrefGateCmd
 * Description   : Get the the clock gate state for VREF module
 * This function will get the clock gate state for VREF moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetVrefGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC4_VREF(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableTpmClock
 * Description   : Enable the clock for TPM module
 * This function enables the clock for TPM moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableTpmClock(uint32_t baseAddr, uint32_t instance)
{
    switch (instance)
    {
    case 0:
        BW_SIM_SCGC6_TPM0(baseAddr, 1);
        break;
    case 1:
        BW_SIM_SCGC6_TPM1(baseAddr, 1);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableTpmClock
 * Description   : Disable the clock for TPM module
 * This function disables the clock for TPM moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableTpmClock(uint32_t baseAddr, uint32_t instance)
{
    switch (instance)
    {
    case 0:
        BW_SIM_SCGC6_TPM0(baseAddr, 0);
        break;
    case 1:
        BW_SIM_SCGC6_TPM1(baseAddr, 0);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetTpmGateCmd
 * Description   : Get the the clock gate state for TPM module
 * This function will get the clock gate state for TPM moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetTpmGateCmd(uint32_t baseAddr, uint32_t instance)
{
    bool retValue = false;

    switch (instance)
    {
    case 0:
        retValue =  BR_SIM_SCGC6_TPM0(baseAddr);
        break;
    case 1:
        retValue =  BR_SIM_SCGC6_TPM1(baseAddr);
        break;
    default:
        retValue =  false;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableLptimerClock
 * Description   : Enable the clock for LPTIMER module
 * This function enables the clock for LPTIMER moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableLptimerClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC5_LPTMR(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableLptimerClock
 * Description   : Disable the clock for LPTIMER module
 * This function disables the clock for LPTIMER moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableLptimerClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC5_LPTMR(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetLptimerGateCmd
 * Description   : Get the the clock gate state for LPTIMER module
 * This function will get the clock gate state for LPTIMER moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetLptimerGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC5_LPTMR(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableRtcClock
 * Description   : Enable the clock for RTC module
 * This function enables the clock for RTC moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableRtcClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_RTC(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableRtcClock
 * Description   : Disable the clock for RTC module
 * This function disables the clock for RTC moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableRtcClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC6_RTC(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetRtcGateCmd
 * Description   : Get the the clock gate state for RTC module
 * This function will get the clock gate state for RTC moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetRtcGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC6_RTC(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableSpiClock
 * Description   : Enable the clock for SPI module
 * This function enables the clock for SPI moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableSpiClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_SPI0(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableSpiClock
 * Description   : Disable the clock for SPI module
 * This function disables the clock for SPI moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableSpiClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_SPI0(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetSpiGateCmd
 * Description   : Get the the clock gate state for SPI module
 * This function will get the clock gate state for SPI moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetSpiGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC4_SPI0(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableI2cClock
 * Description   : Enable the clock for I2C module
 * This function enables the clock for I2C moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableI2cClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_I2C0(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableI2cClock
 * Description   : Disable the clock for I2C module
 * This function disables the clock for I2C moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableI2cClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC4_I2C0(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetI2cGateCmd
 * Description   : Get the the clock gate state for I2C module
 * This function will get the clock gate state for I2C moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetI2cGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC4_I2C0(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableLpuartClock
 * Description   : Enable the clock for LPUART module
 * This function enables the clock for LPUART moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableLpuartClock(uint32_t baseAddr, uint32_t instance)
{
   BW_SIM_SCGC5_LPUART0(baseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableLpuartClock
 * Description   : Disable the clock for LPUART module
 * This function disables the clock for LPUART moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableLpuartClock(uint32_t baseAddr, uint32_t instance)
{
    BW_SIM_SCGC5_LPUART0(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetLpuartGateCmd
 * Description   : Get the the clock gate state for LPUART module
 * This function will get the clock gate state for LPUART moudle.
 *
 *END**************************************************************************/
bool SIM_HAL_GetLpuartGateCmd(uint32_t baseAddr, uint32_t instance)
{
    return BR_SIM_SCGC5_LPUART0(baseAddr);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

