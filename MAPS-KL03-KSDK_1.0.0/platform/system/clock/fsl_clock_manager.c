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

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
const uint32_t g_simBaseAddr[] = SIM_BASE_ADDRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetSource
 * Description   : Set clock source setting 
 * This function will set the settings for specified clock source. Each clock 
 * source has its clock selection settings. Refer to reference manual for 
 * details of settings for each clock source. Refer to clock_source_names_t 
 * for clock sources.
 * 
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_SetSource(clock_source_names_t clockSource,
                                               uint8_t setting)
{
    clock_manager_error_code_t returnCode = kClockManagerSuccess;

    if (CLOCK_HAL_SetSource(g_simBaseAddr[0], clockSource, setting) != kSimHalSuccess)
    {
        returnCode =  kClockManagerNoSuchClockSource;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSource
 * Description   : Get clock source setting
 * This function will get the settings for specified clock source. Each clock 
 * source has its clock selection settings. Refer to reference manual for 
 * details of settings for each clock source. Refer to clock_source_names_t
 * for clock sources.
 * 
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_GetSource(clock_source_names_t clockSource,
                                               uint8_t *setting)
{
    clock_manager_error_code_t returnCode = kClockManagerSuccess;

    if (CLOCK_HAL_GetSource(g_simBaseAddr[0], clockSource, setting) != kSimHalSuccess)
    {
        returnCode =  kClockManagerNoSuchClockSource;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetDivider
 * Description   : Set clock divider setting
 * This function will set the setting for specified clock divider. Refer to 
 * reference manual for supported clock divider and value range. Refer to 
 * clock_divider_names_t for dividers.
 * 
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_SetDivider(clock_divider_names_t clockDivider, 
                                                uint32_t setting)
{
    clock_manager_error_code_t returnCode = kClockManagerSuccess;

    if (CLOCK_HAL_SetDivider(g_simBaseAddr[0], clockDivider, setting) != kSimHalSuccess)
    {
        returnCode = kClockManagerNoSuchDivider;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetDivider
 * Description   : Get clock divider setting
 * This function will get the setting for specified clock divider. Refer to 
 * reference manual for supported clock divider and value range. Refer to 
 * clock_divider_names_t for dividers.
 * 
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_GetDivider(clock_divider_names_t clockDivider,
                                                uint32_t *setting)
{
    clock_manager_error_code_t returnCode = kClockManagerSuccess;

    if (CLOCK_HAL_GetDivider(g_simBaseAddr[0], clockDivider, setting) != kSimHalSuccess)
    {
        returnCode = kClockManagerNoSuchDivider;
    }

    return returnCode;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

