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

#include <stdio.h>
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"
#include "fsl_cmp_driver.h"
#include "fsl_os_abstraction.h"

static cmp_state_t s_CmpState;
static cmp_user_config_t s_cmpConfig;

static void cmp_interrupt_callback(void)
{
    bool bFlag1, bFlag2;
    
    bFlag1 = CMP_DRV_GetFlag(HW_CMP0, kCmpFlagOfCoutFalling);
    bFlag2 = CMP_DRV_GetFlag(HW_CMP0, kCmpFlagOfCoutRising);
    if( (bFlag1 == true) && (s_cmpConfig.fallingIntEnable == true) )
    {
        printf("\r\nF");
        CMP_DRV_ClearFlag(HW_CMP0, kCmpFlagOfCoutFalling);
    }
    if( (bFlag2 == true) && (s_cmpConfig.risingIntEnable == true))
    {
        printf("\r\nR");
        CMP_DRV_ClearFlag(HW_CMP0, kCmpFlagOfCoutRising);
    }
}

static void init_hardware(void)
{
    hardware_init();
    configure_cmp_pins(0);
    dbg_uart_init();
}

int main(void)
{
    cmp_dac_config_t DACChannelConfig;
    cmp_sample_filter_config_t FilterConfig;
	
    init_hardware();
    
    OSA_Init();

    printf("\r\nCMP demo start...");
    printf("\r\n*************************");

    /* initialize cmp user config struct to a default state */
    CMP_DRV_StructInitUserConfigDefault(&s_cmpConfig, kCmpInputChn0, kCmpInputChnDac);
    /* change some user configurations to meet our requirement */
    s_cmpConfig.pinoutEnable = true;
    s_cmpConfig.pinoutUnfilteredEnable = true;
    s_cmpConfig.risingIntEnable = true;
    s_cmpConfig.fallingIntEnable = true;

    /* initialize CMP core module */
    CMP_DRV_Init(HW_CMP0, &s_cmpConfig, &s_CmpState);
    /* install a interrupt callback function to handle the interrupt */
    CMP_DRV_InstallCallback(HW_CMP0, cmp_interrupt_callback);

    /* if use internal DAC output as CMP input, should call this API to set internal DAC */
    DACChannelConfig.refVoltSrcMode = kCmpDacRefVoltSrcOf2;
    DACChannelConfig.dacValue = 31U;
    CMP_DRV_EnableDac(HW_CMP0, &DACChannelConfig);

    /* set the filter block of the CMP module */
    FilterConfig.workMode = kCmpContinuousMode;
    FilterConfig.filterCount = kCmpFilterCountSampleOf4;
    FilterConfig.filterClkDiv = 2;
    FilterConfig.useExtSampleOrWindow = false;
    CMP_DRV_ConfigSampleFilter(HW_CMP0, &FilterConfig);
   
    /* Enable the CMP module */
    CMP_DRV_Start(HW_CMP0);

    OSA_TimeDelay(10000); //Delay 10s for outputing logs
    
    CMP_DRV_DisableDac(HW_CMP0);
    CMP_DRV_Stop(HW_CMP0);
    CMP_DRV_Deinit(HW_CMP0);

    configure_swd_pins(0);
    
    printf("\r\nCMP demo end.");
    
    return 0;
}



