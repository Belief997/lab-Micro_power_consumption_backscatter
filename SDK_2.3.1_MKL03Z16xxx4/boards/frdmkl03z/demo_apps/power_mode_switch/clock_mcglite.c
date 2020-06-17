/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "fsl_common.h"
#include <assert.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * Set the clock configuration for RUN mode from VLPR mode.
 */
void APP_SetClockRunFromVlpr(void)
{
    const mcglite_config_t mcgliteConfig = {.outSrc = kMCGLITE_ClkSrcHirc,
                                            .irclkEnableMode = 0U, // kMCGLITE_IrclkEnable
                                            .ircs = kMCGLITE_Lirc8M,
                                            .fcrdiv = kMCGLITE_LircDivBy1,
                                            .lircDiv2 = kMCGLITE_LircDivBy1,
                                            .hircEnableInNotHircMode = true};

//    const mcglite_config_t mcgliteConfig = {.outSrc = kMCGLITE_ClkSrcHirc,
//                                            .irclkEnableMode = 0U, // kMCGLITE_IrclkEnable
//                                            .ircs = kMCGLITE_Lirc8M,
//                                            .fcrdiv = kMCGLITE_LircDivBy2,
//                                            .lircDiv2 = kMCGLITE_LircDivBy4,
//                                            .hircEnableInNotHircMode = true};

    const sim_clock_config_t simConfig =
    {
        .clkdiv1 = 0x00010000U, /* SIM_CLKDIV1. */
#if (defined(FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION) && FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION)
        .er32kSrc = 0U, /* SIM_SOPT1[OSC32KSEL]. */
#endif
    };

    CLOCK_SetSimSafeDivs();
    CLOCK_SetMcgliteConfig(&mcgliteConfig);
    CLOCK_SetSimConfig(&simConfig);
}

/*
 * Set the clock configuration for VLPR mode.
 */
void APP_SetClockVlpr(void)
{
//        const mcglite_config_t mcgliteConfig = {
//        .outSrc = kMCGLITE_ClkSrcLirc,
//        .irclkEnableMode = kMCGLITE_IrclkEnable,
//        .ircs = kMCGLITE_Lirc2M,
//        .fcrdiv = kMCGLITE_LircDivBy1,  // 内核时钟分频系数，当频率小于或等于 2M 时才能进入 VLPR
//        .lircDiv2 = kMCGLITE_LircDivBy1, // 
//        .hircEnableInNotHircMode = false,
//    };
        
    const mcglite_config_t mcgliteConfig = {
        .outSrc = kMCGLITE_ClkSrcLirc,
        .irclkEnableMode = kMCGLITE_IrclkEnable,
//        .ircs = kMCGLITE_Lirc2M,
        .ircs = kMCGLITE_Lirc8M,
        .fcrdiv = kMCGLITE_LircDivBy4,  // 内核时钟分频系数，当频率小于或等于 2M 时才能进入 VLPR
        .lircDiv2 = kMCGLITE_LircDivBy1, // 
        .hircEnableInNotHircMode = false,   
    };

    const sim_clock_config_t simConfig =
    {
        .clkdiv1 = 0x00010000U, /* SIM_CLKDIV1. */
#if (defined(FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION) && FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION)
        .er32kSrc = 0U, /* SIM_SOPT1[OSC32KSEL]. */
#endif
    };

    CLOCK_SetSimSafeDivs();
    CLOCK_SetMcgliteConfig(&mcgliteConfig);
    CLOCK_SetSimConfig(&simConfig);
}
