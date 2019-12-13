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
#include <string.h>
#include "fsl_interrupt_manager.h"
#include "fsl_tpm_common.h"
#include "fsl_tpm_driver.h"
#include "fsl_clock_manager.h"
#include "board.h"
#include "tpm_demo.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Global Variables
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/********************************************************************/


int tpm_pwm (void)
{      
    tpmModule  = 0;
    tpmChannel = 0;
    
    tpm_pwm_param_t param = {
        .mode = kTpmEdgeAlignedPWM,
        .edgeMode = kTpmHighTrue,
        .uFrequencyHZ = 10,
        .uDutyCyclePercent = 10
    };
    
    tpmBaseAddr = g_tpmBaseAddr[tpmModule];
    
    tpm_general_config_t driver_info;
    
    memset(&driver_info, 0, sizeof(driver_info));

    printf("\r\n*********TPM PWM DEMO START*********\r\n");
    
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);
    
    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm0_pwm);
     
    TPM_DRV_SetChnIntCmd(tpmModule, tpmChannel, true);
    
    configure_tpm_pins(tpmModule);
    
    printf("[GREEN LED]: TPM0 PWM duty increase from 10%% to 90%%, every second by 10%%\r\n");
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy64);

    /* first test for edge aligned PWM */
    TPM_DRV_PwmStart(tpmModule, &param, tpmChannel);
    
    tpmChannelValue = TPM_HAL_GetChnCountVal(tpmBaseAddr, tpmChannel);
    
    while(1)
    {
        if(countLoop>90)
        {
            countLoop = 0;
            break;
        }

    }
    
    TPM_DRV_Deinit(tpmModule);
    
    CLOCK_SYS_EnableTpmClock(tpmModule);
    
    tpmModule = 1;
    tpmChannel = 1;
    
    tpmBaseAddr = g_tpmBaseAddr[tpmModule];
    
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);

    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm1_pwm);
    
    TPM_DRV_SetChnIntCmd(tpmModule, tpmChannel, true);
    
    configure_tpm_pins(tpmModule);
    
    printf("[RED  LED]: TPM1 PWM duty decrease from 90%% to 10%%, every second by 10%%\r\n");
    
    param.uDutyCyclePercent = 90;
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy64);

    /* first test for edge aligned PWM */
    TPM_DRV_PwmStart(tpmModule, &param, tpmChannel);
    
    tpmChannelValue = TPM_HAL_GetChnCountVal(tpmBaseAddr, tpmChannel);
    
    while(1)
    {
        if(countLoop>90)
        {
            countLoop = 0;
            break;
        }        
    }
    
    TPM_DRV_Deinit(tpmModule);
    
    printf("*********TPM PWM DEMO COMPLETE*********\r\n");
    return 0;

}


/*
 * tpm0 call back funciton
*/
void tpm_user_callback_tpm0_pwm(void)
{
    countLoop++;
    if((countLoop%10) == 0)
    {
        TPM_HAL_SetChnCountVal(tpmBaseAddr, tpmChannel, tpmChannelValue*(countLoop/10));
    }    
}

/*
* tpm1 call back function
*/
void tpm_user_callback_tpm1_pwm(void)
{
    countLoop++;
    if((countLoop%10) == 0)
    {
        TPM_HAL_SetChnCountVal(tpmBaseAddr, tpmChannel, tpmChannelValue * (9 - countLoop/10)/9);
    }
    
}


/********************************************************************/
/********************************************************************/
