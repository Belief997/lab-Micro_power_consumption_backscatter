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
#include "fsl_rtc_driver.h"
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

int tpm_outputcompare (void)
{ 
    uint32_t tpm_clock_freq = 0;
    
    tpmModule  = 0;
    tpmChannel = 0;    
    tpmBaseAddr = g_tpmBaseAddr[tpmModule];
    
    tpm_general_config_t driver_info;
    memset(&driver_info, 0, sizeof(driver_info));

    printf("\r\n*********TPM OUTPUTCOMPARE DEMO START*********\r\n");
    
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);
    
    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm0_outputcompare);
    
    configure_tpm_pins(tpmModule);
    
    printf("TPM output compare toggle output every seconds\r\n");
    printf("[GREEN LED]: on/off every second\r\n");
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy64);

    tpm_clock_freq = TPM_DRV_GetClock(tpmModule);
    /*
     GREEN toggle every second
     make sure tpm_clock_freq never larger than TPM largest count period
    */
    if(tpm_clock_freq>0xFFFF)
    {
        tpm_clock_freq = 0xFFFF;
    }
    
    TPM_DRV_OutputCompareEnable(tpmModule, tpmChannel, kTpmToggleOutput, tpm_clock_freq, tpm_clock_freq/2, true);
    
    while(1)
    {
        if(countLoop>6)
        {
            countLoop = 0;
            break;
        }
    }
    
    TPM_DRV_Deinit(tpmModule);
    
    tpmChannel = 1;
    
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);
     
    configure_tpm_pins(tpmModule);
    
    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm0_outputcompare);
    
    printf("TPM output compare pulse output every seconds\r\n");
    printf("[RED  LED]: on very short time every second\r\n");
    
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy64);

    tpm_clock_freq = TPM_DRV_GetClock(tpmModule);
    /*
     RED pulse on every second
     make sure tpm_clock_freq never larger than TPM largest count period
    */
    if(tpm_clock_freq>0xFFFF)
    {
        tpm_clock_freq = 0xFFFF;
    }
    
    TPM_DRV_OutputCompareEnable(tpmModule, tpmChannel, kTpmLowPulseOutput, tpm_clock_freq, tpm_clock_freq/2, true);

    while(1)
    {
        if(countLoop>6)
        {
            countLoop = 0;
            break;
        }
    }
    
    TPM_DRV_Deinit(tpmModule);

    tpmModule = 1;
    tpmChannel = 1;
    
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);
     
    configure_tpm_pins(tpmModule);
    
    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm1_outputcompare);
    
    printf("TPM output compare pulse output clear \r\n");
    printf("[RED LED]: on very short time once compared\r\n");
    
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy64);

    tpm_clock_freq = TPM_DRV_GetClock(tpmModule);
    /*
     RED pulse on every second
     make sure tpm_clock_freq never larger than TPM largest count period
    */
    if(tpm_clock_freq>0xFFFF)
    {
        tpm_clock_freq = 0xFFFF;
    }
    
    TPM_DRV_OutputCompareEnable(tpmModule, tpmChannel, kTpmClearOutput, tpm_clock_freq, tpm_clock_freq/100, true);

    while(1)
    {
        if(countLoop>1)
        {
            countLoop = 0;
            break;
        }
    }
    
    printf("TPM output compare pulse output set \r\n");
    printf("[RED LED]: off very short time once compared\r\n");
    
    TPM_DRV_OutputCompareEnable(tpmModule, tpmChannel, kTpmSetOutput, tpm_clock_freq, tpm_clock_freq/100, true);

    while(1)
    {
        if(countLoop>1)
        {
            countLoop = 0;
            break;
        }
    }
    
    TPM_DRV_Deinit(tpmModule);

    
    printf("*********TPM OUTPUTCOMPARE DEMO COMPLETE*********\r\n");
    return 0;

}


/*
* tpm0 call back function
*/
void tpm_user_callback_tpm0_outputcompare(void)
{
    countLoop++;
}

/*
* tpm1 call back function
*/
void tpm_user_callback_tpm1_outputcompare(void)
{
    countLoop++;
}


/********************************************************************/
/********************************************************************/
