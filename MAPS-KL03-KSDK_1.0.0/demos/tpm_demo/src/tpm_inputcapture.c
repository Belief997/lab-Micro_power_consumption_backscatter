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

static uint32_t capture_value[8] = {0};
static uint32_t calculate_result[2] = {0};
    
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/********************************************************************/

int tpm_inputcapture (void)
{    
    uint32_t tpm_clock_freq = 0;
    tpmModule  = 0;
    tpmChannel = 0;
    tpmBaseAddr = g_tpmBaseAddr[tpmModule];
    
    tpm_general_config_t driver_info;
    memset(&driver_info, 0, sizeof(driver_info));
    
    printf("\r\n*********TPM0 INPUTCAPTURE DEMO DESCRIPTION*********\r\n");
    printf("This Demo use LPUART0 rxd feed to TPM0 CH0\r\n");
    printf("TPM config in capture both edges of input signals\r\n");
    printf("Input from HyperTerminal will cause TPM0 CH0 capture the events\r\n");
    printf("After 8 captures, TPM0 is disabled\r\n");
    printf("And start to calculate the smallest interval of the input edges\r\n");
    printf("then print the calculate result\r\n");
    printf("\r\n*********TPM0 INPUTCAPTURE DEMO START*********\r\n");
    
    printf("UART0 rxd output a pulse to TPM0 CH0\r\n");
    printf("TPM0 CH0 InputCapture, rising or falling edge\r\n");
    printf("Connect LPUART0 RXD PTB2 and PTB11 together\r\n");
    printf("Input any key some times from HyperTerminal tool to continue test\r\n");
    TPM_DRV_Init(tpmModule, &driver_info, &tpmState);

    TPM_DRV_InstallCallback(tpmModule, tpm_user_callback_tpm0_inputcapture);
    
    configure_tpm_pins(tpmModule);
    
    /*
     depending on user MCG clock settings, unit test should take a correct clock source
    */
    TPM_DRV_SetClock(tpmModule, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy2);

    tpm_clock_freq = TPM_DRV_GetClock(tpmModule);
    
    TPM_DRV_InputCaptureEnable(tpmModule, tpmChannel, kTpmRiseOrFallEdge, tpm_clock_freq, true);
    
    while(1)
    {
        if(countLoop>=7)
        {
            countLoop = 0;
            break;
        }        
    }

    
    TPM_DRV_Deinit(tpmModule);

    calculate_result[0] = capture_value[1] - capture_value[0];
    if(calculate_result[0]<0)
    {
        calculate_result[0] = capture_value[0] - capture_value[1];
    }
    for(countLoop = 2; countLoop<=7; countLoop++)
    {
        calculate_result[1] = capture_value[countLoop] - capture_value[countLoop-1];
        if(calculate_result[1] <0)
        {
            calculate_result[1] = capture_value[countLoop-1] - capture_value[countLoop];
        }
        if(calculate_result[1]<calculate_result[0])
        {
            calculate_result[0] = calculate_result[1];
        }
    }
    printf("calculate value = %d\r\n", (int)calculate_result[0]);
    printf("calculate BPS of UART0 is %d\r\n", (int)(tpm_clock_freq/calculate_result[0]));
    printf("*********TPM INPUTCAPTURE DEMO COMPLETE*********\r\n");
    return 0;

}


/*
* tpm0 call back function
*/
void tpm_user_callback_tpm0_inputcapture(void)
{
    countLoop++;
    capture_value[countLoop] = TPM_DRV_GetChnVal(tpmModule, tpmChannel);
}




/********************************************************************/
/********************************************************************/
