/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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
#include <stdlib.h>
#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
#include "fsl_adc_driver.h"

#define RES_MAX_VAL   4095
#define VREF_ADC      33
#define ADJUST_FACTOR 10000

static void print_menu(void)
{
    printf("Select Potentiometer mode:\r\n");
    printf("1: Potentiometer -- Single End(RV2).\r\n");
    printf("2: Potentiometer -- Single End(RV1).\r\n"); 
    
    printf("Please enter your choice (1 - 2): ");
}

static uint32_t get_user_choice(void)
{
    uint32_t in_val = 0;

    in_val = getchar();

    putchar(in_val);

    putchar('\r');
    putchar('\n');

    return in_val;
}

static int32_t getVoltage(uint32_t channel, bool diff, adc_resolution_mode_t resMode)
{
    adc_chn_config_t MyChnConfig;
    volatile int16_t MyAdcValue;

    MyChnConfig.chnNum = channel;
    MyChnConfig.diffEnable = diff;
    MyChnConfig.intEnable = false;
    MyChnConfig.chnMux = kAdcChnMuxOfDefault;
    ADC_DRV_ConfigConvChn(HW_ADC0, 0U, &MyChnConfig);

    /* poll to complete status and read back result */
    ADC_DRV_WaitConvDone(HW_ADC0, 0U);
    MyAdcValue = ADC_DRV_GetConvValueRAW(HW_ADC0, 0U);
    //MyAdcValue = ADC_DRV_ConvRAWData(MyAdcValue, diff, resMode);
    //printf("\r\nRaw: %X, %d\r\n", MyAdcValue, MyAdcValue);

    return MyAdcValue;
}

static void print_voltage(int32_t adcValue)
{
    int32_t integer;
    int32_t decimal;
    uint32_t isNegative = 0;
    uint32_t decimalNumber = 0;
    uint32_t numerator = 0;
    uint32_t denominator = (RES_MAX_VAL * 10);
    uint32_t i = 0;

    if(adcValue < 0)
    {
        adcValue = -adcValue;
        isNegative = 1;
    }

    integer = (adcValue * VREF_ADC) / denominator;
    numerator = (adcValue * VREF_ADC) - (integer * denominator);
    
    if(numerator)
    {
        while( (numerator / denominator) == 0 )
        {
            decimalNumber++;
            denominator /= 10;
            if(denominator == 0)
            {
                break;
            }
        }
    }
    decimal = (numerator*ADJUST_FACTOR) / (RES_MAX_VAL * 10);

    printf("voltage: ");
    if(isNegative != 0)
    {
        printf("-");
    }
    printf("%d.", integer);
    for(i=0; i<decimalNumber-1; i++)
    {
        printf("0");
    }
    printf("%dV", decimal);
    printf("\r\n");
}

int main (void)
{
    uint32_t select;

    hardware_init();

    OSA_Init();

    dbg_uart_init();

    configure_adc_pins(0);

    printf("\r\nPotentiometer demo start...\r\n");

    adc_calibration_param_t MyAdcCalibraitionParam;
    adc_user_config_t MyAdcUserConfig;
    adc_state_t MyAdcState;
    volatile int32_t adcValue;

    /* Auto calibration. */
    ADC_DRV_GetAutoCalibrationParam(HW_ADC0, &MyAdcCalibraitionParam);
    ADC_DRV_SetCalibrationParam(HW_ADC0, &MyAdcCalibraitionParam);

    /* Initialization for interrupt mode. */ 
    ADC_DRV_StructInitUserConfigForOneTimeTriggerMode(&MyAdcUserConfig);
    MyAdcUserConfig.resolutionMode = kAdcResolutionBitOfSingleEndAs12;
    ADC_DRV_Init(HW_ADC0, &MyAdcUserConfig, &MyAdcState);

    while(1)
    {
        printf("\r\n");
        print_menu();

        switch (get_user_choice())
        {
        case '1':
            adcValue = getVoltage(9, false, MyAdcUserConfig.resolutionMode);
            print_voltage(adcValue);
            break;
        case '2':
            adcValue = getVoltage(2, false, MyAdcUserConfig.resolutionMode);
            print_voltage(adcValue);
            break;
        default:
            printf("Invalid choice.\r\n");
            break;
        }
    }
}
