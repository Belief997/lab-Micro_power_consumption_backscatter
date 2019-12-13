/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 *
 */

/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
/*******************************************************************************
 * SDK Included Files
 ******************************************************************************/
#include "AT24C02.h"
#include "board.h"
#include "fsl_i2c_hal.h"
#include "fsl_i2c_common.h"

#include "fsl_debug_console.h"


/*******************************************************************************
 * Application Included Files
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

   
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/******************************************************************************/
int main (void)
{
    at24c02_handler_t handle;
    at24c02_status_t status;
    uint8_t write_data;	
    uint8_t read_data;
    uint8_t eeprom_add;
	uint16_t cnt;
    
    hardware_init();
    //configure_uart_pins(4);
    dbg_uart_init();
    
    configure_i2c_pins(0);
    
    OSA_Init();
	
    /* Initialize UART terminal. */
    //DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUD, kDebugConsoleUART);
    
    printf("EEPROM Demo\n\r");
    printf("This demo will write data into on board EEPROM and then read it back\n\r");

    handle.i2c_instance = 0;
    
    AT24C02_I2CInit(&handle);

    for(cnt = 0; cnt < 256; cnt++)
    {
        eeprom_add = cnt;
        write_data = 255 - cnt;
        
        status = AT24C02_ByteWrite(&handle, eeprom_add, write_data);
        if(status == kStatus_AT24C02_I2CFail)
          printf("Write failure\n");
        
        AT24C02_ACKPoll(&handle);
	
        status = AT24C02_ByteRead(&handle, eeprom_add, &read_data);
        if(status == kStatus_AT24C02_I2CFail)
    	  printf("Read failure\n");
        
        if(read_data != write_data)
          printf("Read data is 0x%x instead of 0x%x\n", read_data, write_data);
        else
          printf("Write data at 0x%x is 0x%x, read back data is 0x%x\n\r", eeprom_add, write_data, read_data);
        
    }

    while(1)
    {

    }
}
/******************************************************************************/

/******************************************************************************
 * EOF
 ******************************************************************************/
