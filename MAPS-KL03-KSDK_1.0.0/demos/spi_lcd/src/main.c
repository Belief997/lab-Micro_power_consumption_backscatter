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
 */

#include <stdio.h>
#include <stdlib.h>

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "board.h"
#include "lcd.h"

     
#ifdef DEBUG
#include "fsl_debug_console.h"
#endif
 
extern const unsigned char Freescale_logo[];

spi_master_user_config_t g_master_config;
spi_master_state_t g_master_state;

int main(void)
{
    uint32_t calculatedBaudRate;
    
    /* Initialize standard SDK demo application pins */
    hardware_init();
    dbg_uart_init();
    
    configure_spi_lcd_pins(0);

    SPI_DRV_MasterInit(0, &g_master_state);
    g_master_state.spiSourceClock = CLOCK_SYS_GetSpiFreq(0);
    
    g_master_config.polarity = kSpiClockPolarity_ActiveHigh;
    g_master_config.phase = kSpiClockPhase_FirstEdge;
    g_master_config.direction = kSpiMsbFirst;
    g_master_config.bitsPerSec = 1000000;

    SPI_DRV_MasterConfigureBus(0, &g_master_config, &calculatedBaudRate);
    if (calculatedBaudRate > g_master_config.bitsPerSec)
    {
        printf("ERROR: Failed to set slow baud rate: %d!\r\n", g_master_config.bitsPerSec);
        return 1;
    }
    
    LCD_Initialize();
    
    while(1)
    {         
//        LCD_PutString(0,0, "hello kinetis       1");
//        LCD_PutString(0,8, "hello kinetis       2");
//        LCD_PutString(0,16,"hello kinetis       3");
//        LCD_PutString(0,24,"hello kinetis       4");
//        LCD_PutString(0,32,"hello kinetis       5");
//        LCD_PutString(0,40,"hello kinetis       6");
//        LCD_PutString(0,48,"hello kinetis       7");
//        LCD_PutString(0,56,"hello kinetis       8");
          LCD_FillAll((unsigned char *)Freescale_logo);
    }
}