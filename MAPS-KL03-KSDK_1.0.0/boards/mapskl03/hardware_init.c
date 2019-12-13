/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_sim_hal.h"
#include "pin_mux.h"

void hardware_init(void)
{
    int32_t i;

    /* enable clock for PORTs */
    for (i = 0; i < HW_PORT_INSTANCE_COUNT; i++)
    {
        CLOCK_SYS_EnablePortClock(i);
    }
#if (CLOCK_SETUP == 1)
    CLOCK_SYS_SetSource(kClockLpuart0Src, 1);
#else
    CLOCK_SYS_SetSource(kClockLpuart0Src, 3);
#endif    
    for (i = 0; i < HW_PORT_INSTANCE_COUNT; i++)
    {
        configure_gpio_pins(i);
    }

    //configure_i2c_pins(0);
    //configure_spi_pins(0);
}

void dbg_uart_init(void)
{
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
    //DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUD, kDebugConsoleLPUART);
    DbgConsole_Init(0, BOARD_DEBUG_UART_BAUD, kDebugConsoleLPUART);
}
