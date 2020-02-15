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
 
/*!
**  @addtogroup pin_mux_module pin_mux module documentation
**  @{
*/         

/* MODULE pin_mux. */

#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"
#include "fsl_gpio_common.h"
#include "pin_mux.h"


void configure_gpio_pins(uint32_t instance)
{
  switch(instance) {    
    case 1:                             /* PTB */
      /* PORTB_PCR5 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],5u,kPortMuxAsGpio);
      /* PORTB_PCR7 */
//      PORT_HAL_SetMuxMode(g_portBaseAddr[1],7u,kPortMuxAsGpio);
      /* PORTB_PCR10 */
//      PORT_HAL_SetMuxMode(g_portBaseAddr[1],10u,kPortMuxAsGpio);
      /* PORTB_PCR11 */
//      PORT_HAL_SetMuxMode(g_portBaseAddr[1],11u,kPortMuxAsGpio);
      /* PORTB_PCR13 */
//      PORT_HAL_SetMuxMode(g_portBaseAddr[1],13u,kPortMuxAsGpio);
      break;
    default:
      break;
  }
}

void configure_i2c_pins(uint32_t instance)
{
  switch(instance) {    
    case 0:                             /* I2C0 */
      /* PORTB_PCR3 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],3u,kPortMuxAlt2);
      /* PORTB_PCR4 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],4u,kPortMuxAlt2);
      break;
    default:
      break;
  }
}

void configure_rtc_pins(uint32_t instance)
{
  /* PORTB_PCR13 */
  PORT_HAL_SetMuxMode(g_portBaseAddr[1],13u,kPortMuxAlt3);
}

void configure_spi_pins(uint32_t instance)
{
  switch(instance) {    
    case 0:                             /* SPI0 */
      /* PORTA_PCR6  MISO */
      PORT_HAL_SetMuxMode(g_portBaseAddr[0],6u,kPortMuxAlt3);
      /* PORTA_PCR7 MOSI */
      PORT_HAL_SetMuxMode(g_portBaseAddr[0],7u,kPortMuxAlt3);
      /* PORTB_PCR0 SCLK */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],0u,kPortMuxAlt3);
      break;
    default:
      break;
  }
}

void configure_uart_pins(uint32_t instance)
{
  switch(instance) {    
    case 0:                             /* DBG UART */
      /* PORTA_PCR3 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[0],3u,kPortMuxAlt4);
      /* PORTA_PCR4 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[0],4u,kPortMuxAlt4);
      break;
	case 1:                             /* UART1 */
      /* PORTB_PCR1 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],1u,kPortMuxAlt2);
      /* PORTB_PCR2 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],2u,kPortMuxAlt2);
      break;
	case 2:                             /* UART2 */
      /* PORTB_PCR3 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],3u,kPortMuxAlt3);
      /* PORTB_PCR4 */
      PORT_HAL_SetMuxMode(g_portBaseAddr[1],4u,kPortMuxAlt3);
      break;
    default:
      break;
  }
}

void configure_spi_mem_cs_pins(uint32_t instance)
{
  PORT_HAL_SetMuxMode(g_portBaseAddr[0],5u,kPortMuxAsGpio);
}

void configure_spi_sd_cs_pins(uint32_t instance)
{
  PORT_HAL_SetMuxMode(g_portBaseAddr[0],9u,kPortMuxAsGpio);
}

void configure_spi_lcd_pins(uint32_t instance)
{
    /* CS */
    PORT_HAL_SetMuxMode(g_portBaseAddr[0],12u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOA_BASE, 12u, kGpioDigitalOutput);
    /* C/D */
    PORT_HAL_SetMuxMode(g_portBaseAddr[0],8u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOA_BASE, 8u, kGpioDigitalOutput);
    /* SCLK */
    PORT_HAL_SetMuxMode(g_portBaseAddr[1],0u,kPortMuxAlt3);
    /* MOSI */
    PORT_HAL_SetMuxMode(g_portBaseAddr[0],7u,kPortMuxAlt3);
}

void configure_spi_clcd_pins(uint32_t instance)
{
    /* CS */
    PORT_HAL_SetMuxMode(PORTB_BASE,6u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOB_BASE, 6u, kGpioDigitalOutput);
    /* SCLK */
    PORT_HAL_SetMuxMode(PORTB_BASE,0u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOB_BASE, 0u, kGpioDigitalOutput);
    /* SDA - MOSI */
    PORT_HAL_SetMuxMode(PORTA_BASE,7u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOA_BASE, 7u, kGpioDigitalOutput);
    /* SD0 - MISO */
    PORT_HAL_SetMuxMode(PORTA_BASE,6u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOA_BASE, 6u, kGpioDigitalInput);
    /* BLC */
    PORT_HAL_SetMuxMode(PORTB_BASE,13u,kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(GPIOB_BASE, 13u, kGpioDigitalOutput);
}

void configure_tpm_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* TPM0 */
        /* PTB11 TPM0 channel 0 -- GRENN LED*/
        PORT_HAL_SetMuxMode(g_portBaseAddr[1],11u,kPortMuxAlt2);
        /* PTB10 TPM0 channel 1 -- RED  LED*/
        PORT_HAL_SetMuxMode(g_portBaseAddr[1],10u,kPortMuxAlt2);
      break;
    case 1:                             /* TPM1 */
        /* PTB7  TPM1 channel 0 -- No LED*/
        PORT_HAL_SetMuxMode(g_portBaseAddr[1],7u,kPortMuxAlt2);
        /* PTB13 TPM1 channel 1 --RED LED */
        PORT_HAL_SetMuxMode(g_portBaseAddr[1],13u,kPortMuxAlt2);
      break;
    default:
      break;
  }
}

void configure_cmp_pins(uint32_t instance)
{
  switch(instance) {
    case 0:                             /* CMP0 */
        /* PTA12 CMP0 input channel 0 */
        PORT_HAL_SetMuxMode(g_portBaseAddr[0],12u,kPortPinDisabled);
        /* PTA2 CMP0 output */
        PORT_HAL_SetMuxMode(g_portBaseAddr[0],2u,kPortMuxAlt2);
        break;
    default:
        break;
    }
}

void configure_swd_pins(uint32_t instance)
{
    switch(instance) {
      case 0:
          /* PTA2 SWD_DIO */
          PORT_HAL_SetMuxMode(g_portBaseAddr[0],2u,kPortMuxAlt3);
          break;
      default:
          break;
    }
}

void configure_adc_pins(uint32_t instance)
{
    switch(instance) {
      case 0:
          /* PTB0 - Ain0*/
          PORT_HAL_SetMuxMode(g_portBaseAddr[1],0u,kPortPinDisabled);
          /* PTA9 - Ain1*/
          PORT_HAL_SetMuxMode(g_portBaseAddr[0],9u,kPortPinDisabled);
          break;
      default:
          break;
    }
}

/* END pin_mux. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
