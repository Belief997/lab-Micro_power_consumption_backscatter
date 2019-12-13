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
#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__

#include "fsl_gpio_driver.h"

/*! @file */
/*!*/
/*! This file contains gpio pin definitions used by gpio peripheral driver.*/
/*! The enums in _gpio_pins map to the real gpio pin numbers defined in*/
/*! gpioPinLookupTable. And this might be different in different board.*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
/*! @brief gpio pin names.*/
/*!*/ 
/*! This should be defined according to board setting.*/
enum _gpio_pins 
{
    kGpioLED1        = GPIO_MAKE_PIN(HW_GPIOB, 7),  /* FRDM-KL03Z RGB LED Red LED */
    kGpioLED2	     = GPIO_MAKE_PIN(HW_GPIOB, 10),  /* FRDM-KL03Z RGB LED Green LED  */
    kGpioLED3        = GPIO_MAKE_PIN(HW_GPIOB, 11),  /* FRDM-KL03Z RGB LED Blue LED */
	kGpioLED4        = GPIO_MAKE_PIN(HW_GPIOB, 13),
	kGpioSW1         = GPIO_MAKE_PIN(HW_GPIOB, 5),   /* Joystick - Select */
    kGpioSW2         = GPIO_MAKE_PIN(HW_GPIOB, 7),   /* Joystick - Down */
    kGpioSW3         = GPIO_MAKE_PIN(HW_GPIOB, 10),  /* Joystick - Up */
	kGpioSW4         = GPIO_MAKE_PIN(HW_GPIOB, 11),  /* Joystick - Right */
	kGpioSW5         = GPIO_MAKE_PIN(HW_GPIOB, 13),  /* Joystick - Left */
	kGpioSpiSdCs     = GPIO_MAKE_PIN(HW_GPIOA, 9),
    kGpioSpiLcdCs    = GPIO_MAKE_PIN(HW_GPIOA, 12),
	kGpioSpicLcdCs   = GPIO_MAKE_PIN(HW_GPIOB, 6),
	kGpioSpiMemCs    = GPIO_MAKE_PIN(HW_GPIOA, 5),
    kGpiollwuWakeup  = GPIO_MAKE_PIN(HW_GPIOA, 0)
};
    
extern gpio_input_pin_user_config_t switchPins[];
extern gpio_input_pin_user_config_t llwuWakeupPins[];
extern gpio_output_pin_user_config_t ledPins[];
extern const gpio_output_pin_user_config_t spiSdCsPin[];
extern const gpio_output_pin_user_config_t spiLcdCsPin[];
extern const gpio_output_pin_user_config_t spicLcdCsPin[];
extern const gpio_output_pin_user_config_t spiMemCsPin[];

#endif /* __FSL_GPIO_PINS_H__ */
