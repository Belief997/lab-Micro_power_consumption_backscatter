/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#ifndef __ILI9341_H
#define __ILI9341_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* some colors */
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430

#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458
 
#define LIGHTGREEN     	 0X841F
#define LIGHTGRAY        0XEF5B
#define LGRAY 			 0XC618

#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12

#define ILI9341_CS_HIGH()       GPIO_HAL_SetPinOutput(GPIOB_BASE,6)
#define ILI9341_CS_LOW()        GPIO_HAL_ClearPinOutput(GPIOB_BASE,6)
#define ILI9341_CLK_HIGH()      GPIO_HAL_SetPinOutput(GPIOB_BASE,0)
#define ILI9341_CLK_LOW()       GPIO_HAL_ClearPinOutput(GPIOB_BASE,0)
#define ILI9341_MOSI_HIGH()      GPIO_HAL_SetPinOutput(GPIOA_BASE,7)
#define ILI9341_MOSI_LOW()       GPIO_HAL_ClearPinOutput(GPIOA_BASE,7)
#define ILI9341_MISO_HIGH()      GPIO_HAL_SetPinOutput(GPIOA_BASE,6)
#define ILI9341_MISO_LOW()       GPIO_HAL_ClearPinOutput(GPIOA_BASE,6)

#define MDM2802_BLC_HIGH()       GPIO_HAL_SetPinOutput(GPIOB_BASE,13)
#define MDM2802_BLC_LOW()        GPIO_HAL_ClearPinOutput(GPIOB_BASE,13)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef struct
{
    char* mame;
    uint8_t  x_size;
    uint8_t  y_size;
    const char* data;
}chgui_font_t;

/*******************************************************************************
 * API
 ******************************************************************************/
void ili9341_init(void);
void ili9341_draw_pixel(int c, int x, int y);
void ili9341_clear(int c);
int GUI_printf(int x, int y, const char *format,...);

#endif
/********************************************************************/
