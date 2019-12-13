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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "ili9341.h"
#include "board.h"
#include "SimSun6x12.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CLR_CLK     ILI9341_CLK_LOW
#define SET_CLK     ILI9341_CLK_HIGH
#define SET_SDA     ILI9341_MOSI_HIGH
#define CLR_SDA     ILI9341_MOSI_LOW
#define CLR_CS      ILI9341_CS_LOW
#define SET_CS      ILI9341_CS_HIGH

/*******************************************************************************
 * Variables
 ******************************************************************************/

static chgui_font_t _gFontTbl[] = 
{
    {"SimSun", FONT_SimSun6x12_XSize, FONT_SimSun6x12_YSize, FONT_SimSun6x8 },
};

/*FUNCTION*********************************************************************
 *
 * Function Name : send_byte
 * Description   : send one byte
 *    
 *
 *END*************************************************************************/
static inline  void send_byte(uint8_t data)    
{  
    uint8_t count;  
	
    for(count = 0; count < 8; count++)  
    { 
        CLR_CLK();
        if(data & 0x80)
        {
            SET_SDA(); 
        }     
        else
        {
            CLR_SDA();   
        }
		data <<= 1;    
		SET_CLK();   		
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : write_resiger
 * Description   : write lcd controller's register value
 *    
 *
 *END*************************************************************************/
static inline void write_register(uint8_t cmd)	  
{ 
	CLR_CS(); 
	CLR_CLK();
	CLR_SDA();
	SET_CLK(); 
	send_byte(cmd);
	SET_CS();         
}

/*FUNCTION*********************************************************************
 *
 * Function Name : write_data
 * Description   : write lcd controller's GRAM
 *    
 *
 *END*************************************************************************/
static inline void write_data(uint8_t data)	  
{ 	 
    CLR_CS(); 
	CLR_CLK();
	SET_SDA();
	SET_CLK(); 
	send_byte(data);
	SET_CS();   
}

/*FUNCTION*********************************************************************
 *
 * Function Name : gram_prepare
 * Description   : send gram write cmd
 *    
 *
 *END*************************************************************************/
static inline void gram_prepare(void)
{
	write_register(0X2C);
}	 

/*FUNCTION*********************************************************************
 *
 * Function Name : display_on
 * Description   : set display on
 *    
 *
 *END*************************************************************************/
void display_on(void)
{					   
    write_register(0X29);
}	 

/*FUNCTION*********************************************************************
 *
 * Function Name : display_off
 * Description   : set display off
 *    
 *
 *END*************************************************************************/
void display_off(void)
{	   
    write_register(0X28);
}   

/*FUNCTION*********************************************************************
 *
 * Function Name : set_cursor
 * Description   : set gram cursor
 *    
 *
 *END*************************************************************************/
void set_cursor(uint16_t Xpos, uint16_t Ypos)
{
    write_register(0x2A);
    write_data(Xpos >> 8);
    write_data(0xFF & Xpos);

    write_register(0x2B);
    write_data(Ypos >> 8);
    write_data(0xFF & Ypos);
}  

/*FUNCTION*********************************************************************
 *
 * Function Name : ili9341_draw_pixel
 * Description   : draw a pixel
 *    
 *
 *END*************************************************************************/
void ili9341_draw_pixel(int c, int x, int y)
{
    set_cursor(x,y);
    gram_prepare();
    write_data(c >> 8);
    write_data(c); 
} 	

/*FUNCTION*********************************************************************
 *
 * Function Name : ili9341_clear
 * Description   : clear screen
 *    
 *
 *END*************************************************************************/
void ili9341_clear(int c)
{
	uint32_t index=0;      
	set_cursor(0x00, 0x0000);
	gram_prepare(); 
	for(index = 0; index < (320*240); index++) 
	{
		write_data(c >> 8);
		write_data(c);
	}
}  

/*FUNCTION*********************************************************************
 *
 * Function Name : lcdc_delay
 * Description   : delay some time
 *    
 *
 *END*************************************************************************/
static void lcdc_delay(int n)
{
    volatile int i;
    for(i=0; i<n*100; i++);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : ili9341_iniinit lcm moudle
 * Description   : init lcm moudle
 *    
 *
 *END*************************************************************************/
void ili9341_init(void)
{
    write_register(0xCF);  
    write_data(0x00); 
    write_data(0xC1); 
    write_data(0X30); 
    write_register(0xED);  
    write_data(0x64); 
    write_data(0x03); 
    write_data(0X12); 
    write_data(0X81); 
    write_register(0xE8);  
    write_data(0x85); 
    write_data(0x10); 
    write_data(0x7A); 
    write_register(0xCB);  
    write_data(0x39); 
    write_data(0x2C); 
    write_data(0x00); 
    write_data(0x34); 
    write_data(0x02); 
    write_register(0xF7);  
    write_data(0x20); 
    write_register(0xEA);  
    write_data(0x00); 
    write_data(0x00); 
    write_register(0xC0);    //Power control 
    write_data(0x1B);   //VRH[5:0] 
    write_register(0xC1);    //Power control 
    write_data(0x01);   //SAP[2:0];BT[3:0] 
    write_register(0xC5);    //VCM control 
    write_data(0x30); 	 //3F
    write_data(0x30); 	 //3C
    write_register(0xC7);    //VCM control2 
    write_data(0XB7); 
		
    write_register(0x36);    // Memory Access Control 
    write_data(0x08); 
		
    write_register(0x3A);   
    write_data(0x55); 
		
    write_register(0xB1);   
    write_data(0x00);   
    write_data(0x1A); 
		
    write_register(0xB6);    // Display Function Control 
    write_data(0x0A); 
    write_data(0xA2); 
		
    write_register(0xF2);    // 3Gamma Function Disable 
    write_data(0x00); 
    write_register(0x26);    //Gamma curve selected 
    write_data(0x01); 
		
    write_register(0xE0);    //Set Gamma 
    write_data(0x0F); 
    write_data(0x2A); 
    write_data(0x28); 
    write_data(0x08); 
    write_data(0x0E); 
    write_data(0x08); 
    write_data(0x54); 
    write_data(0XA9); 
    write_data(0x43); 
    write_data(0x0A); 
    write_data(0x0F); 
    write_data(0x00); 
    write_data(0x00); 
    write_data(0x00); 
    write_data(0x00); 		 
		
    write_register(0XE1);    //Set Gamma 
    write_data(0x00); 
    write_data(0x15); 
    write_data(0x17); 
    write_data(0x07); 
    write_data(0x11); 
    write_data(0x06); 
    write_data(0x2B); 
    write_data(0x56); 
    write_data(0x3C); 
    write_data(0x05); 
    write_data(0x10); 
    write_data(0x0F); 
    write_data(0x3F); 
    write_data(0x3F); 
    write_data(0x0F); 
    write_register(0x2B); 
    write_data(0x00);
    write_data(0x00);
    write_data(0x01);
    write_data(0x3f);
    write_register(0x2A); 
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xef);	 
    write_register(0x11); //Exit Sleep
    lcdc_delay(1000);
    write_register(0x29); //display on   
		 
    ili9341_clear(BLACK);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : _GUI_DispChar
 * Description   : display a char, internal function
 *    
 *
 *END*************************************************************************/
static void _GUI_DispChar(char c, int x, int y, const char *pdata, int font_xsize, int font_ysize, int fcolor, int bcolor)
{
    uint8_t j,pos,t;
    uint8_t temp;
    uint8_t XNum;
    uint32_t base;
    XNum = (font_xsize/8) + 1;
    if(font_ysize%8 == 0)
    {
        XNum--;
    }
    if(c < ' ')
    {
        return;
    }
    c = c - ' ';
    base = (c*XNum*font_ysize);

    for(j = 0; j < XNum; j++)
    {
        for(pos = 0; pos < font_ysize; pos++)
        {
            temp = (uint8_t)pdata[base + pos + j*font_ysize];
            if(j < XNum)
            {
                for(t = 0; t < 8; t++)
                {
                    if((temp>>t)&0x01)
                    {
                        ili9341_draw_pixel(fcolor, x+t, y+pos);
                    }
                    else
                    {
                        ili9341_draw_pixel(bcolor, x+t, y+pos);
                    }
                }
            }
            else
            {
                for(t = 0; t < font_xsize%8; t++)
                {
                    if((temp >> t) & 0x01)
                    {
                        ili9341_draw_pixel(fcolor, x+t, y+pos);
                    }
                    else
                    {
                        ili9341_draw_pixel(bcolor, x+t, y+pos);
                    }
                }
            }
        }
    x += 8;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : GUI_DispChar
 * Description   : display a char
 *    
 *
 *END*************************************************************************/
void GUI_DispChar(char c, int x, int y)
{
    _GUI_DispChar(c, x, y, _gFontTbl[0].data, _gFontTbl[0].x_size, _gFontTbl[0].y_size, 0xFFFF, 0x0000);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : GUI_printf
 * Description   : display string.
 *    
 *
 *END*************************************************************************/
int GUI_printf(int x, int y, const char *format,...)
{
    int chars;
    int i;
    va_list ap;
    char printbuffer[64];
    va_start(ap, format);
    chars = vsprintf(printbuffer, format, ap);
    va_end(ap);
    for(i = 0; i < chars; i++)
    {
        GUI_DispChar(printbuffer[i],x + i*_gFontTbl[0].x_size ,y);
    }
    return chars ;
}


/********************************************************************/

