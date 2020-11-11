#include "main.h"
#include "AD9838.h"

#define AD9838_Control_Port  GPIOB
#define AD9838_RESET  GPIO_PIN_14		
#define AD9838_SDATA  GPIO_PIN_12 		
#define AD9838_SCLK   GPIO_PIN_2
#define AD9838_FSEL   GPIO_PIN_15

#define AD9838_FSYNC_Port  GPIOA
#define AD9838_FSYNC  GPIO_PIN_1   


#define AD9838_FSYNC_SET   HAL_GPIO_WritePin(AD9838_FSYNC_Port ,AD9838_FSYNC, GPIO_PIN_SET) 
#define AD9838_FSYNC_CLR   HAL_GPIO_WritePin(AD9838_FSYNC_Port ,AD9838_FSYNC, GPIO_PIN_RESET) 
#define AD9838_SCLK_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SCLK, GPIO_PIN_SET) 
#define AD9838_SCLK_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SCLK, GPIO_PIN_RESET) 
#define AD9838_SDATA_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SDATA, GPIO_PIN_SET) 
#define AD9838_SDATA_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SDATA, GPIO_PIN_RESET) 
#define AD9838_RESET_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_RESET, GPIO_PIN_SET) 
#define AD9838_RESET_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_RESET, GPIO_PIN_RESET) 


void delay_n(int time)
{
	int i = 0;
	while(i++<20*time);
	
	
}


void AD9838_Write_16Bits(unsigned int data) 
{ 
    unsigned char i = 0 ; 

    AD9838_SCLK_SET ; 
    AD9838_FSYNC_CLR ; 

    for(i=0 ;i<16 ;i++)      
		{ 
        if(data & 0x8000) 
          AD9838_SDATA_SET ; 
        else 
          AD9838_SDATA_CLR ; 

        AD9838_SCLK_CLR ; 
        data <<= 1 ; 
        AD9838_SCLK_SET ; 
    } 
    AD9838_SDATA_SET ; 
    AD9838_FSYNC_SET ; 
} 



void AD9838_Select_Wave(unsigned int initdata) 
{ 
    AD9838_FSYNC_SET; 
    AD9838_SCLK_SET; 
    AD9838_RESET_SET;  
    AD9838_RESET_CLR;
    AD9838_Write_16Bits(initdata); 
} 


void AD9838_Init() 
{
		AD9838_RESET_SET;
AD9838_SCLK_CLR ; 	

	AD9838_FSYNC_SET;	
		AD9838_SCLK_SET;
		AD9838_SDATA_SET;
		AD9838_RESET_CLR;

	
	
		AD9838_Write_16Bits(0x2100);
		AD9838_Write_16Bits(0x2038);
		AD9838_Write_16Bits(0XC000);
		AD9838_Write_16Bits(0x2100);
} 

#define AD9838_SYSTEM_COLCK     16000000UL //16MHzÎªAD9838Ö÷Ê±ÖÓ

void AD9838_Set_Freq(unsigned char freq_number, unsigned long freq) 
{
    unsigned long FREQREG = (unsigned long)(268435456.0/AD9838_SYSTEM_COLCK*freq); 
    unsigned int FREQREG_LSB_14BIT = (unsigned int)FREQREG; 
    unsigned int FREQREG_MSB_14BIT = (unsigned int)(FREQREG>>14); 
	
    if(freq_number == FREQ_0) 
    { 
        FREQREG_LSB_14BIT &= ~(1U<<15); 
        FREQREG_LSB_14BIT |= 1<<14; 
        FREQREG_MSB_14BIT &= ~(1U<<15); 
        FREQREG_MSB_14BIT |= 1<<14; 
    }
    else
    {
        FREQREG_LSB_14BIT &= ~(1<<14); 
        FREQREG_LSB_14BIT |= 1U<<15; 
        FREQREG_MSB_14BIT &= ~(1<<14); 
        FREQREG_MSB_14BIT |= 1U<<15; 
    } 
    AD9838_Write_16Bits(FREQREG_LSB_14BIT); 
    AD9838_Write_16Bits(FREQREG_MSB_14BIT);      
} 

