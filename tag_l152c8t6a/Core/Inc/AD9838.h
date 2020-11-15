#ifndef ___AD9838__H
#define ___AD9838__H

#define PIN_CONTROL  1
#define SIN_OUTPUT 1
#define COMPARE_ONBOARD 1



#define Triangle_Wave    0x2002 
#define WaveSetting  (0x2008 | (PIN_CONTROL <<  9)| (SIN_OUTPUT <<  5) | (COMPARE_ONBOARD <<  4))
//#define Sine_Wave  0x2230 

//#define Square_Wave 0x2028

#define Square_Wave 0x2228
//#define Square_Wave 0x2238


#define FREQ_0      0 
#define FREQ_1      1 

void AD9838_Init(); 
void AD9838_Select_Wave(unsigned int initdata);
void AD9838_Set_Freq(unsigned char freq_number, unsigned long freq); 
void AD9838_Write_16Bits(unsigned int data);
void delay_n(int time);




#endif