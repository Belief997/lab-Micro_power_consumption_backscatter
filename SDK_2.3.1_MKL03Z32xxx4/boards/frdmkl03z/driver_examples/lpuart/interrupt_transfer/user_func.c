//#include "fsl_common.h"
#include "user_func.h"

//#include "fsl_lpuart.h"
#include "fsl_gpio.h"

// Common

void delay(void)
{
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
}

void delay_n(u32 n)
{
    volatile uint32_t i = 0;
    for (i = 0; i < n; ++i)
    {
    	delay();
    }
}

// Uart

lpuart_handle_t g_lpuartHandle;

//uint8_t g_tipString[] =
//    "Lpuart interrupt example\r\nBoard receives 8 characters then sends them out\r\nNow please input:\r\n";

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_LPUART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_LPUART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}

// DAC
//#define CS_SET {GPIO_PortSet(GPIOA, PIN_CS);}
//#define CS_CLR {GPIO_PortClear(GPIOA, PIN_CS);}
//#define CLK_SET {GPIO_PortSet(GPIOA, PIN_CLK);}
//#define CLK_CLR {GPIO_PortClear(GPIOA, PIN_CLK);}
//#define DIN_SET {GPIO_PortSet(GPIOB, PIN_DIN);}
//#define DIN_CLR {GPIO_PortClear(GPIOB, PIN_DIN);}

#define CS_SET {GPIO_PinWrite(GPIOA, PIN_CS, GPIO_H);}
#define CS_CLR {GPIO_PinWrite(GPIOA, PIN_CS, GPIO_L);}
#define CLK_SET {GPIO_PinWrite(GPIOA, PIN_CLK, GPIO_H);}
#define CLK_CLR {GPIO_PinWrite(GPIOA, PIN_CLK, GPIO_L);}
#define DIN_SET {GPIO_PinWrite(GPIOB, PIN_DIN, GPIO_H);}
#define DIN_CLR {GPIO_PinWrite(GPIOB, PIN_DIN, GPIO_L);}

void dac_init(void)
{
    gpio_pin_config_t config = {0};

    // DOUT
    config.pinDirection = kGPIO_DigitalInput;
    config.outputLogic = GPIO_L;
    GPIO_PinInit(GPIOA, PIN_DOUT, &config);

    // CS
    config.pinDirection = kGPIO_DigitalOutput;
    config.outputLogic = GPIO_H;
    GPIO_PinInit(GPIOA, PIN_CS, &config);

    // CLK
    config.pinDirection = kGPIO_DigitalOutput;
    config.outputLogic = GPIO_H;
    GPIO_PinInit(GPIOA, PIN_CLK, &config);

    // DIN
    config.pinDirection = kGPIO_DigitalOutput;
    config.outputLogic = GPIO_H;
    GPIO_PinInit(GPIOB, PIN_DIN, &config);

}

// 10 bit data, send 12 bit, MSB first
#define DAC_MASK 0X03FF  // 10 bit
void dac_send(u16 data)
{
	u8 i = 0;
	data &= DAC_MASK;
	data <<= 2;

	CS_SET;
	CLK_SET;
	DIN_SET;
	delay_n(1);

	//
	CLK_CLR;
	delay_n(1);

	//
	CS_CLR;
	for(i=0; i < 12; i++)
	{
		CLK_CLR;
		//
		if( (data >> (11 - i)) & 0x01)
		{
			DIN_SET;
		}
		else
		{
			DIN_CLR;
		}
		delay_n(1);
		CLK_SET;
		delay_n(1);
	}
	CLK_CLR;
	CS_SET;
	delay_n(1);
	CLK_SET;
	DIN_SET;
}

void dac_setVol(float Vol_mV)
{
	float Vref_mV = 2048.f;
	u16 daData = 0;

	if(Vol_mV > 2 * Vref_mV)
	{
		daData = 1024 - 1; // 1023 10bit 0x03ff
	}
	else if(Vol_mV > 0)
	{
		daData = Vol_mV  * 1024 / (2 * Vref_mV);
	}
	else
	{
		daData = 0;
	}

	dac_send(daData);
}




void dac_test(void)
{
	CS_CLR;
	CLK_CLR;
	DIN_CLR;

	CS_SET;
	CLK_SET;

	DIN_SET;

//	while(1)
	{
		dac_send(0xa5);
//		delay_n(20);


		dac_send(0x1a5);
		dac_send(0x2a5);
		dac_send(0x3a5);
	}

	// 677
	while(1)
	{
		dac_setVol(2708);
		delay_n(20);
	}
	dac_setVol(2708);
	dac_setVol(2708);
	dac_setVol(2708);
}

