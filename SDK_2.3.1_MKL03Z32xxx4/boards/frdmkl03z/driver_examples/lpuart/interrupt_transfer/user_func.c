//#include "fsl_common.h"
#include "user_func.h"

//#include "fsl_lpuart.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_lptmr.h"
// Common
extern volatile float adc_high;
extern volatile float adc_low;
extern volatile float adc_value;


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


#ifdef EN_UART
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
#endif


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


//    {
//        //
//        config.pinDirection = kGPIO_DigitalOutput;
//        config.outputLogic = GPIO_H;
//        GPIO_PinInit(GPIOB, 1, &config);
//    }

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

    // MOD CTRL
    config.pinDirection = kGPIO_DigitalInput;
	config.outputLogic = GPIO_L;
	GPIO_PinInit(GPIOB, PIN_MOD, &config);
}

// 10 bit data, send 12 bit, MSB first
#define DAC_BIT_NUM 16
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
#if DAC_BIT_NUM == 12
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
#elif DAC_BIT_NUM == 16
	for(i=0; i < 16; i++)
	{
		CLK_CLR;
		//
		if( (data >> (15 - i)) & 0x01)
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
#else
#error "dac_bit_num"
#endif
	CLK_CLR;
	CS_SET;
	delay_n(1);
	CLK_SET;
	DIN_SET;
}

//
void dac_setVol(float Vol_mV, float shift)
{
	float Vref_mV = 2048.f;
	u16 daData = 0;

	Vol_mV += shift;
	if(Vol_mV > 2 * Vref_mV)
	{
		daData = 1024 - 1; // 1023 10bit 0x03ff
	}
	else if(Vol_mV > 0)
	{
		daData = Vol_mV  * 1024.f / (2.f * Vref_mV);
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
//	while(1)
//	{
//		dac_setVol(2708);
//		delay_n(20);
//	}
//	dac_setVol(2708);
//	dac_setVol(2708);
//	dac_setVol(2708);
}

// Timer

//#define TIMER_DEBUG
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT 250000U

//#define TIMER_DEBUG
#ifdef TIMER_DEBUG
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()
#endif

void LPTMR_HANDLER(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);

    static u8 cnt = 0;
//    LED_TOGGLE();
    // input high: output square, input low: output power
    if(GPIO_PinRead(GPIOB, PIN_MOD))
    {
		if(++cnt % 2 == 0)
		{
//	    	LED_TOGGLE();
			adc_value = adc_low;
			if(cnt % 4 == 0)
			{
//				LED_TOGGLE();
				adc_value = adc_high;
			}
		}
    }
    else
    {
    	adc_value = adc_high;
    }

    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}

void timer_init(void)
{

    lptmr_config_t lptmrConfig;

//    LED_INIT();

    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);
//    lptmrConfig¡£prescalerClockSource = kLPTMR_PrescalerClock_0;

    /* Initialize the LPTMR */
    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

    /*
     * Set timer period.
     * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
    */
    LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(DEMO_LPTMR_IRQn);

    /* Start counting */
    LPTMR_StartTimer(DEMO_LPTMR_BASE);
}
