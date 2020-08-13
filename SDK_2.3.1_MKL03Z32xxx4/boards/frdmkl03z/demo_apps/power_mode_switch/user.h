
#ifndef ___USER__H
#define ___USER__H

#include "fsl_common.h"
#include "fsl_lpuart.h"



#define MAX_ADC_COUNT  (25)
#define ADC_PACK_LEN   (20) // header(7) + payload(12) + check_bit(1) = 20
#define ADC_HEADER_LEN (7)
#define ADC_DATA_LEN   (12)

#define ADC_HEADER  (0xAE >> 1)


#define SENSOR_DATA_LEN (6)
#define SENSOR_HEADER (0XA5)
#define SENSOR_HEADER_LEN (1)
#define SENSOR_CHECK_LEN (1)
#define SENSOR_FRAME_LEN (SENSOR_HEADER_LEN + SENSOR_DATA_LEN + SENSOR_CHECK_LEN) // 1+6+1


typedef uint8_t  u8 ;
typedef uint16_t u16;
typedef uint32_t u32;

typedef enum
{
	FALSE = 0,
	TRUE,

	STATUS_WAIT_TRIG,
	STATUS_WAIT_DATA,
	STATUS_WAIT_SEND,

	REC_WAIT,
	REC_FAIL,
	REC_SUCCESS,

	SEND_IDLE,
	SEND_WAIT,
	SEND_SUCCESS,

}USER_BOOL;

typedef struct
{
	u16 adcValue;
}ADC_DATA;

typedef struct
{
    int front;
    int rear;
    ADC_DATA adc[MAX_ADC_COUNT];
}ADC_QUEUE;


typedef struct
{
	ADC_QUEUE adc_queue;

}DATA;

typedef struct{
	u32 reserve : 12;
    u32 header  : 7;
    u32 data    : 12;
    u32 check   : 1;
}ADC_PACK;


DATA *data_getData(void);
USER_BOOL data_isadcQueueFull(void);
USER_BOOL data_isadcQueueEmpty(void);
USER_BOOL data_enqueueadc(ADC_DATA* adc);
USER_BOOL data_dequeueadc(ADC_DATA* adc);



// uart
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPUART LPUART0
#define DEMO_LPUART_CLKSRC SYS_CLK
#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)

#define RX_RING_BUFFER_SIZE 20U
#define ECHO_BUFFER_SIZE 1U


void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);





#endif


