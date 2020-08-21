
#ifndef ___USER__H
#define ___USER__H

#include "fsl_common.h"
#include "fsl_lpuart.h"



#define MAX_ADC_COUNT  (25)
#define ADC_PACK_LEN   (20) // header(7) + payload(12) + check_bit(1) = 20
#define ADC_HEADER_LEN (7)
#define ADC_DATA_LEN   (12)

#define ADC_HEADER  (0xAE >> 1)

// sensor
#define SENSOR_DATA_LEN (6)

// fsk
// crc is default enable
#define FSK_PREAMBLE (0XAA)
#define FSK_PREAMBLE_LEN (5)
#define FSK_SYNC_WORD (0X00C194C1)
#define FSK_SYNC_LEN (3)
#define FSK_PAYPLOAD_LEN (6)
#define FSK_CRC_LEN (2)
#define FSK_FRAME_LEN (FSK_PREAMBLE_LEN + FSK_SYNC_LEN + 1 + FSK_PAYPLOAD_LEN + FSK_CRC_LEN) // 5() + 3 + 1 + 6 + 2


#define n2s16(x) (((x & 0xff00) >> 8) | ((x & 0x00ff) << 8))

typedef uint8_t  u8 ;
typedef int8_t   s8 ;
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

u16 CRC_calc(u8 *buffer, u8 bufferLength);
s8 user_whitening(u8 *bufferIn, u8 lenIn, u8 *bufferOut);
u8 user_fskFrame(u8 *fskBuff, u8 fskBufSize, u8 *data, u8 dataLen);

#endif


