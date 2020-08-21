
//#include "fsl_common.h"
#include <stdio.h>
#include "user.h"


static DATA data;
DATA *data_getData(void)
{
    return &data;
}

/*
 * to judge whether the queue is full
 */
USER_BOOL data_isadcQueueFull(void)
{
    return (USER_BOOL)(data.adc_queue.front == (data.adc_queue.rear + 1) % MAX_ADC_COUNT);
}

/*
 * to judge whether the queue is empty
 */
USER_BOOL data_isadcQueueEmpty(void)
{
    return (USER_BOOL)(data.adc_queue.front == data.adc_queue.rear);
}

/*
 * put a adc in the queue
 */
USER_BOOL data_enqueueadc(ADC_DATA* adc)
{
    if (data_isadcQueueFull())
    {
        return FALSE;
    }

    data.adc_queue.adc[data.adc_queue.rear].adcValue = adc->adcValue;

    data.adc_queue.rear = (data.adc_queue.rear + 1) % MAX_ADC_COUNT;

    return TRUE;
}


/*
 * get a adc in the queue
 */
USER_BOOL data_dequeueadc(ADC_DATA* adc)
{
    if (!adc)
    {
        return FALSE;
    }

    if (data_isadcQueueEmpty())
    {
        return FALSE;
    }

    adc->adcValue = data.adc_queue.adc[data.adc_queue.front].adcValue;

    data.adc_queue.front = (data.adc_queue.front + 1) % MAX_ADC_COUNT;

    return TRUE;
}


/*
 * lpuart
 */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
//lpuart_handle_t g_lpuartHandle;
//uint8_t g_tipString[] = "LPUART RX ring buffer example\r\nSend back received data\r\nEcho every 8 types\r\n";
//uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */

uint8_t g_rxBuffer[ECHO_BUFFER_SIZE] = {0}; /* Buffer for receive data to echo. */
uint8_t g_txBuffer[ECHO_BUFFER_SIZE] = {0}; /* Buffer for send data to echo. */
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

void uart_init()
{


}


// crc
u16 CRC_calc(u8 *buffer, u8 bufferLength)
{
    u16 crc = 0x1D0F;
    u16 poly = 0x1021;

    u8 i;
    for(i = 0; i < bufferLength; i++)
    {
        u8 data = buffer[i];
        u8 j;
        for(j = 0; j < 8; j++)
        {
            if( (( (crc & 0x8000) >> 8 ) ^ (data & 0x80)) != 0 )
            {
                crc <<= 1;
                crc ^= poly;
            }
            else
            {
                crc <<= 1;
            }
            data <<= 1;
        }
    }
    return (u16)(~crc);
}

//
static u8 array_lfsr[] = {
//		11111111 10000111 10111000 01011001
//		10110111 10100001 11001100 00100100
//		01010111 01011110 01001011 10011100
//		00001110 11101001 11101010 01010000
//		00101010 10111110

		0xff, 0x87, 0xb8, 0x59,
		0xb7, 0xa1, 0xcc, 0x24,
		0x57, 0x5e, 0x4b, 0x9c,
		0x0e, 0xe9, 0xea, 0x50,
		0x2a, 0xbe
};

// I/O buffer size should be same
s8 user_whitening(u8 *bufferIn, u8 lenIn, u8 *bufferOut)
{
	if(lenIn > sizeof(array_lfsr))
	{
		// data is oversized
		return -1;
	}

	u8 i = 0;
	for(i = 0; i < lenIn; i++)
	{
		bufferOut[i] = bufferIn[i] ^ array_lfsr[i];
	}
	return 0;
}


u8 user_fskFrame(u8 *fskBuff, u8 fskBufSize, u8 *data, u8 dataLen)
{
	if(dataLen != FSK_PAYPLOAD_LEN || fskBufSize < FSK_FRAME_LEN)
	{
		// wrong dataLen
		return 1;
	}

    u8 *pdata = fskBuff;

    memset(pdata, FSK_PREAMBLE, FSK_PREAMBLE_LEN);
    pdata += FSK_PREAMBLE_LEN;

    u32 temp = FSK_SYNC_WORD;
    memcpy(pdata, &temp, FSK_SYNC_LEN);
    pdata += FSK_SYNC_LEN;

	u8 dataBuff[2][FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN] = {FSK_PAYPLOAD_LEN};
	memcpy(dataBuff[0] + 1, data, FSK_PAYPLOAD_LEN);

	temp = n2s16(CRC_calc(dataBuff, FSK_PAYPLOAD_LEN + 1));
	memcpy(&dataBuff[0][0]+FSK_PAYPLOAD_LEN + 1, &temp, FSK_CRC_LEN);

	user_whitening(dataBuff[0], FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN, dataBuff[1]);
	memcpy(pdata, dataBuff[1], FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN);

	return 0;
}

