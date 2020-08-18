
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




// iic


