
//#include "fsl_common.h"
#include <stdio.h>
#include "user.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"

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

#define EXAMPLE_I2C_SLAVE_BASEADDR I2C0
#define I2C_SLAVE_CLK_SRC I2C0_CLK_SRC
#define I2C_SLAVE_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_DATA_LENGTH 34U

uint8_t g_slave_buff[I2C_DATA_LENGTH];
i2c_slave_handle_t g_s_handle;
volatile bool g_SlaveCompletionFlag = false;

u8 user_isIICRecDone(void)
{
	return g_SlaveCompletionFlag;
}

void user_setIICRecDone(u8 isDone)
{
	g_SlaveCompletionFlag = isDone ? 1:0;
}

static void i2c_slave_callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *userData)
{
    switch (xfer->event)
    {
        /*  Address match event */
        case kI2C_SlaveAddressMatchEvent:
            xfer->data = NULL;
            xfer->dataSize = 0;
            break;
        /*  Transmit request */
        case kI2C_SlaveTransmitEvent:
            /*  Update information for transmit process */
            xfer->data = &g_slave_buff[2];
            xfer->dataSize = g_slave_buff[1];
            break;

        /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            /*  Update information for received process */
            xfer->data = g_slave_buff;
            xfer->dataSize = I2C_DATA_LENGTH;
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            g_SlaveCompletionFlag = true;
            xfer->data = NULL;
            xfer->dataSize = 0;
            break;

        default:
            g_SlaveCompletionFlag = false;
            break;
    }
}

void user_iicSlaveInit(void)
{
	i2c_slave_config_t slaveConfig;
	BOARD_I2C_ConfigurePins();
	PRINTF("\r\nI2C board2board interrupt example -- Slave transfer.\r\n\r\n");

	/*1.Set up i2c slave first*/
	/*
	 * slaveConfig->addressingMode = kI2C_Address7bit;
	 * slaveConfig->enableGeneralCall = false;
	 * slaveConfig->enableWakeUp = false;
	 * slaveConfig->enableBaudRateCtl = false;
	 * slaveConfig->enableSlave = true;
	 */
	I2C_SlaveGetDefaultConfig(&slaveConfig);

	slaveConfig.addressingMode = kI2C_Address7bit;
	slaveConfig.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
	slaveConfig.upperAddress = 0; /*  not used for this example */

	I2C_SlaveInit(EXAMPLE_I2C_SLAVE_BASEADDR, &slaveConfig, I2C_SLAVE_CLK_FREQ);

	memset(g_slave_buff, 0, sizeof(g_slave_buff));
	memset(&g_s_handle, 0, sizeof(g_s_handle));

	I2C_SlaveTransferCreateHandle(EXAMPLE_I2C_SLAVE_BASEADDR, &g_s_handle, i2c_slave_callback, NULL);
}

void user_iicSlaveRec(void)
{
	I2C_SlaveTransferNonBlocking(EXAMPLE_I2C_SLAVE_BASEADDR, &g_s_handle,
								 kI2C_SlaveCompletionEvent | kI2C_SlaveAddressMatchEvent);
}

u8 user_iicSlaveRead(u8 *buff)
{
	memcpy(buff, g_slave_buff + 2, g_slave_buff[1]);
	return g_slave_buff[1];
}

