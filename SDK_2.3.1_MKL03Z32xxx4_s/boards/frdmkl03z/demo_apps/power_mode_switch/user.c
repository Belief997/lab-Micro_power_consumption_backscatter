
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

/* I2C source clock */
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define EXAMPLE_I2C_MASTER_BASEADDR I2C0

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
i2c_master_handle_t g_m_handle;
volatile bool g_MasterCompletionFlag = false;
static i2c_master_transfer_t IIC_masterXfer;


static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

u8 user_isIICSendDone(void)
{
	return g_MasterCompletionFlag;
}

void user_setIICSendDone(u8 isDone)
{
	g_MasterCompletionFlag = isDone ? 1:0;
}

void user_i2c_init()
{
	// iic init
	i2c_master_transfer_t *pIIC_masterXfer = &IIC_masterXfer;
	BOARD_I2C_ConfigurePins();
	i2c_master_config_t masterConfig;
	uint32_t sourceClock;
	i2c_master_transfer_t *pmasterXfer;
	pmasterXfer = pIIC_masterXfer;
	PRINTF("\r\nI2C board2board interrupt example -- Master transfer.\r\n");

	/* Set up i2c master to send data to slave*/

	/*
	 * masterConfig->baudRate_Bps = 100000U;
	 * masterConfig->enableStopHold = false;
	 * masterConfig->glitchFilterWidth = 0U;
	 * masterConfig->enableMaster = true;
	 */
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = I2C_BAUDRATE;

	sourceClock = I2C_MASTER_CLK_FREQ;

	I2C_MasterInit(EXAMPLE_I2C_MASTER_BASEADDR, &masterConfig, sourceClock);

	memset(&g_m_handle, 0, sizeof(g_m_handle));
	memset(pmasterXfer, 0, sizeof(i2c_master_transfer_t));

	/* subAddress = 0x01, data = g_master_txBuff - write to slave.
	  start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
	uint8_t deviceAddress = 0x01U;
	pmasterXfer->slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;//  <<1
	pmasterXfer->direction = kI2C_Write;
	pmasterXfer->subaddress = (uint32_t)deviceAddress;
	pmasterXfer->subaddressSize = 1;
//		pmasterXfer->data = g_master_txBuff;
//		pmasterXfer->dataSize = I2C_DATA_LENGTH;
	pmasterXfer->flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
}


void user_iicSend(u8 *pbuff, u8 dataSize)
{
	g_master_txBuff[0] = dataSize;
	memcpy(g_master_txBuff + 1, pbuff, dataSize);
	IIC_masterXfer.data = g_master_txBuff;
	IIC_masterXfer.dataSize = dataSize + 1;
	I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &IIC_masterXfer);

}


