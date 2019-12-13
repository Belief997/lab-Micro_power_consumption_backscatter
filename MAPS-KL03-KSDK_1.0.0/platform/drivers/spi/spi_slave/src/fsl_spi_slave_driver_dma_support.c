/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <assert.h>
#include "fsl_spi_common.h"
#include "fsl_spi_slave_driver.h"
#include "fsl_spi_shared_function.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_dma_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! SPI slave constants */
enum _spi_slave_constants
{
    kEmptyChar = 0,                        /*!< Empty character */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_byteToSendSlave;  /* Word to send, if no send buffer, this variable is used
                                      as the word to send, which should be initialized to 0. Needs
                                      to be static and stored in data section as this variable
                                      address is the DMA source address if no source buffer.  */

/*******************************************************************************
 * Code
 ******************************************************************************/

typedef void (*spi_slave_done_t)(void);

void SPI_DRV_SlaveDmaCallback(void *param, dma_channel_status_t chanStatus);

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_SlaveDmaCallback
 * Description   : This function is called when the DMA generates an interrupt.
 * The DMA generates an interrupt when the channel is "done", meaning that the
 * expected number of bytes have been transferred.  When the interrupt occurs,
 * the DMA will jump to this callback as it was registered in the DMA register
 * callback service function.  The user will define their own callback function
 * to take whatever action they deem necessary for handling the end of a transfer.
 * For example, the user may simply want their callback function to set a global
 * flag to indicate that the transfer is complete.  The user defined callback
 * is passed in through the "param" parameter.
 * The parameter chanStatus is currently not used.
 *
 *END**************************************************************************/
void SPI_DRV_SlaveDmaCallback(void *param, dma_channel_status_t chanStatus)
{
    spi_slave_done_t doneFunction;

    /* Due to MISRA 11.1 rule:
     * Conversions shall not be performed between a pointer to a function
     * and any type other than an integral type.
     * We first have to typecast "param" as a uint32_t before typecasting as
     * a void function pointer.
     */
    uint32_t paramAsInt = (uint32_t)(param);

    if (param != NULL)
    {
        /* Define "doneFunction" as a pointer to the user defined callback function passed
         * in through the parameter "param".  Because param is a void pointer, we need to
         * typecast it as a function pointer.  The typedef definition of the function pointer
         * is defined previously as "typedef void (*spi_slave_done_t)(void);"
         * However, becasue of MISRA 11.1, we had to typecast param as an integral so that it
         * is re-named paramAsInt
         */
        doneFunction = (spi_slave_done_t)(paramAsInt);
        doneFunction(); /* Jump to the user defined callback function */
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_SlaveInitDma
 * Description   : Initialize a SPI instance for slave mode operation with DMA support.
 * This function is exactly like the SPI_DRV_SlaveInit function but in addition, adds DMA support.
 * This function saves the callbacks to the run-time state structure for later use in the
 * interrupt handler. However, unlike the CPU driven slave driver, there is no need to define
 * callbacks for the data sink or data source since the user will simply pass in buffers
 * for the send and receive data, and the DMA engine will use those buffers.
 * An onError callback is needed to service potential errors seen during a TX FIFO underflow or
 * RX FIFO overflow.
 * The user also passes in a user defined callback for handling end of transfers of type
 * spi_slave_done_t.
 * These callbacks are set in the spi_slave_callbacks_t structure which is part of the
 * spi_slave_user_config_t structure.  See example below.
 * This function also ungates the clock to the SPI module, initializes the SPI
 * for slave mode, enables the module and corresponding interrupts and sets up the DMA channels.
 * Once initialized, the SPI module is configured in slave mode and ready to receive data from a
 * SPI master. The following is an example of how to set up the spi_slave_state_t and the
 * spi_slave_user_config_t parameters and how to call the SPI_DRV_SlaveInit function by passing
 * in these parameters:
 *   instance = slaveInstance; <- the desired module instance number
 *   spi_slave_state_t spiSlaveState; <- the user simply allocates memory for this struct
 *   spi_slave_user_config_t slaveUserConfig;
 *   slaveUserConfig.callbacks.onError = on_error; <- set to user implementation of function
 *   slaveUserConfig.callbacks.spi_slave_done_t = spiDmaDone; <- user defined callback
 *   slaveUserConfig.dataConfig.clkPhase = kSpiClockPhase_FirstEdge; <- example setting
 *   slaveUserConfig.dataConfig.clkPolarity = kSpiClockPolarity_ActiveHigh; <- example setting
 *   sendBuffer <- (pointer) to the source data buffer, can be NULL
 *   receiveBuffer <- (pointer) to the receive data buffer, can be NULL
 *   transferCount <- number of bytes to transfer
 *
 *   SPI_DRV_SlaveInitDma(slaveInstance, &slaveUserConfig, &spiSlaveState,
 *                         &sendBuffer, &receiveBuffer, transferCount);
 *
 *END**************************************************************************/
spi_status_t SPI_DRV_SlaveInitDma(uint32_t instance,
                                  spi_slave_state_t * spiState,
                                  const spi_slave_user_config_t * slaveConfig,
                                  const uint8_t * sendBuffer,
                                  uint8_t * receiveBuffer,
                                  size_t transferByteCount)
{
    assert(slaveConfig);
    assert(instance < HW_SPI_INSTANCE_COUNT);

    uint32_t spiBaseAddr = g_spiBaseAddr[instance];
    uint32_t dmaBaseAddr;
    spi_status_t errorCode = kStatus_SPI_Success;

    /* Clear the run-time state struct for this instance. */
    memset(spiState, 0, sizeof(* spiState));

    /* Initialize s_byteToSendSlave */
    s_byteToSendSlave = 0;

    /* Save the application info. */
    spiState->callbacks = slaveConfig->callbacks;

    /* Set the DSPI run-time state struct flag to indicate it will use the DMA */
    spiState->useDma = true;

    /* Enable clock for DSPI */
    CLOCK_SYS_EnableSpiClock(instance);

    /* Reset the SPI module, which also disables the SPI module */
    SPI_HAL_Init(spiBaseAddr);

    /* Set to slave mode.*/
    SPI_HAL_SetMasterSlave(spiBaseAddr, kSpiSlave);

    /* Configure the slave clock polarity, phase and data direction */
    SPI_HAL_SetDataFormat(spiBaseAddr, slaveConfig->polarity, slaveConfig->phase, slaveConfig->direction);

    /* SPI system enable */
    SPI_HAL_Enable(spiBaseAddr);

    /* Configure IRQ state structure, so irq handler can point to the correct state structure */
    g_spiStatePtr[instance] = spiState;

    /* Only need to set the DMA reg base addr once since it should be the same for all code */
    dmaBaseAddr = g_dmaRegBaseAddr[spiState->dmaReceive.channel/FSL_FEATURE_DMA_DMAMUX_CHANNELS];

    /*************************************************************************
     * Set up DMA channel for RX FIFO only if user passes in a receive buffer
     *************************************************************************/
    if (receiveBuffer)
    {
        /***********************************
         * Request DMA channels for RX FIFO
         * This channel transfers data from RX FIFO to receiveBuffer
         ***********************************/
        if (instance == 0)
        {
            DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI0Rx, &spiState->dmaReceive);
        }
        else
        {
            DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI1Rx, &spiState->dmaReceive);
        }

        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA_DRV_ConfigTransfer(&spiState->dmaReceive, kDmaPeripheralToMemory, 1,
                               (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                               (uint32_t)(receiveBuffer),
                               (uint32_t)(transferByteCount));

        /* Enable the cycle steal mode which forces a single read/write transfer per request */
        DMA_HAL_SetCycleStealCmd(dmaBaseAddr, spiState->dmaReceive.channel, true);

        /* Enable the DMA peripheral request */
        DMA_DRV_StartChannel(&spiState->dmaReceive);

        /* Set up DMA interrupt handler */
        /* Due to MISRA 11.1 rule:
         * Conversions shall not be performed between a pointer to a function
         * and any type other than an integral type.
         * We first have to typecast the callback function pointer as a uint32_t before typecasting
         * as a void pointer.
         */
        uint32_t userCallbackInt = (uint32_t)(slaveConfig->callbacks.spi_slave_done_t);
        /* Register callback for DMA interrupt */
        DMA_DRV_RegisterCallback(&spiState->dmaReceive, SPI_DRV_SlaveDmaCallback,
                                 (void *)(userCallbackInt));

        /* Enable the SPI RX DMA Request */
        SPI_HAL_SetRxDmaCmd(spiBaseAddr, true);

    }

    /*************************************************************************
     * Set up DMA channel for TX FIFO only if user passes in a send buffer
     *************************************************************************/
   /***********************************
     * Request DMA channel for TX FIFO
     ***********************************/
    if (instance == 0)
    {
        DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI0Tx, &spiState->dmaTransmit);
    }
    else
    {
        DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI1Tx, &spiState->dmaTransmit);
    }

    /* Per the reference manual, before enabling the SPI transmit DMA request, we first need
     * to read the status register and then write to the SPI data register.  Afterwards, we need
     * to decrement the sendByteCount and perform other driver maintenance functions.
     *
     * At this point, get the data ready to send and then set up the Tx DMA channel. Once the DMA
     * channel is set up (with the proper remaining send byte count), then go ahead and write
     * the data into the SPI data register to transmit.
     */

    /* Read the SPI Status register */
    SPI_HAL_IsTxBuffEmptyPending(spiBaseAddr);

    /* Start the transfer by writing the first byte. If a send buffer was provided, the byte
     * comes from there. Otherwise we just send a zero byte. The actual write to the SPI data
     * register takes place after the Tx DMA channel is set up.
     */
    if (sendBuffer)
    {
        s_byteToSendSlave = *(sendBuffer);
        sendBuffer++;
    }
    --transferByteCount; /* Decrement the send byte count for use in DMA setup */

    /* If there is a send buffer, data comes from there, else send 0 */
    if (sendBuffer)
    {
        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA_DRV_ConfigTransfer(&spiState->dmaTransmit, kDmaMemoryToPeripheral, 1,
                               (uint32_t)(sendBuffer),
                               (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                               (uint32_t)(transferByteCount));
    }
    else
    {
        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA_DRV_ConfigTransfer(&spiState->dmaTransmit, kDmaMemoryToPeripheral, 1,
                               (uint32_t)(&s_byteToSendSlave),
                               (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                               (uint32_t)(transferByteCount));

        /* Now clear SINC since we are only sending zeroes */
        DMA_HAL_SetSourceIncrementCmd(dmaBaseAddr, spiState->dmaTransmit.channel, false);
    }

    /* Enable the cycle steal mode which forces a single read/write transfer per request */
    DMA_HAL_SetCycleStealCmd(dmaBaseAddr, spiState->dmaTransmit.channel, true);

    /* If there is a receive buffer then the receive DMA channel was setup including
     * enabling the DMA interrupt, therefore disable the interrupt for the transmit chan.
     */
    if (receiveBuffer)
    {
        /* Now, disable the TX chan interrupt since it is enable by the receive DMA */
        DMA_HAL_SetIntCmd(dmaBaseAddr, spiState->dmaTransmit.channel, false);
    }
    /* Else, keep the transmit channel interrupt enabled and install callback */
    else
    {
        /* Set up DMA interrupt handler */
        /* Due to MISRA 11.1 rule:
         * Conversions shall not be performed between a pointer to a function
         * and any type other than an integral type.
         * We first have to typecast the callback function pointer as a uint32_t before typecasting
         * as a void pointer.
         */
        uint32_t userCallbackInt = (uint32_t)(slaveConfig->callbacks.spi_slave_done_t);
        /* Register callback for DMA interrupt */
        DMA_DRV_RegisterCallback(&spiState->dmaTransmit, SPI_DRV_SlaveDmaCallback,
                                 (void *)(userCallbackInt));
    }

    /* Enable the DMA peripheral request */
    DMA_DRV_StartChannel(&spiState->dmaTransmit);

    /* Once the Tx DMA channel is set up, send the data and then enable the DMA request */
    SPI_HAL_WriteData(spiBaseAddr, s_byteToSendSlave);

    /* Enable the SPI TX DMA Request */
    SPI_HAL_SetTxDmaCmd(spiBaseAddr, true);

    return errorCode;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

