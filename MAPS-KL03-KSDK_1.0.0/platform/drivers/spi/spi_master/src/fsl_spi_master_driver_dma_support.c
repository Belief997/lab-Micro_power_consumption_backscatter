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
#include "fsl_spi_common.h"
#include "fsl_spi_master_driver.h"
#include "fsl_dma_driver.h"
#include "fsl_spi_shared_function.h"
#include "fsl_dma_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_byteToSend;  /* Word to send, if no send buffer, this variable is used
                                 as the word to send, which should be initialized to 0. Needs
                                 to be static and stored in data section as this variable
                                 address is the DMA source address if no source buffer.  */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
spi_status_t SPI_DRV_MasterStartTransferDma(uint32_t instance,
                                            const spi_master_user_config_t * restrict device);

static void SPI_DRV_MasterCompleteTransferDma(uint32_t instance);

void SPI_DRV_DmaCallback(void *param, dma_channel_status_t chanStatus);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : DSPI_DRV_SlaveDmaCallback
 * Description   : This function is called when the eDMA generates an interrupt.
 * The eDMA generates an interrupt when the channel is "done", meaning that the
 * expected number of bytes have been transferred.  When the interrupt occurs,
 * the eDMA will jump to this callback as it was registered in the eDMA register
 * callback service function.  The user will defined their own callback function
 * to take whatever action they deem necessary for handling the end of a transfer.
 * For example, the user may simply want their callback function to set a global
 * flag to indicate that the transfer is complete.  The user defined callback
 * is passed in through the "param" parameter.
 * The parameter chanStatus is currently not used.
 *
 *END**************************************************************************/
void SPI_DRV_DmaCallback(void *param, dma_channel_status_t chanStatus)
{
    uint32_t instance;

    instance = (uint32_t)(param);

    SPI_DRV_MasterCompleteTransferDma(instance);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterInitDma
 * Description   : Initialize a SPI instance for master mode operation with DMA support.
 * This function is exactly like the spi_master_init function but in addition, adds DMA support.
 * If the user desires to use DMA based transfers, then the user should use this function call
 * instead of the spi_master_init function call.  Like the spi_master_init,
 * this function initializes the run-time state structure to track the ongoing
 * transfers, ungates the clock to the SPI module, resets the SPI module, initializes the module
 * to user defined settings and default settings, configures the IRQ state structure, enables
 * the module-level interrupt to the core, and enables the SPI module.
 *
 * This init function also configures the DMA module by requesting channels for DMA operation
 * and sets a "useDma" flag in the run-time state structure to notify transfer functions
 * to use DMA driven operations.
 *
 *END**************************************************************************/
void SPI_DRV_MasterInitDma(uint32_t instance, spi_master_state_t * spiState)
{
    SPI_DRV_MasterInit(instance, spiState);

    /* Set the SPI run-time state struct flag to indicate it will use the DMA */
    spiState->useDma = true;

    /***********************************
     * Request DMA channel for RX FIFO
     ***********************************/
    /* This channel transfers data from RX FIFO to receiveBuffer */
    if (instance == 0)
    {
        DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI0Rx, &spiState->dmaReceive);
    }
    else
    {
        DMA_DRV_RequestChannel(kDmaAnyChannel, kDmaRequestMux0SPI1Rx, &spiState->dmaReceive);
    }

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
}

/*!
 * @brief Initiate (start) a transfer using DMA. This is not a public API as it is called from
 *  other driver functions
 */
spi_status_t SPI_DRV_MasterStartTransferDma(uint32_t instance,
                                            const spi_master_user_config_t * restrict device)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    /* For temporarily storing DMA register channel */
    uint8_t txChannel, rxChannel;
    void * param;
    uint32_t calculatedBaudRate;
    uint32_t spiBaseAddr = g_spiBaseAddr[instance];
    uint32_t dmaBaseAddr;
    uint32_t dmamuxBaseAddr;

    /* Initialize s_byteToSend */
    s_byteToSend = 0;

    param = (void *)(instance); /* For DMA callback, set "param" as the SPI instance number */
    rxChannel = spiState->dmaReceive.channel;
    txChannel = spiState->dmaTransmit.channel;
    /* Only need to set the DMA reg base addr once since it should be the same for all code */
    dmaBaseAddr = g_dmaRegBaseAddr[rxChannel/FSL_FEATURE_DMA_DMAMUX_CHANNELS];
    dmamuxBaseAddr = g_dmamuxRegBaseAddr[txChannel/FSL_FEATURE_DMAMUX_MODULE_CHANNEL];

    /* If the transfer count is zero, then return immediately.*/
    if (spiState->remainingSendByteCount == 0)
    {
        /* Signal the synchronous completion object if the transfer wasn't async.
         * Otherwise, when we return the the sync function we'll get stuck in the sync wait loop.
         */
        if (!spiState->isTransferAsync)
        {
            OSA_SemaPost(&spiState->irqSync);
        }

        return kStatus_SPI_Success;
    }

    /* Check that we're not busy.*/
    if (spiState->isTransferInProgress)
    {
        return kStatus_SPI_Busy;
    }

    /* Configure bus for this device. If NULL is passed, we assume the caller has
     * preconfigured the bus using SPI_DRV_MasterConfigureBus().
     * Do nothing for calculatedBaudRate. If the user wants to know the calculatedBaudRate
     * then they can call this function separately.
     */
    if (device)
    {
        SPI_DRV_MasterConfigureBus(instance, device, &calculatedBaudRate);
    }

    /* Save information about the transfer for use by the ISR.*/
    spiState->isTransferInProgress = true;

    /* Enable the SPI module for the duration of this transfer.*/
    SPI_HAL_Enable(spiBaseAddr);

    /* Clear the read buffer if there's anything in it.*/
    if (SPI_HAL_IsReadBuffFullPending(spiBaseAddr))
    {
        uint8_t unused = SPI_HAL_ReadData(spiBaseAddr);
        unused = unused; /* Keep compiler happy.*/
    }

    /* The DONE needs to be cleared before programming the channel's TCDs for the next
     * transfer.
     */
    DMA_HAL_ClearStatus(dmaBaseAddr, rxChannel);
    DMA_HAL_ClearStatus(dmaBaseAddr, txChannel);

    /* Disable and enable the TX DMA channel at the DMA mux. Doing so will prevent an
     * inadvertent DMA transfer when the TX DMA channel ERQ bit is set after having been
     * cleared from a previous DMA transfer (clearing of the ERQ bit is automatically performed
     * at the end of a transfer when D_REQ is set).
     */
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, txChannel, false);
    DMAMUX_HAL_SetChannelCmd(dmamuxBaseAddr, txChannel, true);

    /************************************************************************************
     * Set up the RX DMA channel Transfer Control Descriptor (TCD)
     * Note, if there is no receive buffer (if user passes in NULL), then bypass RX DMA
     * set up.
     ***********************************************************************************/
    /* If a receive buffer is used */
    if (spiState->receiveBuffer)
    {
        /* Set up this channel's control which includes enabling the DMA interrupt */
        DMA_DRV_ConfigTransfer(&spiState->dmaReceive, kDmaPeripheralToMemory, 1,
                               (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                               (uint32_t)(spiState->receiveBuffer),
                               (uint32_t)(spiState->remainingReceiveByteCount));

        /* Enable the cycle steal mode which forces a single read/write transfer per request */
        DMA_HAL_SetCycleStealCmd(dmaBaseAddr, rxChannel, true);

        /* Enable the DMA peripheral request */
        DMA_DRV_StartChannel(&spiState->dmaReceive);

        /* Register callback for DMA interrupt */
        DMA_DRV_RegisterCallback(&spiState->dmaReceive, SPI_DRV_DmaCallback, param);

        /* Enable the SPI RX DMA Request */
        SPI_HAL_SetRxDmaCmd(spiBaseAddr, true);
    }

    /************************************************************************************
     * Set up the TX DMA channel Transfer Control Descriptor (TCD)
     * Note, if there is no source buffer (if user passes in NULL), then send zeros
     ***********************************************************************************/
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
    if (spiState->sendBuffer)
    {
        s_byteToSend = *(spiState->sendBuffer);
        ++spiState->sendBuffer;
    }
    --spiState->remainingSendByteCount; /* Decrement the send byte count for use in DMA setup */

    /* If there are more bytes to send, enable the TX DMA request else complete the transfer */
    if (!spiState->remainingSendByteCount)
    {
        /* Once the Tx DMA channel is set up, send the data and then enable the DMA request */
        SPI_HAL_WriteData(spiBaseAddr, s_byteToSend);

        /* Complete the transfer */
        SPI_DRV_MasterCompleteTransferDma(instance);
    }
    else
    {
        /* If there is a send buffer, data comes from there, else send 0 */
        if (spiState->sendBuffer)
        {
            /* Set up this channel's control which includes enabling the DMA interrupt */
            DMA_DRV_ConfigTransfer(&spiState->dmaTransmit, kDmaMemoryToPeripheral, 1,
                                   (uint32_t)(spiState->sendBuffer),
                                   (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                                   (uint32_t)(spiState->remainingSendByteCount));
        }
        else
        {
            /* Set up this channel's control which includes enabling the DMA interrupt */
            DMA_DRV_ConfigTransfer(&spiState->dmaTransmit, kDmaMemoryToPeripheral, 1,
                                   (uint32_t)(&s_byteToSend),
                                   (uint32_t)HW_SPI_D_ADDR(spiBaseAddr),
                                   (uint32_t)(spiState->remainingSendByteCount));

            /* Now clear SINC since we are only sending zeroes */
            DMA_HAL_SetSourceIncrementCmd(dmaBaseAddr, txChannel, false);
        }

        /* Enable the cycle steal mode which forces a single read/write transfer per request */
        DMA_HAL_SetCycleStealCmd(dmaBaseAddr, txChannel, true);

        /* If there is a receive buffer then the receive DMA channel was setup including
         * enabling the DMA interrupt, therefore disable the interrupt for the transmit chan.
         */
        if (spiState->receiveBuffer)
        {
            /* Now, disable the TX chan interrupt since it is enable by the receive DMA */
            DMA_HAL_SetIntCmd(dmaBaseAddr, txChannel, false);
        }
        /* Else, keep the transmit channel interrupt enabled and install callback */
        else
        {
            DMA_DRV_RegisterCallback(&spiState->dmaTransmit, SPI_DRV_DmaCallback, param);
        }

        /* Enable the DMA peripheral request */
        DMA_DRV_StartChannel(&spiState->dmaTransmit);

        /* Once the Tx DMA channel is set up, send the data and then enable the DMA request */
        SPI_HAL_WriteData(spiBaseAddr, s_byteToSend);

        /* Enable the SPI TX DMA Request */
        SPI_HAL_SetTxDmaCmd(spiBaseAddr, true);
    }

    return kStatus_SPI_Success;
}

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the SPI module
 * is disabled. This is not a public API as it is called from other driver functions.
 */
static void SPI_DRV_MasterCompleteTransferDma(uint32_t instance)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    uint32_t spiBaseAddr = g_spiBaseAddr[instance];

    /* The transfer is complete.*/
    spiState->isTransferInProgress = false;

    /* Disable DMA requests. */
    SPI_HAL_SetRxDmaCmd(spiBaseAddr, false);
    SPI_HAL_SetTxDmaCmd(spiBaseAddr, false);

    /* Signal the synchronous completion object even if the transfer wasn't async.*/
    OSA_SemaPost(&spiState->irqSync);

    /* Transfer is complete, so disable the module.*/
    SPI_HAL_Disable(spiBaseAddr);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

