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

#include <stdlib.h>
#include <string.h>
#include "fsl_spi_master_driver.h"
#include "fsl_spi_shared_function.h"
#include "fsl_spi_common.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static spi_status_t SPI_DRV_MasterStartTransfer(uint32_t instance,
                                                const spi_master_user_config_t * restrict device);
static void SPI_DRV_MasterCompleteTransfer(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterInit
 * Description   : Initialize a SPI instance for master mode operation.
 * This function uses a CPU interrupt driven method for transferring data.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, ungates the clock to the SPI module, resets and initializes the module
 * to default settings, configures the IRQ state structure, enables
 * the module-level interrupt to the core, and enables the SPI module.
 *
 *END**************************************************************************/
void SPI_DRV_MasterInit(uint32_t instance, spi_master_state_t * spiState)
{
    uint32_t spiSourceClock;
    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* Clear the state for this instance.*/
    memset(spiState, 0, sizeof(* spiState));

    /* Enable clock for SPI*/
    CLOCK_SYS_EnableSpiClock(instance);

    /* Get module clock freq*/
    /* spi instance 1 use bus clock, instance 0 use system clock*/
    if(instance == 0)
    {
        CLOCK_SYS_GetFreq(kBusClock, &spiSourceClock);
    }
    else
    {
        CLOCK_SYS_GetFreq(kSystemClock, &spiSourceClock);
    }

    /* configure the run-time state struct with the source clock value */
    spiState->spiSourceClock = spiSourceClock;

    /* Reset the SPI module to it's default state, which includes SPI disabled */
    SPI_HAL_Init(baseAddr);

    /* Set SPI to master mode */
    SPI_HAL_SetMasterSlave(baseAddr, kSpiMaster);

    /* Set slave select to automatic output mode */
    SPI_HAL_SetSlaveSelectOutputMode(baseAddr, kSpiSlaveSelect_AutomaticOutput);

    /* Set the SPI pin mode to normal mode */
    SPI_HAL_SetPinMode(baseAddr, kSpiPinMode_Normal);

    /* Save runtime structure pointers to irq handler can point to the correct state structure*/
    g_spiStatePtr[instance] = spiState;

    /* Enable SPI interrupt.*/
    INT_SYS_EnableIRQ(g_spiIrqId[instance]);

    /* SPI system enable*/
    SPI_HAL_Enable(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterDeinit
 * Description   : Shutdown a SPI instance.
 * This function resets the SPI peripheral, gates its clock, and disables the interrupt to
 * the core.
 *
 *END**************************************************************************/
void SPI_DRV_MasterDeinit(uint32_t instance)
{
    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* Restore the module to defaults then power it down.*/
    SPI_HAL_Init(baseAddr);

    /* Disable SPI interrupt.*/
    INT_SYS_DisableIRQ(g_spiIrqId[instance]);

    /* Gate the clock for SPI.*/
    CLOCK_SYS_DisableSpiClock(instance);


    /* Clear state pointer. */
    g_spiStatePtr[instance] = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterConfigureBus
 * Description   : Configures the SPI port to access a device on the bus.
 * The term "device" is used to indicate the SPI device for which the SPI master is communicating.
 * The user has two options to configure the device parameters: either pass in the
 * pointer to the device configuration structure to the desired transfer function (see
 * SPI_DRV_MasterTransferDataBlocking or SPI_DRV_MasterTransferData) or pass it in to the
 * SPI_DRV_MasterConfigureBus function.  The user can pass in a device structure to the transfer
 * function which contains the parameters for the bus (the transfer function will then call
 * this function). However, the user has the option to call this function directly especially
 * to get the calculated baud rate, at which point they may pass in NULL for the device
 * struct in the transfer function (assuming they have called this configure bus function
 * first).
 *
 *END**************************************************************************/
void SPI_DRV_MasterConfigureBus(uint32_t instance,
                                const spi_master_user_config_t * device,
                                uint32_t * calculatedBaudRate)
{
    assert(device);

    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* Configure the bus to access the provided device.*/
    *calculatedBaudRate = SPI_HAL_SetBaud(baseAddr, device->bitsPerSec, spiState->spiSourceClock);
    SPI_HAL_SetDataFormat(baseAddr, device->polarity, device->phase, device->direction);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterTransferBlocking
 * Description   : Performs a blocking SPI master mode transfer.
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function will not return until the transfer is complete.
 *
 *END**************************************************************************/
spi_status_t SPI_DRV_MasterTransferBlocking(uint32_t instance,
                                            const spi_master_user_config_t * restrict device,
                                            const uint8_t * restrict sendBuffer,
                                            uint8_t * restrict receiveBuffer,
                                            size_t transferByteCount,
                                            uint32_t timeout)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    /* fill in members of the run-time state struct*/
    spiState->isTransferAsync = false;
    spiState->sendBuffer = (const uint8_t *)sendBuffer;
    spiState->receiveBuffer = (uint8_t *)receiveBuffer;
    spiState->remainingSendByteCount = transferByteCount;
    spiState->remainingReceiveByteCount = transferByteCount;

    /* Init the interrupt sync object.*/
    OSA_SemaCreate(&spiState->irqSync, 0);

    /* start the transfer process*/
    {
        if (SPI_DRV_MasterStartTransfer(instance, device) == kStatus_SPI_Busy)
        {
            return kStatus_SPI_Busy;
        }
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    spi_status_t error = kStatus_SPI_Success;
    osa_status_t syncStatus;

    do
    {
        syncStatus = OSA_SemaWait(&spiState->irqSync, timeout);
    }while(syncStatus == kStatus_OSA_Idle);

    if (syncStatus != kStatus_OSA_Success)
    {
        /* Abort the transfer so it doesn't continue unexpectedly.*/
        SPI_DRV_MasterAbortTransfer(instance);

        error = kStatus_SPI_Timeout;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterTransfer
 * Description   : Perform a non-blocking SPI master mode transfer.
 * This function will return immediately. It is the user's responsiblity to check back to
 * ascertain if the transfer is complete (using the SPI_DRV_MasterGetTransferStatus function).
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function will not return until the transfer is complete.
 *
 *END**************************************************************************/
spi_status_t SPI_DRV_MasterTransfer(uint32_t instance,
                                    const spi_master_user_config_t * restrict device,
                                    const uint8_t * restrict sendBuffer,
                                    uint8_t * restrict receiveBuffer,
                                    size_t transferByteCount)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    /* fill in members of the run-time state struct*/
    spiState->isTransferAsync = true;
    spiState->sendBuffer = sendBuffer;
    spiState->receiveBuffer = (uint8_t *)receiveBuffer;
    spiState->remainingSendByteCount = transferByteCount;
    spiState->remainingReceiveByteCount = transferByteCount;

    /* start the transfer process*/
    {
        if (SPI_DRV_MasterStartTransfer(instance, device) == kStatus_SPI_Busy)
        {
            return kStatus_SPI_Busy;
        }
    }

    /* Else, return immediately as this is an async transfer */
    return kStatus_SPI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterGetTransferStatus
 * Description   : Returns whether the previous transfer finished.
 * When performing an a-sync transfer, the user can call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success). In addition, if the transfer
 * is still in progress, the user can get the number of words that have been
 * transferred up to now.
 *
 *END**************************************************************************/
spi_status_t SPI_DRV_MasterGetTransferStatus(uint32_t instance,
                                             uint32_t * bytesTransferred)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    /* Fill in the bytes transferred.*/
    if (bytesTransferred)
    {
        {
            *bytesTransferred = spiState->transferredByteCount;
        }
    }

    return (spiState->isTransferInProgress ? kStatus_SPI_Busy : kStatus_SPI_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_DRV_MasterAbortTransfer
 * Description   : Terminates an asynchronous transfer early.
 * During an async transfer, the user has the option to terminate the transfer early if the transfer
 * is still in progress.
 *
 *END**************************************************************************/
spi_status_t SPI_DRV_MasterAbortTransfer(uint32_t instance)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    /* Check if a transfer is running.*/
    if (!spiState->isTransferInProgress)
    {
        return kStatus_SPI_NoTransferInProgress;
    }

    /* Stop the running transfer.*/
    SPI_DRV_MasterCompleteTransfer(instance);

    return kStatus_SPI_Success;
}

/*!
 * @brief Initiate (start) a transfer. This is not a public API as it is called from other
 *  driver functions
 */
static spi_status_t SPI_DRV_MasterStartTransfer(uint32_t instance,
                                                const spi_master_user_config_t * restrict device)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    uint32_t calculatedBaudRate;
    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* If the byte count is zero, then return immediately.*/
    if (spiState->remainingSendByteCount == 0)
    {
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
    spiState->transferredByteCount = 0;

    /* Enable the SPI module for the duration of this transfer.*/
    SPI_HAL_Enable(baseAddr);

    /* Clear the read buffer if there's anything in it. This also causes a read of the status
     * register which is required before writing to the data register below.
     */
    if (SPI_HAL_IsReadBuffFullPending(baseAddr))
    {
        (void)SPI_HAL_ReadData(baseAddr);
    }

    /* Start the transfer by writing the first byte. If a send buffer was provided, the byte
     * comes from there. Otherwise we just send a zero byte. Note that before writing to the
     * data register, the status register must first be read, which was already performed above.
     */
    uint8_t byteToSend = 0;
    if (spiState->sendBuffer)
    {
        byteToSend = *(spiState->sendBuffer);
        ++spiState->sendBuffer;
    }
    SPI_HAL_WriteData(baseAddr, byteToSend);
    --spiState->remainingSendByteCount;
    ++spiState->transferredByteCount;

    /* Only enable the receive interrupt.  This should ok ok since SPI is a synchronous
     * protocol, so every RX interrupt we get, we should be ready to send. However, since
     * the SPI has a shift register and data buffer (for transmit and receive), things may
     * be cycle-to-cycle synchronous. To compensate for this, enabling the RX interrupt only
     * ensures that we do indeed receive data before sending out the next data word. In the ISR
     * we make sure to first check for the RX data buffer full before checking the TX data register
     * empty flag.
     */
    SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, true);

    return kStatus_SPI_Success;
}

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the SPI module
 * is disabled. This is not a public API as it is called from other driver functions.
 */
static void SPI_DRV_MasterCompleteTransfer(uint32_t instance)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* The transfer is complete.*/
    spiState->isTransferInProgress = false;

    /* Disable interrupts. Though the TX interrupt was not enabled, it is a good idea to ensure
     * it is disabled.
     */
    SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, false);
    SPI_HAL_SetTransmitIntCmd(baseAddr, false);

    /* Signal the synchronous completion object even if the transfer wasn't async.*/
    OSA_SemaPost(&spiState->irqSync);

    /* Transfer is complete, so disable the module.*/
    SPI_HAL_Disable(baseAddr);
}

/*!
 * @brief Interrupt handler for SPI master mode.
 * This handler uses the buffers stored in the spi_master_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void SPI_DRV_MasterIRQHandler(uint32_t instance)
{
    /* instantiate local variable of type spi_master_state_t and point to global state */
    spi_master_state_t * spiState = (spi_master_state_t *)g_spiStatePtr[instance];

    uint32_t baseAddr = g_spiBaseAddr[instance];

    /* Exit the ISR if no transfer is happening for this instance.*/
    if (!spiState->isTransferInProgress)
    {
        return;
    }


    /* Check read buffer.*/
    uint8_t byteReceived;
    if (spiState->remainingReceiveByteCount)
    {
        if (SPI_HAL_IsReadBuffFullPending(baseAddr))
        {
            byteReceived = SPI_HAL_ReadData(baseAddr);

            if (spiState->receiveBuffer)
            {
                *spiState->receiveBuffer = byteReceived;
                ++spiState->receiveBuffer;
            }
            --spiState->remainingReceiveByteCount;
        }
    }

    /* Check write buffer. We always have to send a byte in order to keep the transfer*/
    /* moving. So if the caller didn't provide a send buffer, we just send a zero byte.*/
    uint8_t byteToSend = 0;
    if (spiState->remainingSendByteCount)
    {
        if (SPI_HAL_IsTxBuffEmptyPending(baseAddr))
        {
            if (spiState->sendBuffer)
            {
                byteToSend = *(spiState->sendBuffer);
                ++spiState->sendBuffer;
            }
            --spiState->remainingSendByteCount;
            ++spiState->transferredByteCount;

            SPI_HAL_WriteData(baseAddr, byteToSend);
        }
    }

    /* Check if we're done with this transfer.*/
    if ((spiState->remainingSendByteCount == 0) && (spiState->remainingReceiveByteCount == 0))
    {
        /* Complete the transfer. This disables the interrupts, so we don't wind up in*/
        /* the ISR again.*/
        SPI_DRV_MasterCompleteTransfer(instance);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

