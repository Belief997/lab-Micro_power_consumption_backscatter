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
#if !defined(__FSL_SPI_HAL_H__)
#define __FSL_SPI_HAL_H__



#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

/*! @addtogroup spi_hal*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Error codes for the SPI driver.*/
typedef enum _spi_errors
{
    kStatus_SPI_Success = 0,
    kStatus_SPI_SlaveTxUnderrun,      /*!< SPI Slave TX Underrun error.*/
    kStatus_SPI_SlaveRxOverrun,       /*!< SPI Slave RX Overrun error.*/
    kStatus_SPI_Timeout,              /*!< SPI transfer timed out.*/
    kStatus_SPI_Busy,                 /*!< SPI instance is already busy performing a transfer.*/
    kStatus_SPI_NoTransferInProgress, /*!< Attempt to abort a transfer when no transfer
                                           was in progress.*/
    kStatus_SPI_OutOfRange            /*< SPI out-of-range error used in slave callback */
} spi_status_t;

/*! @brief SPI master or slave configuration.*/
typedef enum _spi_master_slave_mode {
    kSpiMaster = 1,     /*!< SPI peripheral operates in master mode.*/
    kSpiSlave = 0       /*!< SPI peripheral operates in slave mode.*/
} spi_master_slave_mode_t;

/*! @brief SPI clock polarity configuration.*/
typedef enum _spi_clock_polarity {
    kSpiClockPolarity_ActiveHigh = 0,   /*!< Active-high SPI clock (idles low).*/
    kSpiClockPolarity_ActiveLow = 1     /*!< Active-low SPI clock (idles high).*/
} spi_clock_polarity_t;

/*! @brief SPI clock phase configuration.*/
typedef enum _spi_clock_phase {
    kSpiClockPhase_FirstEdge = 0,       /*!< First edge on SPSCK occurs at the middle of the first
                                         *   cycle of a data transfer.*/
    kSpiClockPhase_SecondEdge = 1       /*!< First edge on SPSCK occurs at the start of the
                                         *   first cycle of a data transfer.*/
} spi_clock_phase_t;

/*! @brief SPI data shifter direction options.*/
typedef enum _spi_shift_direction {
    kSpiMsbFirst = 0,    /*!< Data transfers start with most significant bit.*/
    kSpiLsbFirst = 1    /*!< Data transfers start with least significant bit.*/
} spi_shift_direction_t;

/*! @brief SPI slave select output mode options.*/
typedef enum _spi_ss_output_mode {
    kSpiSlaveSelect_AsGpio = 0,         /*!< Slave select pin configured as GPIO.*/
    kSpiSlaveSelect_FaultInput = 2,     /*!< Slave select pin configured for fault detection.*/
    kSpiSlaveSelect_AutomaticOutput = 3 /*!< Slave select pin configured for automatic SPI output.*/
} spi_ss_output_mode_t;

/*! @brief SPI pin mode options.*/
typedef enum _spi_pin_mode {
    kSpiPinMode_Normal = 0,     /*!< Pins operate in normal, single-direction mode.*/
    kSpiPinMode_Input = 1,      /*!< Bidirectional mode. Master: MOSI pin is input;
                                 *   Slave: MISO pin is input*/
    kSpiPinMode_Output = 3      /*!< Bidirectional mode. Master: MOSI pin is output;
                                 *   Slave: MISO pin is output*/
} spi_pin_mode_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern const uint32_t spi_base_addr[];

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Restores the SPI to reset configuration.
 *
 * This function basically resets all of the SPI registers to their default setting including
 * disabling the module.
 *
 * @param baseAddr Module base address
 */
void SPI_HAL_Init(uint32_t baseAddr);

/*!
 * @brief Enables the SPI peripheral.
 *
 * @param baseAddr Module base address
 */
static inline void SPI_HAL_Enable(uint32_t baseAddr)
{
    BW_SPI_C1_SPE(baseAddr, 1);
}

/*!
 * @brief Disables the SPI peripheral.
 *
 * @param baseAddr Module base address
 */
static inline void SPI_HAL_Disable(uint32_t baseAddr)
{
    BW_SPI_C1_SPE(baseAddr, 0);
}

/*!
 * @brief Sets the SPI baud rate in bits per second.
 *
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate without exceeding the desired baud rate, and returns the calculated
 * baud rate in bits-per-second. It requires that the caller also provide the frequency of the
 * module source clock (in Hertz).
 *
 * @param baseAddr Module base address
 * @param bitsPerSec The desired baud rate in bits per second
 * @param sourceClockInHz Module source input clock in Hertz
 * @return  The actual calculated baud rate
 */
uint32_t SPI_HAL_SetBaud(uint32_t baseAddr, uint32_t bitsPerSec, uint32_t sourceClockInHz);

/*!
 * @brief Configures the baud rate divisors manually.
 *
 * This function allows the caller to manually set the baud rate divisors in the event that
 * these dividers are known and the caller does not wish to call the SPI_HAL_SetBaudRate function.
 *
 * @param baseAddr Module base address
 * @param prescaleDivisor baud rate prescale divisor setting
 * @param rateDivisor baud rate divisor setting
 */
static inline void SPI_HAL_SetBaudDivisors(uint32_t baseAddr, uint32_t prescaleDivisor,
                                            uint32_t rateDivisor)
{
    /* Use the "HW_SPI_BR_WR" function to perform this write in one line of code for inline */
    HW_SPI_BR_WR(baseAddr, BF_SPI_BR_SPR(rateDivisor) |
                 BF_SPI_BR_SPPR(prescaleDivisor));
}

/*!
 * @brief Configures the SPI for master or slave.
 *
 * @param baseAddr Module base address
 * @param mode Mode setting (master or slave) of type dspi_master_slave_mode_t
 */
static inline void SPI_HAL_SetMasterSlave(uint32_t baseAddr, spi_master_slave_mode_t mode)
{
    BW_SPI_C1_MSTR(baseAddr, (uint32_t)mode);
}

/*!
 * @brief Returns whether the SPI module is in master mode.
 *
 * @param baseAddr Module base address
 * @retval true The module is in master mode.
 * @retval false The module is in slave mode.
 */
static inline bool SPI_HAL_IsMaster(uint32_t baseAddr)
{
    return (bool)BR_SPI_C1_MSTR(baseAddr);
}

/*!
 * @brief Sets how the slave select output operates.
 *
 * This function allows the user to configure the slave select in one of the three operational
 * modes: as GPIO, as a fault input, or as an automatic output for standard SPI modes.
 *
 * @param baseAddr Module base address
 * @param mode Selection input of one of three mdoes of type spi_ss_output_mode_t
 */
void SPI_HAL_SetSlaveSelectOutputMode(uint32_t baseAddr, spi_ss_output_mode_t mode);

/*!
 * @brief Sets the polarity, phase, and shift direction.
 *
 * This function configures the clock polarity, clock phase, and data shift direction.
 *
 * @param baseAddr Module base address
 * @param polarity Clock polarity setting of type spi_clock_polarity_t.
 * @param polarity Clock phase setting of type spi_clock_phase_t.
 * @param direction Data shift direction (MSB or LSB) of type spi_shift_direction_t.
 */
void SPI_HAL_SetDataFormat(uint32_t baseAddr,
    spi_clock_polarity_t polarity,
    spi_clock_phase_t phase,
    spi_shift_direction_t direction);

/*!
 * @brief Sets the SPI pin mode.
 *
 * This function configures the SPI data pins to one of three modes (of type spi_pin_mode_t):
 * Single direction mode: MOSI and MISO pins operate in normal, single direction mode.
 * Bidirectional mode: Master: MOSI configured as input, Slave: MISO configured as input.
 * Bidirectional mode: Master: MOSI configured as output, Slave: MISO configured as output.
 *
 * @param baseAddr Module base address
 * @param mode Operational of SPI pins of type spi_pin_mode_t.
 */
void SPI_HAL_SetPinMode(uint32_t baseAddr, spi_pin_mode_t mode);

/*@}*/


/*!
 * @name Low power
 * @{
 */

/*!
 * @brief Enables or disables the SPI clock to stop when the CPU enters wait mode.
 *
 * This function enables or disables the SPI clock operation in wait mode.
 *
 * @param baseAddr Module base address
 * @param enable Enable (true) or disable (false) the SPI clock in wait mode.
 */
static inline void SPI_HAL_ConfigureStopInWaitMode(uint32_t baseAddr, bool enable)
{
    BW_SPI_C2_SPISWAI(baseAddr, (enable == true));
}

/*@}*/

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables or disables the SPI receive buffer full and mode fault interrupt.
 *
 * This function enables or disables the SPI receive buffer full and mode fault interrupt.
 *
 * @param baseAddr Module base address
 * @param enable Enable (true) or disable (false) the receive buffer full and mode fault interrupt.
 */
static inline void SPI_HAL_SetReceiveAndFaultIntCmd(uint32_t baseAddr, bool enable)
{
    BW_SPI_C1_SPIE(baseAddr, (enable == true));
}

/*!
 * @brief Enables or disables the SPI transmit buffer empty interrupt.
 *
 * This function enables or disables the SPI transmit buffer empty interrupt.
 *
 * @param baseAddr Module base address
 * @param enable Enable (true) or disable (false) the transmit buffer empty interrupt.
 */
static inline void SPI_HAL_SetTransmitIntCmd(uint32_t baseAddr, bool enable)
{
    BW_SPI_C1_SPTIE(baseAddr, (enable == true));
}

/*!
 * @brief Enables or disables the SPI match interrupt.
 *
 * This function enables or disables the SPI match interrupt.
 *
 * @param baseAddr Module base address
 * @param enable Enable (true) or disable (false) the match interrupt.
 */
static inline void SPI_HAL_SetMatchIntCmd(uint32_t baseAddr, bool enable)
{
    BW_SPI_C2_SPMIE(baseAddr, (enable == true));
}

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Checks whether the read buffer is full.
 *
 * The read buffer full flag is only cleared by reading it when it is set, then reading the
 * data register by calling the SPI_HAL_ReadData(). This example code demonstrates how to check
 * the flag, read data, and clear the flag.
   @code
        // Check read buffer flag.
        if (SPI_HAL_IsReadBuffFullPending(0))
        {
            // Read the data in the buffer, which also clears the flag.
            byte = SPI_HAL_ReadData(0);
        }
   @endcode
 *
 * @param baseAddr Module base address
 * @retval Current setting of the read buffer full flag
 */
static inline bool SPI_HAL_IsReadBuffFullPending(uint32_t baseAddr)
{
    return BR_SPI_S_SPRF(baseAddr);
}

/*!
 * @brief Checks whether the transmit buffer is empty.
 *
 * To clear the transmit buffer empty flag, you must first read the flag when it is set. Then
 * write a new data value into the transmit buffer with a call to the SPI_HAL_WriteData(). The
 * example code shows how to do this.
   @code
        // Check if transmit buffer is empty.
        if (SPI_HAL_IsTxBuffEmptyPending(0))
        {
            // Buffer has room, so write the next data value.
            SPI_HAL_WriteData(0, byte);
        }
   @endcode
 *
 * @param baseAddr Module base address
 * @retval Current setting of the transmit buffer empty flag
 */
static inline bool SPI_HAL_IsTxBuffEmptyPending(uint32_t baseAddr)
{
    return BR_SPI_S_SPTEF(baseAddr);
}

/*!
 * @brief Checks whether a mode fault occurred.
 *
 * @param baseAddr Module base address
 * @retval Current setting of the mode fault flag
 */
static inline bool SPI_HAL_IsModeFaultPending(uint32_t baseAddr)
{
    return BR_SPI_S_MODF(baseAddr);
}

/*!
 * @brief Clears the mode fault flag.
 *
 * @param baseAddr Module base address
 */
void SPI_HAL_ClearModeFaultFlag(uint32_t baseAddr);

/*!
 * @brief Checks whether the data received matches the previously-set match value.
 *
 * @param baseAddr Module base address
 * @retval Current setting of the match flag
 */
static inline bool SPI_HAL_IsMatchPending(uint32_t baseAddr)
{
    return BR_SPI_S_SPMF(baseAddr);
}

/*!
 * @brief Clears the match flag.
 *
 * @param baseAddr Module base address
 */
void SPI_HAL_ClearMatchFlag(uint32_t baseAddr);

/*@}*/

/*!
 * @name Data transfer
 *@{
 */

/*!
 * @brief Reads a byte from the data buffer.
 *
 * @param baseAddr Module base address
 */
static inline uint8_t SPI_HAL_ReadData(uint32_t baseAddr)
{
    return HW_SPI_D_RD(baseAddr);
}

/*!
 * @brief Writes a byte into the data buffer.
 *
 * @param baseAddr Module base address
 * @param data The data to send
 */
static inline void SPI_HAL_WriteData(uint32_t baseAddr, uint8_t data)
{
    HW_SPI_D_WR(baseAddr, data);
}

/*!
 * @brief Writes a byte into the data buffer and waits till complete to return.
 *
 * This function writes data to the data register and waits until the
 * TX is empty to return.
 *
 * @param baseAddr Module base address
 * @param data The data to send
 */
void SPI_HAL_WriteDataBlocking(uint32_t baseAddr, uint8_t data);

/*@}*/

/*! @name Match byte*/
/*@{*/

/*!
 * @brief Sets the value which triggers the match interrupt.
 *
 * @param baseAddr Module base address
 * @param matchBtye The value which triggers the match interrupt.
 */
static inline void SPI_HAL_SetMatchValue(uint32_t baseAddr, uint8_t matchByte)
{
    HW_SPI_M_WR(baseAddr, matchByte);
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_SPI_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

