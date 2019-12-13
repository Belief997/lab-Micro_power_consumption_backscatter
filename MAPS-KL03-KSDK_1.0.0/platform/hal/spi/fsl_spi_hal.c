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
#include "fsl_spi_hal.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Bit offsets for bits encoded in enum values.*/
enum _spi_pin_bit_encodings
{
    kSpiSsoeBit = 0U,    /*!< SSOE is bit 0 of #spi_ss_output_mode_t.*/
    kSpiModfenBit = 1U,  /*!< MODFEN is bit 1 of #spi_ss_output_mode_t.*/
    kSpiSpc0Bit = 0U,    /*!< SPC0 is bit 0 of #spi_pin_mode_t.*/
    kSpiBidiroeBit = 1U  /*!< BIDIROE is bit 1 of #spi_pin_mode_t.*/
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_Init
 * Description   : RestoreS SPI to reset configuration.
 * This function basically resets all of the SPI registers to their default setting including
 * disabling the module.
 *
 *END**************************************************************************/
void SPI_HAL_Init(uint32_t baseAddr)
{
    HW_SPI_C1_WR(baseAddr, BM_SPI_C1_CPHA);
    HW_SPI_C2_WR(baseAddr, 0);
    HW_SPI_BR_WR(baseAddr, 0);
    HW_SPI_M_WR(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_SetBaud
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate without exceeding the desired baud rate, and  returns the calculated
 * baud rate in bits-per-second. It requires that the caller also provide the frequency of the
 * module source clock (in Hertz).
 *
 *END**************************************************************************/
uint32_t SPI_HAL_SetBaud(uint32_t baseAddr, uint32_t bitsPerSec, uint32_t sourceClockInHz)
{
    uint32_t prescaler, bestPrescaler;
    uint32_t rateDivisor, bestDivisor;
    uint32_t rateDivisorValue;
    uint32_t realBaudrate, bestBaudrate;
    uint32_t diff, min_diff;
    uint32_t baudrate = bitsPerSec;

    /* find combination of prescaler and scaler resulting in baudrate closest to the
     * requested value
     */
    min_diff = 0xFFFFFFFFU;
    bestPrescaler = 0;
    bestDivisor = 0;
    bestBaudrate = 0; /* required to avoid compilation warning */

    /* In all for loops, if min_diff = 0, the exit for loop*/
    for (prescaler = 0; (prescaler <= 7) && min_diff; prescaler++)
    {
        rateDivisorValue = 2U;  /* Initialize to div-by-2 */

        for (rateDivisor = 0; (rateDivisor <= 8U) && min_diff; rateDivisor++)
        {
            /* calculate actual baud rate, note need to add 1 to prescaler */
            realBaudrate = ((sourceClockInHz) /
                            ((prescaler + 1) * rateDivisorValue));

            /* calculate the baud rate difference based on the conditional statement*/
            /* that states that the calculated baud rate must not exceed the desired baud rate*/
            if (baudrate >= realBaudrate)
            {
                diff = baudrate-realBaudrate;
                if (min_diff > diff)
                {
                    /* a better match found */
                    min_diff = diff;
                    bestPrescaler = prescaler; /* Prescale divisor SPIx_BR register bit setting */
                    bestDivisor = rateDivisor; /* baud rate divisor SPIx_BR register bit setting */
                    bestBaudrate = realBaudrate;
                }
            }
            /* Multiply by 2 for each iteration, possible divisor values: 2, 4, 8, 16, ... 512 */
            rateDivisorValue *= 2U;
        }
    }

    /* write the best prescalar and baud rate scalar */
    SPI_HAL_SetBaudDivisors(baseAddr, bestPrescaler, bestDivisor);

    /* return the actual calculated baud rate*/
    return bestBaudrate;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_SetSlaveSelectOutputMode
 * This function allows the user to configure the slave select in one of the three operational
 * modes: as GPIO, as a fault input, or as an automatic output for standard SPI modes.
 *
 *END**************************************************************************/
void SPI_HAL_SetSlaveSelectOutputMode(uint32_t baseAddr, spi_ss_output_mode_t mode)
{
    /* The mode enum values encode the SSOE and MODFEN bit values.*/
    /* Bit 0: SSOE*/
    /* Bit 1: MODFEN*/
    BW_SPI_C1_SSOE(baseAddr, ((uint32_t)mode & (1U << kSpiSsoeBit)) >> kSpiSsoeBit);
    BW_SPI_C2_MODFEN(baseAddr, ((uint32_t)mode &
                     (1U << kSpiModfenBit)) >> kSpiModfenBit);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_SetDataFormat
 * This function configures the clock polarity, clock phase, and data shift direction.
 *
 *END**************************************************************************/
void SPI_HAL_SetDataFormat(uint32_t baseAddr,
    spi_clock_polarity_t polarity,
    spi_clock_phase_t phase,
    spi_shift_direction_t direction)
{
    BW_SPI_C1_CPOL(baseAddr, (uint32_t)polarity);
    BW_SPI_C1_CPHA(baseAddr, (uint32_t)phase);
    BW_SPI_C1_LSBFE(baseAddr, (uint32_t)direction);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_SetPinMode
 * This function configures the SPI data pins to one of three modes (of type spi_pin_mode_t):
 * Single direction mdoe: MOSI and MISO pins operate in normal, single direction mode.
 * Birectional mode: Master: MOSI configured as input, Slave: MISO configured as input.
 * Birectional mode: Master: MOSI configured as output, Slave: MISO configured as output.
 *END**************************************************************************/
void SPI_HAL_SetPinMode(uint32_t baseAddr, spi_pin_mode_t mode)
{
    /* The mode enum values encode the SPC0 and BIDIROE bit values.*/
    /* Bit 0: SPC0*/
    /* Bit 1: BIDIROE*/
    BW_SPI_C2_SPC0(baseAddr, ((uint32_t)mode & (1U << kSpiSpc0Bit)) >> kSpiSpc0Bit);
    BW_SPI_C2_BIDIROE(baseAddr, ((uint32_t)mode & (1U << kSpiBidiroeBit)) >> kSpiBidiroeBit);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_ClearModeFaultFlag
 * This function clears the mode fault flag.
 *END**************************************************************************/
void SPI_HAL_ClearModeFaultFlag(uint32_t baseAddr)
{
    /* Must make sure we read from the status register first. Then, if set,
     * we must write to SPI_C1 (per the reference manual).
     */
    if (SPI_HAL_IsModeFaultPending(baseAddr))
    {
        /* Then we have to write to C1.*/
        HW_SPI_C1_WR(baseAddr, HW_SPI_C1_RD(baseAddr));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_ClearMatchFlag
 * This function clears the match flag.
 *END**************************************************************************/
void SPI_HAL_ClearMatchFlag(uint32_t baseAddr)
{
    /* Check that the match flag is set before writing 1 to clear it. This read*/
    /* is required in order to clear the flag.*/
    if (SPI_HAL_IsMatchPending(baseAddr))
    {
        /* We have to hack this to write to the register because it is incorrectly
         * defined as a read-only register, even though the SPI_S.SPMF bitfield documentation
         * states you must write a 1 to the bitfield to clear it.
         */
        *(volatile uint8_t *)HW_SPI_S_ADDR(baseAddr) = BM_SPI_S_SPMF;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI_HAL_WriteDataBlocking
 * This function will write data to the data register and will wait until
 * TX empty to return.
 *END**************************************************************************/
void SPI_HAL_WriteDataBlocking(uint32_t baseAddr, uint8_t data)
{
    /* Since this is a blocking write, it is assume the user will call this function
     * directly. Per the ref manual, the status register must first be read with the
     * SPTEF bit set.  So wait until SPTEF gets set to make sure the buffer is empty.
     */
    while(SPI_HAL_IsTxBuffEmptyPending(baseAddr) == 0) { }

    HW_SPI_D_WR(baseAddr, data);

    /* Wait until TX empty is set before return */
    while(SPI_HAL_IsTxBuffEmptyPending(baseAddr) == 0) { }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

