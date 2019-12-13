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

#if !defined(__FSL_SPI_SLAVE_DRIVER_H__)
#define __FSL_SPI_SLAVE_DRIVER_H__


#include "fsl_spi_hal.h"

/*! @addtogroup spi_slave_driver*/
/*! @{*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The set of callbacks used for SPI slave mode.*/
typedef struct SPISlaveCallbacks {
    /*! Callback used to get byte to transmit.*/
    spi_status_t (*dataSource)(uint8_t * sourceByte, uint32_t instance);

    /*! Callback used to put received byte.*/
    spi_status_t (*dataSink)(uint8_t sinkByte, uint32_t instance);

    /*! Callback used to report an SPI error.*/
    void (*onError)(spi_status_t error, uint32_t instance);

    /*! Callback to report the slave SPI DMA is done transferring data. Used only for
     * DMA enabled slave SPI operation and not used for interrupt operation.
     */
    void (*spi_slave_done_t)(void);

} spi_slave_callbacks_t;

/*!
 * @brief Runtime state of the SPI slave driver.
 *
 * This structure holds data that is used by the SPI slave peripheral driver to
 * communicate between the transfer function and the interrupt handler. The user
 * needs to pass in the memory for this structure and the driver  fills out
 * the members.
 */
typedef struct SPISlaveState {
    uint32_t instance;                  /*!< DSPI module instance number */
    spi_slave_callbacks_t callbacks;   /*!< Application/user callbacks */

} spi_slave_state_t;


/*! @brief Definition of application implemented configuration and callback */
/*! functions used by the SPI slave driver.*/
typedef struct SPISlaveUserConfig {
    spi_slave_callbacks_t callbacks;    /*!< Application callbacks.*/
    spi_clock_phase_t phase;            /*!< Clock phase setting. */
    spi_clock_polarity_t polarity;      /*!< Clock polarity setting.*/
    spi_shift_direction_t direction;    /*!< Either LSB or MSB first.*/
} spi_slave_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 *@name SPI Slave
 *@{
 */

/*!
 * @brief Initializes the SPI module for slave mode.
 *
 * Saves the application callback info, turns on the clock to the module,
 * enables the device, and enables interrupts. Sets the SPI to a slave mode.
 *
 * @param instance Instance number of the SPI module.
 * @param spiState The pointer to the SPI slave driver state structure.
 * @param config Pointer to slave mode configuration.
 */
void SPI_DRV_SlaveInit(uint32_t instance, spi_slave_state_t * spiState,
                       const spi_slave_user_config_t * config);


/*!
 * @brief De-initializes the device.
 *
 * Clears the control register and turns off the clock to the module.
 *
 * @param instance Instance number of the SPI module.
 */
void SPI_DRV_SlaveDeinit(uint32_t instance);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_SPI_SLAVE_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

