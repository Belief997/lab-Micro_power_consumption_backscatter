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
#ifndef _FSL_SPI_ABSTRACTION_H_
#define _FSL_SPI_ABSTRACTION_H_


#include "fsl_os_abstraction.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* SPI mode flags */
#define SPI_CPHA        0x01                    /* clock phase */
#define SPI_CPOL        0x02                    /* clock polarity */
#define SPI_MODE_0      (0 | 0)                 /* (original MicroWire) */
#define SPI_MODE_1      (0 | SPI_CPHA)
#define SPI_MODE_2      (SPI_CPOL | 0)
#define SPI_MODE_3      (SPI_CPOL | SPI_CPHA)
#define SPI_CS_HIGH     0x04                    /* CS active high */
#define SPI_LSB_FIRST   0x08                    /* per-word bits-on-wire */
#define SPI_3WIRE       0x10                    /* SI/SO signals shared */
#define SPI_LOOP        0x20                    /* loopback mode */

/* FIXME: wish someday, I can use DSPI macro here. */
/* #ifdef FSL_FEATURE_SPI_IS_DSPI */
#if defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK22FN512VDC12) || \
    defined(CPU_MKV31F512VLL12) || defined(CPU_MK22FN128VDC10) || \
    defined(CPU_MKV31F128VLL10) || defined(CPU_MK22FN256VDC12) || \
    defined(CPU_MKV31F256VLL12)

#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_hal.h"

typedef struct SpiSlaveDev
{
    uint32_t instance;
    dspi_device_t spiDevice;
    dspi_master_state_t dspiMasterState;
    int32_t ss_pol;
    int32_t cs;
} spi_slave_dev;

static inline int32_t spi_init(spi_slave_dev *spi_slave, uint32_t bus, uint32_t cs, uint32_t max_hz, uint32_t mode)
{
    dspi_master_user_config_t userConfig;
    dspi_device_t *spiDevice = &spi_slave->spiDevice;
    dspi_master_state_t *dspiMasterState = &spi_slave->dspiMasterState;
    dspi_clock_phase_t cpha;
    dspi_clock_polarity_t cpol;
    uint32_t calculatedBaudRate;

    switch (mode)
    {
    case 0:
        cpol = kDspiClockPolarity_ActiveHigh;
        cpha = kDspiClockPhase_FirstEdge;
        break;
    case 1:
        cpol = kDspiClockPolarity_ActiveHigh;
        cpha = kDspiClockPhase_SecondEdge;
        break;
    case 2:
        cpol = kDspiClockPolarity_ActiveLow;
        cpha = kDspiClockPhase_FirstEdge;
        break;
    case 3:
        cpol = kDspiClockPolarity_ActiveLow;
        cpha = kDspiClockPhase_SecondEdge;
        break;
    default:
        cpol = kDspiClockPolarity_ActiveHigh;
        cpha = kDspiClockPhase_FirstEdge;
        break;
    }

    if (mode & SPI_CS_HIGH)
    {
        userConfig.pcsPolarity = kDspiPcs_ActiveHigh;
    }
    else
    {
        userConfig.pcsPolarity = kDspiPcs_ActiveLow;
    }

    userConfig.isChipSelectContinuous = true;
    userConfig.isSckContinuous = false;
    userConfig.whichCtar = kDspiCtar0;
    userConfig.whichPcs = (dspi_which_pcs_config_t)(1 << cs);

    spiDevice->bitsPerSec = max_hz;
    spiDevice->dataBusConfig.bitsPerFrame = 8;
    spiDevice->dataBusConfig.clkPolarity = cpol;
    spiDevice->dataBusConfig.clkPhase = cpha;
    spiDevice->dataBusConfig.direction = kDspiMsbFirst;

    spi_slave->instance = bus;
    spi_slave->ss_pol = 0;
    spi_slave->cs = cs;

    DSPI_DRV_MasterInit(bus, dspiMasterState, &userConfig);

    DSPI_DRV_MasterConfigureBus(bus, spiDevice, &calculatedBaudRate);

    return 0;
}


static inline int32_t spi_xfer(spi_slave_dev *spi_slave, const uint8_t *data_out,
				uint8_t *data_in, size_t data_len)
{
    uint32_t instance = spi_slave->instance;
    int32_t timeout = 1000;
 
    if (DSPI_DRV_MasterTransferData(instance, NULL, data_out,
		    			data_in, data_len) != kStatus_DSPI_Success)
    {
        return 1;
    }

    while (kStatus_DSPI_Busy == DSPI_DRV_MasterGetTransferStatus(instance, NULL) &&
            timeout--)
    {
        OSA_TimeDelay(1);
    }

    return (timeout <= 0) ? 1 : 0;
}

static inline int32_t spi_cs_gpio_init(uint32_t bus)
{
    return 0;
}

#else

#include "fsl_spi_master_driver.h"
#include "fsl_spi_hal.h"

typedef struct SpiSlaveDev
{
    uint32_t instance;
    spi_master_state_t spiMasterState;
    int32_t ss_pol;
    int32_t cs;
} spi_slave_dev;

static inline int32_t spi_init(spi_slave_dev *spi_slave, uint32_t bus, uint32_t cs, uint32_t max_hz, uint32_t mode)
{
    spi_master_user_config_t userConfig;
    spi_master_state_t *spiMasterState = &spi_slave->spiMasterState;
    spi_clock_phase_t cpha;
    spi_clock_polarity_t cpol;
    uint32_t calculatedBaudRate;

    switch (mode)
    {
    case 0:
        cpol = kSpiClockPolarity_ActiveHigh;
        cpha = kSpiClockPhase_FirstEdge;
        break;
    case 1:
        cpol = kSpiClockPolarity_ActiveHigh;
        cpha = kSpiClockPhase_SecondEdge;
        break;
    case 2:
        cpol = kSpiClockPolarity_ActiveLow;
        cpha = kSpiClockPhase_FirstEdge;
        break;
    case 3:
        cpol = kSpiClockPolarity_ActiveLow;
        cpha = kSpiClockPhase_SecondEdge;
        break;
    default:
        cpol = kSpiClockPolarity_ActiveHigh;
        cpha = kSpiClockPhase_FirstEdge;
        break;
    }

    userConfig.bitsPerSec = max_hz;
    userConfig.polarity = cpol;
    userConfig.phase = cpha;
    userConfig.direction = kSpiMsbFirst;

    spi_slave->instance = bus;
    spi_slave->ss_pol = 0;
    spi_slave->cs = cs;

    SPI_DRV_MasterInit(bus, spiMasterState);

    SPI_DRV_MasterConfigureBus(bus, &userConfig, &calculatedBaudRate);

    return 0;
}

static inline int32_t spi_xfer(const spi_slave_dev *spi_slave, const uint8_t *data_out, uint8_t *data_in, size_t data_len)
{
    uint32_t instance = spi_slave->instance;
    //int32_t timeout = 10000;
    uint32_t timeout = OSA_WAIT_FOREVER;

    if (SPI_DRV_MasterTransferBlocking(instance, NULL, data_out,
                               data_in, data_len, timeout) != kStatus_SPI_Success)
    {
        return 1;
    }

//    while (kStatus_SPI_Busy == SPI_DRV_MasterGetTransferStatus(instance, NULL) &&
//            timeout--)
//    {
//       OSA_TimeDelay(1);
//    }

//    return (timeout <= 0) ? 1 : 0;
    return 0;
}

static inline int32_t spi_cs_gpio_init(uint32_t bus)
{
    return 0;
}

#endif


#if defined(__cplusplus)
}
#endif

#endif

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
