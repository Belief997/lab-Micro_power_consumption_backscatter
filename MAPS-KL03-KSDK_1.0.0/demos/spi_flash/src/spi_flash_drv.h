/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include "device/fsl_device_registers.h"
#include "fsl_misc_utilities.h"

#define SFDEBUG

#define SPI_FLASH_WINBOND_VENDER_ID    (0xEF) /* WINBOND vendor id: 0xEF */
#define IDCODE_LEN                     (0x3)

/* flash capacity configuration */
#define FLASH_PAGE_SIZE 256 /* 256B page size */
#define FLASH_SECTOR_SIZE (256 * 16) /* 4K sector size */
#define FLASH_TOTAL_SIZE  (1024 * 1024) /* 4K sector size */

#define ROUND(a,b)  (((a) + (b)) & ~((b) - 1))

#ifndef min
#define min(a, b)   ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b)   ((a) > (b) ? (a) : (b))
#endif

#ifdef SFDEBUG
#define SF_DEBUG(str) printf(str)
#else
#define SF_DEBUG(str)
#endif

#define SPI_FLASH_TIMEOUT		(20000000)

/* flash commands */
#define CMD_READ_ID			0x9f

#define CMD_READ_ARRAY_SLOW		0x03
#define CMD_READ_ARRAY_FAST		0x0b

#define CMD_WRITE_STATUS		0x01
#define CMD_PAGE_PROGRAM		0x02
#define CMD_WRITE_DISABLE		0x04
#define CMD_READ_STATUS			0x05
#define CMD_WRITE_ENABLE		0x06
#define CMD_ERASE_4K			0x20
#define CMD_ERASE_32K			0x52
#define CMD_ERASE_64K			0xd8
#define CMD_ERASE_CHIP			0xc7

/* flash status */
#define STATUS_BUSY			0x01

#define ERASE_4K_SIZE            (4096)
#define ERASE_32K_SIZE           (32768)
#define ERASE_64K_SIZE           (65536)

struct spi_flash {
    uint32_t size;
    uint32_t page_size;
    uint32_t sector_size;
};

int32_t spi_flash_write(uint32_t offset, uint32_t len, const void *buf);
int32_t spi_flash_read(uint32_t offset, uint32_t data_len, void *data);
int32_t spi_flash_erase_block(uint32_t offset, uint32_t blkSize);
int32_t spi_flash_erase_sector(uint32_t offset, uint32_t len);
int32_t spi_flash_erase_all(void);
int32_t spi_flash_write_status(uint8_t sts_reg);
int32_t spi_flash_readid(uint8_t *vendorId, uint8_t devId[]);
int32_t spi_flash_init_winbond(uint32_t pageSize, uint32_t sectorSize, uint32_t sectorNum);
void spi_flash_drv_init(uint32_t bus, uint32_t cs);

#endif /* __SPI_FLASH_H__ */ 
