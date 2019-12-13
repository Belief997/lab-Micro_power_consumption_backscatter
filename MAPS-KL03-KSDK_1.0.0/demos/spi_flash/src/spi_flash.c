/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spi_flash_drv.h"
#include "fsl_os_abstraction.h"
#include "board.h"

char g_buf[FLASH_PAGE_SIZE];


#define VERIFY_PATTERN 0x5A

static const char gStrInputAddr[] = "\r\nInput the %s address(HEX), 0x";
static const char gStrInputLen[] = "\r\nInput the %s length:";
static const char gStrInvalid[] = "Invalid input!\r\n";
static const char gStrEraseFail[] = "Erase failed!\r\n";
static const char gStrEraseOk[] = "Erase OK!\r\n";

/*!
 * @brief receive the console input and echo
 *
 */
static void recv_from_console(char *buf, uint32_t size)
{
    uint32_t i;

    for (i = 0; i < size; i++)
    {
        buf[i] = getchar();
        if (buf[i] == '\r')
        {
            buf[i] = '\0'; 
            return;
        }
        putchar(buf[i]);
    }
}

static uint8_t show_menu(void)
{
    char input;

    printf("\r\n1 - Erase entire chip\r\n");
    printf("2 - Erase sectors\r\n");
    printf("3 - Erase block\r\n");
    printf("4 - Program one page with pattern (0x%x) and verify\r\n", VERIFY_PATTERN);
    printf("5 - Read byte\r\n");
    printf("Please enter your choice (1-5): ");

    input = getchar();
    printf("%c\r\n", input);

    return input - '0';
}

/* main function */
int main(void)
{
    //char input, buf[FLASH_PAGE_SIZE];
    char input;
    int32_t ret, blkSize;
    uint8_t vendorId;
    uint8_t devId[2];
    int32_t offset, length, i;

    hardware_init();
    dbg_uart_init();
    /* configure the SPI0 bus pins */
    configure_spi_pins(BOARD_SPI_FLASH_BUS);
    configure_spi_mem_cs_pins(BOARD_SPI_FLASH_BUS);
    GPIO_DRV_Init(NULL, spiMemCsPin);

    OSA_Init();

    printf("\r\n***SPI Flash Demo***\r\n");

    spi_flash_drv_init(BOARD_SPI_FLASH_BUS, BOARD_SPI_FLASH_CS);
    /* READ the SPI Flash ID */
    ret = spi_flash_readid(&vendorId, devId);
    if (ret)
    {
        printf("Can not find any SPI Flash device!\r\n");
        return -1;
    }

    /* check if it's WinBond W25Q80DV chip or not */
    if (SPI_FLASH_WINBOND_VENDER_ID == vendorId)
    {
        if (spi_flash_init_winbond(FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, 256))
        {
            printf("Failed to init SPI flash chip!\r\n");
            return -1;
        }
    }
    else
    {
        printf("Not a WinBond SPI flash chip! VID:%x\r\n", vendorId);
        return -1;
    }

    while (1)
    {

        char recvBuf[10];

        /* showing the UI */
        input = show_menu();

        offset = -1;

        switch (input)
        {
        case 1:
            /* erase all chips */
            ret = spi_flash_erase_all();
            if (ret)
            {
                printf(gStrEraseFail);
            }
            else
            {
                printf(gStrEraseOk);
            }
            break;

        case 2:
            printf(gStrInputAddr, "sector");
            recv_from_console(recvBuf, 8);
            sscanf(recvBuf, "%x",&offset);
            printf(gStrInputLen, "sector");
            recv_from_console(recvBuf, 8);
            sscanf(recvBuf, "%d",&length);
	    printf("\r\n");

            if (offset < 0 || offset >= FLASH_TOTAL_SIZE || length <= 0)
            {
                printf(gStrInvalid);
                break;
            }

            /* erase sector */
            ret = spi_flash_erase_sector(offset, length);
            if (ret)
            {
                printf(gStrEraseFail);
            }
            else
            {
                printf(gStrEraseOk);
            }
            break;

        case 3:
            printf(gStrInputAddr, "block");
            recv_from_console(recvBuf, 8);
            sscanf(recvBuf, "%x",&offset);
            printf("\r\n1. 32K block size");
            printf("\r\n2. 64K block size");
            printf("\r\nSelect the block size:");
            blkSize = getchar();
            blkSize -= '0';
            printf("%d\r\n", blkSize);

            if (offset < 0  || offset >= FLASH_TOTAL_SIZE
                            || (blkSize != 1 && blkSize != 2))
            {
                printf(gStrInvalid);
                break;
            }

	    /* erase one block */
            ret = spi_flash_erase_block(offset,
	                                blkSize==1?ERASE_32K_SIZE:ERASE_64K_SIZE);
            if (ret)
            {
                printf(gStrEraseFail);
            }
            else
            {
                printf(gStrEraseOk);
            }
            break;

        case 4:
            /* get the program address and length */
            printf(gStrInputAddr, "program");
            recv_from_console(recvBuf, 8);
            sscanf(recvBuf, "%x",&offset);
	    printf("\r\n");

            if (offset < 0 || offset > (FLASH_TOTAL_SIZE - FLASH_PAGE_SIZE))
            {
                printf(gStrInvalid);
                break;
            }

            /* erase sectors first */
            ret = spi_flash_erase_sector(offset & ~(FLASH_SECTOR_SIZE - 1),
                                         FLASH_SECTOR_SIZE);
	    if (ret)
            {
                printf(gStrEraseFail);
                break;
            }

            /* fill the write buffer with pattern */
            memset(g_buf, VERIFY_PATTERN, FLASH_PAGE_SIZE);
            ret = spi_flash_write(offset, FLASH_PAGE_SIZE, (const void*)g_buf);
	    if (ret)
            {
                printf("Program page failed!\r\n");
                break;
            }

	    /* read back and verify */
            memset(g_buf, 0, FLASH_PAGE_SIZE);
            ret = spi_flash_read(offset, FLASH_PAGE_SIZE, (void *)g_buf);
            if (ret)
            {
                printf("Read page failed!\r\n");
                break;
            }

            /* compare */
            for (i = 0; i < FLASH_PAGE_SIZE; i++)
            {
                if (g_buf[i] != VERIFY_PATTERN)
                {
                    printf("Program and verify failed!\r\n");
                    break;
                }
            }

            printf("Program and verify done!\r\n");
            break;

        case 5:
            /* get the read address */
            printf(gStrInputAddr, "read");
            recv_from_console(recvBuf, 8);
            sscanf(recvBuf, "%x", &offset);
	    printf("\r\n");

            if (offset < 0 || offset >= FLASH_TOTAL_SIZE)
            {
                printf(gStrInvalid);
                break;
            }

	    ret = spi_flash_read(offset, 1, (void *)g_buf);
            if (ret)
            {
                printf("Read byte failed!\r\n");
                break;
            }
            printf("0x%x = 0x%x\r\n", offset, g_buf[0]);
            break;

        default:
            printf(gStrInvalid);
            break;
        }
    }
}
