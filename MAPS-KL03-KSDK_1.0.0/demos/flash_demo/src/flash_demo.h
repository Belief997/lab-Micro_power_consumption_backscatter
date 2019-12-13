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

#ifndef _FLASH_DEMO_H_
#define _FLASH_DEMO_H_

#include "user_cfg.h"
#include "fsl_device_registers.h"
#include "board.h"
#include "SSD_Types.h"
#include "SSD_FTFx_Internal.h"
#include "SSD_FTFx_Common.h"
#include "SSD_FTFx.h"

/************************************************************************************************/
/************************************************************************************************/
/*                      K64FN Setup Definitions                                                 */
/************************************************************************************************/
/************************************************************************************************/
#if (defined(CPU_MK64FN1M0VMD12))

#define BUFFER_SIZE_BYTE          0x100

#define EE_ENABLE                 0x00
#define RAM_ENABLE                0xFF
#define DEBUGENABLE               0x00

#define PSECTOR_SIZE              0x00001000    /* 4 KB size */
#define DSECTOR_SIZE              0x00001000    /* 4 KB size */

/* FTFL module base */
#define FTFx_REG_BASE             0x40020000
#define PFLASH_BLOCK_BASE         0x00000000
#define DEFLASH_BLOCK_BASE        0xFFFFFFFF
#define EERAM_BLOCK_BASE          0x14000000

#define PBLOCK_SIZE               0x00100000     /* 1024 KB size */
#define EERAM_BLOCK_SIZE          0x00001000     /* 4 KB size */

#define PBLOCK_NUM                2             /* number of individual Pflash block */
#define DBLOCK_NUM                0             /* number of individual Dflash block */

/* destination to program security key back to flash location */
#define SECURITY_LOCATION         0x408
#define BACKDOOR_KEY_LOCATION     0x400

#define PFLASH_IFR                0x3C0         /*Program flash IFR map*/
#define DFLASH_IFR                0x3F8         /*Data flash IFR map*/

#define EEE_DATA_SIZE_CODE        0x22
#define DE_PARTITION_CODE         0x03

#define PASS   0x00
#define FAIL   0xFF

#define READ_NORMAL_MARGIN        0x00
#define READ_USER_MARGIN          0x01
#define READ_FACTORY_MARGIN       0x02

/****************************************************************************/
/* Use an address towards the end of P-Flash block for the swap indicator.  */
/* The swap indicator is managed by the swap system, but is a location in */
/* PFlash and it gets modified (erased & programmed) throughout the swap */
/* process, so it cannot share space with normal application code */
/* Here, we are using the second-to-last sector, since the Lower/Upper */
/* data blocks use the last sector are used to hold our test data to help */
/* identify each block after the swap */
/****************************************************************************/
#define PSWAP_INDICATOR_ADDR      (PBLOCK_SIZE/PBLOCK_NUM - (2*FTFx_PSECTOR_SIZE))
/* The Lower & Upper Data sectors are used to program test data into, to */
/* help identify each block - for debug purposes.  */
#define PSWAP_LOWERDATA_ADDR      (PSWAP_INDICATOR_ADDR + FTFx_PSECTOR_SIZE)
#define PSWAP_UPPERDATA_ADDR      (PSWAP_LOWERDATA_ADDR + PBLOCK_SIZE/2)

/* swap state in FCCOB5*/
#define FTFx_SWAP_STATE_UNINIT       0x0
#define FTFx_SWAP_STATE_READY        0x1
#define FTFx_SWAP_STATE_UPDATE       0x2
#define FTFx_SWAP_STATE_UPDATE_ERS   0x3
#define FTFx_SWAP_STATE_COMPLETE     0x4

#define CC_ISR_NUM                34
#define RDCOL_ISR_NUM             35

/* Cache disable macro */
#define CACHE_DISABLE        	     FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK | FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK);\
                                     FMC_PFB1CR &= ~(FMC_PFB1CR_B1SEBE_MASK | FMC_PFB1CR_B1IPE_MASK | FMC_PFB1CR_B1DPE_MASK | FMC_PFB1CR_B1ICE_MASK | FMC_PFB1CR_B1DCE_MASK);\

void ErrorTrap(UINT32 returnCode);

/************************************************************************************************/
/************************************************************************************************/
/*                      KL03Z Setup Definitions                                                 */
/************************************************************************************************/
/************************************************************************************************/
#elif (defined(CPU_MKL03Z32VFK4))

#define BUFFER_SIZE_BYTE          0x80

#define EE_ENABLE                 0x00
#define RAM_ENABLE                0xFF
#define DEBUGENABLE               0x00

#define PSECTOR_SIZE              0x400
#define DSECTOR_SIZE              0x400 

/* FTFA module base */
#define FTFx_REG_BASE             0x40020000
#define PFLASH_BLOCK_BASE         0x00000000
#define DEFLASH_BLOCK_BASE        0xFFFFFFFF  
#define EERAM_BLOCK_BASE          0xFFFFFFFF

#define PBLOCK_SIZE               0x00008000      /* 32 KB size */
#define EERAM_BLOCK_SIZE          0x00000000      /* 0 KB size */

#define PBLOCK_NUM                1 /* number of individual Pflash block */

/* destination to program security key back to flash location */
#define SECURITY_LOCATION         0x40C
#define BACKDOOR_KEY_LOCATION     0x400

#define PFLASH_IFR                0xC0

/******************************************************/
//#define DFLASH_IFR                0xFC

// #define EEE_DATA_SIZE_CODE        0x22 //2048 : 2048 Byte
// #define DE_PARTITION_CODE         0x03 //1:1

// #define READ_NORMAL_MARGIN        0x00
// #define READ_USER_MARGIN          0x01
// #define READ_FACTORY_MARGIN       0x02

// #define PFLASH_START_ADDR         PFLASH_BLOCK_BASE /* PFlash start address */
// #define DFLASH_START_ADDR         DEFLASH_BLOCK_BASE /* DFlash start address */

// #define PSWAP_INDICATOR_ADDR      0x2000
// #define PSWAP_LOWERDATA_ADDR      (PSWAP_INDICATOR_ADDR + PSECTOR_SIZE) 
// #define PSWAP_UPPERDATA_ADDR      (PSWAP_LOWERDATA_ADDR + PBLOCK_SIZE/2 + PSECTOR_SIZE)

#define PASS                       0x00
#define FAIL                       0xFF

/* swap state in FCCOB5*/
// #define FTFx_SWAP_STATE_UNINIT       0x0
// #define FTFx_SWAP_STATE_READY        0x1
// #define FTFx_SWAP_STATE_UPDATE       0x2
// #define FTFx_SWAP_STATE_UPDATE_ERS   0x3
// #define FTFx_SWAP_STATE_COMPLETE     0x4

#define CC_ISR_NUM                5
#define RDCOL_ISR_NUM             5

/* Disable cache */
#define CACHE_DISABLE             BW_MCM_PLACR_DFCS(MCM_BASE, 1); \
                                  BW_MCM_PLACR_ESFC(MCM_BASE, 1);
 
void ErrorTrap(UINT32 ret);

/************************************************************************************************/
/************************************************************************************************/
/*                      K22F and KV31F Setup Definitions                                        */
/************************************************************************************************/
/************************************************************************************************/
#elif ( defined(CPU_MK22FN512VDC12) || defined(CPU_MKV31F512VLL12)  || \
      defined(CPU_MK22FN256VDC12) || defined(CPU_MKV31F256VLL12)    || \
      defined(CPU_MK22FN128VDC10) || defined(CPU_MKV31F128VLL10)    || \
      defined(CPU_MK22FN512VLH12) )

#define BUFFER_SIZE_BYTE          0x100

#define EE_ENABLE                 0x00
#define RAM_ENABLE                0xFF
#define DEBUGENABLE               0x00

#define PSECTOR_SIZE              0x800
#define DSECTOR_SIZE              0x800

/* FTFA module base */
#define FTFx_REG_BASE             0x40020000
#define PFLASH_BLOCK_BASE         0x00000000
#define DEFLASH_BLOCK_BASE        0xFFFFFFFF
#define EERAM_BLOCK_BASE          0xFFFFFFFF

#if (defined(CPU_MK22FN512VDC12) || defined(CPU_MKV31F512VLL12) || \
     defined(CPU_MK22FN512VLH12) )
#define PBLOCK_SIZE               0x00080000      /* 512 KB size */
#elif (defined(CPU_MK22FN256VDC12) || defined(CPU_MKV31F256VLL12))
#define PBLOCK_SIZE               0x00040000      /* 256 KB size */
#elif (defined(CPU_MK22FN128VDC10) || defined(CPU_MKV31F128VLL10))
#define PBLOCK_SIZE               0x00020000      /* 128 KB size */
#endif 
#define EERAM_BLOCK_SIZE          0x00000000      /* 0 KB size */

#define PBLOCK_NUM                2 /* number of individual Pflash block */

/* destination to program security key back to flash location */
#define SECURITY_LOCATION         0x40C
#define BACKDOOR_KEY_LOCATION     0x400

#define PFLASH_IFR                0xC0

/******************************************************/
//#define DFLASH_IFR                0xFC

// #define EEE_DATA_SIZE_CODE        0x22 //2048 : 2048 Byte
// #define DE_PARTITION_CODE         0x03 //1:1

// #define READ_NORMAL_MARGIN        0x00
// #define READ_USER_MARGIN          0x01
// #define READ_FACTORY_MARGIN       0x02

// #define PFLASH_START_ADDR         PFLASH_BLOCK_BASE /* PFlash start address */
// #define DFLASH_START_ADDR         DEFLASH_BLOCK_BASE /* DFlash start address */

// #define PSWAP_INDICATOR_ADDR      0x2000
// #define PSWAP_LOWERDATA_ADDR      (PSWAP_INDICATOR_ADDR + PSECTOR_SIZE)
// #define PSWAP_UPPERDATA_ADDR      (PSWAP_LOWERDATA_ADDR + PBLOCK_SIZE/2 + PSECTOR_SIZE)

#define PASS                       0x00
#define FAIL                       0xFF

/* swap state in FCCOB5*/
// #define FTFx_SWAP_STATE_UNINIT       0x0
// #define FTFx_SWAP_STATE_READY        0x1
// #define FTFx_SWAP_STATE_UPDATE       0x2
// #define FTFx_SWAP_STATE_UPDATE_ERS   0x3
// #define FTFx_SWAP_STATE_COMPLETE     0x4

#define CC_ISR_NUM                34
#define RDCOL_ISR_NUM             35

/* Disable cache */
#define CACHE_DISABLE             FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK |FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK); \
                                  FMC_PFB1CR &= ~(FMC_PFB1CR_B1SEBE_MASK | FMC_PFB1CR_B1IPE_MASK | FMC_PFB1CR_B1DPE_MASK |FMC_PFB1CR_B1ICE_MASK | FMC_PFB1CR_B1DCE_MASK);

void ErrorTrap(UINT32 ret);

#endif /* if/else device definition */
/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/

/* Other defines */

#define ONE_KB                  1024                        //0x400:  10 zeros
#define TWO_KB                  (2*ONE_KB)
#define THREE_KB                (3*ONE_KB)
#define FOUR_KB                 (4*ONE_KB)
#define FIVE_KB                 (5*ONE_KB)
#define SIX_KB                  (6*ONE_KB)
#define SEVEN_KB                (7*ONE_KB)
#define EIGHT_KB                (8*ONE_KB)
#define NINE_KB                 (9*ONE_KB)
#define TEN_KB                  (10*ONE_KB)
#define ONE_MB                  (ONE_KB*ONE_KB)             //0x100000:     20 zeros
#define ONE_GB                  (ONE_KB*ONE_KB*ONE_KB)      //0x40000000:   30 zeros

#define WORD_SIZE               4
#define BLANK_DATA              0xFFFFFFFF

#define NORMAL_MARGIN_READ      0   /* normal margin read reference */
#define USER_MARGIN_READ        1   /* use for test scenarios for bit shifts */

#define UPPER_BLOCK_START_ADDRESS   (flashSSDConfig.PFlashBlockBase + BYTE2WORD(flashSSDConfig.PFlashBlockSize/PBLOCK_NUM))
#define LOWER_BLOCK_START_ADDRESS   (flashSSDConfig.PFlashBlockBase)

#define SWAP_STATUS_BIT \
  (REG_READ(FTFx_REG_BASE + FTFx_SSD_FCNFG_OFFSET) & FTFE_FCNFG_SWAP_MASK)
#define DEMO_LOCATIONS_ARE_BLANK \
  ((READ32(PSWAP_LOWERDATA_ADDR) == 0xFFFFFFFF) && (READ32(PSWAP_UPPERDATA_ADDR) == 0xFFFFFFFF))

/************************************************************/
/* prototypes                                               */
/************************************************************/
void callback(void);
extern UINT32 RelocateFunction(UINT32 dest, UINT32 size, UINT32 src);
void print_welcome_message(void);
#if (defined(SWAP))
UINT32 flash_swap(void);
void run_flash_swap(void);
void print_swap_application_data(void);
#endif /* #if (defined(SWAP)) */

#endif /* _FLASH_DEMO_H_ */
