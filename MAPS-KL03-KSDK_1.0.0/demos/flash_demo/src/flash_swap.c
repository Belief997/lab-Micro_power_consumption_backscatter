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

/* include the header files */
#include "flash_demo.h"
#include <stdio.h>

extern FLASH_SSD_CONFIG flashSSDConfig;
extern pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand;

#if (PGM_SIZE_BYTE == 8)
UINT8 unsecure_key[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF};
#else
    #if(ENDIANNESS == LITTLE_ENDIAN)
    UINT8 unsecure_key[4] = {0xFE, 0xFF, 0xFF, 0xFF};
    #else
    UINT8 unsecure_key[4] = {0xFF, 0xFF, 0xFF, 0xFE};
    #endif
#endif
UINT8 pgmData[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
UINT8 ProgramApplicationCode(void);
BOOL SwapCallback(UINT8 currentSwapMode);

#if (defined(RAM_TARGET))
/* no need to have relocated SRAM function g_FlashLaunchCommand when we are already running from SRAM */
#define g_FlashLaunchCommand FlashCommandSequence
#endif

/**************************************************************************
    Flash Swap Example Code
 **************************************************************************/
#if (defined(SWAP))
UINT32 flash_swap(void)
{  
    uint32_t testResult;
    uint8_t currentSwapMode, currentSwapBlock, nextSwapBlock;
    
    /* If standalone, FlashInit() is required here */
    
    /**************************************************************************
    *                          Get current swap state                         *
    ***************************************************************************/    
    /* Important note:  Use g_FlashLaunchCommand instead of FlashCommandSequence to run CCIF from SRAM!  */
    /*                  This allows swap application to execute from Flash memory space while CCIF flag is written from SRAM! */    
    testResult = PFlashSwapCtl(&flashSSDConfig, PSWAP_INDICATOR_ADDR, FTFx_SWAP_REPORT_STATUS, &currentSwapMode, \
                    &currentSwapBlock, &nextSwapBlock, g_FlashLaunchCommand);
    
    if (FTFx_OK != testResult)
    {
        ErrorTrap(testResult);
    }  
        
    if (currentSwapMode == FTFx_SWAP_UNINIT)
    {  
        /**************************************************************************
        *      Call PFlashSwapCtl() to stop at each immediate state                *
        ***************************************************************************/
        
        /* after this call, swap state = UPDATE_ERS */        
        testResult = PFlashSwapCtl(&flashSSDConfig, PSWAP_INDICATOR_ADDR, FTFx_SWAP_SET_INDICATOR_ADDR, &currentSwapMode, &currentSwapBlock, &nextSwapBlock, g_FlashLaunchCommand);
        if ((FTFx_OK != testResult)||(currentSwapMode!= FTFx_SWAP_UPDATE_ERASED))
        {
            ErrorTrap(testResult);
        } 
            
        /* Erase non active swap indicator and program test data to lower/upper block */
        testResult = ProgramApplicationCode();
        if (FTFx_OK != testResult)
        {
            ErrorTrap(testResult);
        } 
        
        /* Progress to swap complete mode now that our indicator address is set, and we have programmed some data */        
        testResult = PFlashSwapCtl(&flashSSDConfig, PSWAP_INDICATOR_ADDR, FTFx_SWAP_SET_IN_COMPLETE, &currentSwapMode, \
                        &currentSwapBlock, &nextSwapBlock, g_FlashLaunchCommand);
    
        if ((FTFx_OK != testResult)||(currentSwapMode!= FTFx_SWAP_COMPLETE))
        {
            ErrorTrap(testResult);
        } 
          
        /************************************************************************************/
        /* Need to reset here to complete swap process                                       */
        /************************************************************************************/
    } 
    else /* swap system is in READY state */
    {
        /**************************************************************************
        *      Call PFlashSwap()                                                   *
        *       Swap is already initialized, meaning the swap indicator locations  *
        *       have been assigned and swap is ready to be executed                *
        ***************************************************************************/      
        testResult = PFlashSwap(&flashSSDConfig, PSWAP_INDICATOR_ADDR, SwapCallback, g_FlashLaunchCommand);
        if (FTFx_OK != testResult)
        {
            ErrorTrap(testResult);
        }        
        testResult = PFlashSwapCtl(&flashSSDConfig, PSWAP_INDICATOR_ADDR, FTFx_SWAP_REPORT_STATUS, &currentSwapMode, \
                        &currentSwapBlock, &nextSwapBlock, g_FlashLaunchCommand);        
        if ((FTFx_OK != testResult)||(currentSwapMode != FTFx_SWAP_COMPLETE))
        {
            ErrorTrap(testResult);
        }       
    }
    
    /*********************************************************************************************************/
    return(FTFx_OK);
}

/*******************************************************************************************/
/* This is setting up a small amount of data in upper block and lower block to */
/* show example of how the data gets mapped after a reset following a Swap.  */
/* When Swap is complete and the part is reset, the memory map of the first P-Flash block */
/* and the second P-Flash block are swapped */
/* For example, Data that resides at 0x7F000 will be put at 0xFF000, vice versa */
/*******************************************************************************************/
UINT8 ProgramApplicationCode(void)
{
    UINT32 ret = FTFx_OK;
    UINT8 i;
    
    /* erase non active swap indicator */
    ret |= FlashEraseSector(&flashSSDConfig, PSWAP_INDICATOR_ADDR + PBLOCK_SIZE/2, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);
 
    /* erase area in lower block to program application code */
    ret |= FlashEraseSector(&flashSSDConfig, PSWAP_LOWERDATA_ADDR, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);

    /* Program application code to lower block */
    ret |= FlashProgram(&flashSSDConfig, PSWAP_LOWERDATA_ADDR, PGM_SIZE_BYTE, pgmData, g_FlashLaunchCommand);

    /* erase area in upper block */
    ret |= FlashEraseSector(&flashSSDConfig, PSWAP_UPPERDATA_ADDR, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);

    /* program data to upper block for verification */
    for (i = 0; i < PGM_SIZE_BYTE; i ++)
    {
        pgmData[i] += 0xA0; 
    }
    ret |= FlashProgram(&flashSSDConfig, PSWAP_UPPERDATA_ADDR, PGM_SIZE_BYTE, pgmData, g_FlashLaunchCommand);

    return (ret);
}

BOOL SwapCallback(UINT8 currentSwapMode)
{
    UINT32 destination, ret = FTFx_OK;
    
    switch (currentSwapMode)
    {
        case FTFx_SWAP_UNINIT:
            /* Put your application-specific code here */
            printf("\n\rSwap Callback -> Swap flash is uninitialization status!");
            break;
            
        case FTFx_SWAP_READY:
            /* Put your application-specific code here */
            printf("\n\rSwap Callback -> Swap flash is initialization status!");
            break;        
            
        case FTFx_SWAP_UPDATE:
            /* Put your application-specific code here */   
            printf("\n\rSwap Callback -> Swap flash is update status!");
            /**************************************************************************/
            /* erase swap indicator address so we can move to update-erased state */          
            /*************************************************************************/            
            destination = PSWAP_INDICATOR_ADDR + PBLOCK_SIZE/2;
            ret = FlashEraseSector(&flashSSDConfig, destination, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);
            if (FTFx_OK != ret)
            {
                ErrorTrap(ret);
            }
            break;
            
        case FTFx_SWAP_UPDATE_ERASED:
            /* Put your application-specific code here */
            printf("\n\rSwap Callback -> swap flash is update erased status!");
            
            /* Erase non active swap indicator and Program example application code to lower and upper location */
            /* In a typical user mode, it may be desired to only update the upper block, then swap to run the new code */
            if (DEMO_LOCATIONS_ARE_BLANK)
            {
                /* Program new example code when lower block is mapped to address 0x0000 */
                /* Otherwise, this can be customized to program new application to upper block */
                /* and that will be executed after the next reset */
                ret = ProgramApplicationCode(); 
            }
            break;
            
        case FTFx_SWAP_COMPLETE:
            /* Put your application-specific code here */
            printf("\n\rSwap Callback -> swap flash is complete status!");
            break;
        default:
            break;
    }
    /* Return TRUE to continue swapping and FALSE to stop swapping */
    if (ret == FTFx_OK) return TRUE;
    else return FALSE;
}

/******************************************************************************/
/* Function: print_swap_application_data() */
/******************************************************************************/
/* Two locations are programmed with application data to show how the memory locations get swapped */      
void print_swap_application_data()
{
    printf("\n\r----------------------------------------------------------------");
    if (SWAP_STATUS_BIT)
    {   
        /* Swap status bit is set, upper block mapped to 0x0000 - printout */
        
        printf("\n\n\r@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        printf("\n\rSwap Status Bit: 1, UPPER block resides at location 0x0000_0000!");  
        printf("\n\r@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");        
    }
    else
    {
        /* Swap status bit is clear, lower block mapped to 0x0000 - printout  */
        printf("\n\n\r****************************************************************");
        printf("\n\rSwap Status Bit: 0, LOWER block resides at location 0x0000_0000!");   
        printf("\n\r****************************************************************");
    }
    printf("\n\r\tP-Flash Lower Test Data @(0x%x): 0x%x", PSWAP_LOWERDATA_ADDR, READ32(PSWAP_LOWERDATA_ADDR));
    printf("\n\r\tP-Flash Upper Test Data @(0x%x): 0x%x", PSWAP_UPPERDATA_ADDR, READ32(PSWAP_UPPERDATA_ADDR));    
}
    
#endif /* #if (defined(SWAP)) */

/**************************************************************************
    EOF
 ***************************************************************************/
