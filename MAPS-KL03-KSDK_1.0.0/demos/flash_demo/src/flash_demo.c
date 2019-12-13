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
#include "fsl_debug_console.h"
#include "SSD_FTFx.h"
#include <string.h>
#include <stdio.h>

/********************************************************/
/*      Global Variables                                */
/********************************************************/
UINT8 DataArray[PGM_SIZE_BYTE];
UINT8 program_buffer[BUFFER_SIZE_BYTE];
UINT32 gCallBackCnt; /* global counter in callback(). */
pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)0xFFFFFFFF;

/* array to copy __Launch_Command func to RAM */
UINT16 __ram_func[LAUNCH_CMD_SIZE/2];
UINT16 __ram_for_callback[CALLBACK_SIZE/2]; /* length of this array depends on total size of the functions need to be copied to RAM*/

/************************************************************************************************/
/************************************************************************************************/
/*                      Flash Standard Software Driver Structure                                */
/************************************************************************************************/
/************************************************************************************************/
FLASH_SSD_CONFIG flashSSDConfig =
{
    FTFx_REG_BASE,          /* FTFx control register base */
    PFLASH_BLOCK_BASE,      /* base address of PFlash block */
    PBLOCK_SIZE,            /* size of PFlash block */
    DEFLASH_BLOCK_BASE,     /* base address of DFlash block */
    0,                      /* size of DFlash block */
    EERAM_BLOCK_BASE,       /* base address of EERAM block */
    0,                      /* size of EEE block */
    DEBUGENABLE,            /* background debug mode enable bit */
    NULL_CALLBACK           /* pointer to callback function */
};

/*********************************************************************
*
*  Function Name    : main
*  Description      : Main function
*                     Use Standard Software Drivers (SSD) to modify flash
*
*  Arguments        : void
*  Return Value     : UNIT32
*
**********************************************************************/
int main(void)
{
    UINT32 ret;          /* Return code from each SSD function */
    UINT32 destination;         /* Address of the target location */
    UINT32 size;
    UINT32 end;    
    UINT8  securityStatus;      /* Return protection status */
    UINT16 number;      /* Number of longword or phrase to be program or verify*/    
    UINT32 *p_data;    
    UINT32 margin_read_level;   /* 0=normal, 1=user - margin read for reading 1's */
#if (DEBLOCK_SIZE != 0)
    UINT8  protectStatus;           /* Store Protection Status Value of DFLASH or EEPROM */
#endif        
#if (defined(SWAP))
    UINT32 *p_source, *p_destination;
    UINT8 *source_data;
#else
    UINT32 i, FailAddr;
#endif
    gCallBackCnt = 0;
    
    CACHE_DISABLE;
  
    /* initialize device ports and pins */
    hardware_init();   
    
    /* connect board uart to stdout and stdin for terminal messaging */
    dbg_uart_init();
    
    /**************************************************************************
    *                               FlashInit()                               *
    * Setup flash SSD structure for device and initialize variables           *
    ***************************************************************************/
    ret = FlashInit(&flashSSDConfig);
    if (FTFx_OK != ret)
    {
        ErrorTrap(ret);
    }     
    
    /****************************************/
    /* print welcome message for flash demo */
    /****************************************/
    printf("\n\n\n\r*****************************************************************");
    printf("\n\r*\t\tWelcome to the Flash Demo!");
    printf("\n\r*");
    printf("\n\r*  This demo will erase and program different regions of ");
    printf("\n\r*  flash memory, and perform flash swap if it is supported. ");    
    printf("\n\r*");
#if (defined(FLASH_TARGET))
    printf("\n\r*\t- This demo is running from Flash Memory Space -");
#elif (defined(RAM_TARGET))
    printf("\n\r*\t- This demo is running from SRAM Memory Space -");
#endif
    printf("\n\r*");
        
    /***************************************************************/
    /* Print flash information - PFlash, DFlash, EEE if they exist */
    /***************************************************************/
    printf("\n\r*\tFlash Information: \n\r-----------------------------------------------------------------");
    printf("\n\r*\tTotal Flash Size:\t%d KB, Hex: (0x%x)", (PBLOCK_SIZE/ONE_KB), PBLOCK_SIZE);
    printf("\n\r*\tFlash Sector Size:\t%d KB, Hex: (0x%x)", (FTFx_PSECTOR_SIZE/ONE_KB), FTFx_PSECTOR_SIZE);
    printf("\n\r*");
    
    /*************************************/
    /* Does DFlash exist on this device? */
    /*************************************/
    if (flashSSDConfig.DFlashBlockSize) 
    {   
      printf("\n\r*\tData Flash Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig.DFlashBlockSize/ONE_KB), (unsigned int)flashSSDConfig.DFlashBlockSize);
      printf("\n\r*\tData Flash Base Address:\t0x%x", (unsigned int)flashSSDConfig.DFlashBlockBase);
    }
    else
    {
      printf("\n\r*\tNo D-Flash (FlexNVM) Present on this Device..."); 
    }

    /******************************************/
    /* Does FlexMemory Exist on this device ? */
    /******************************************/
    if (flashSSDConfig.EEEBlockSize) 
    {   
      printf("\n\r*\tEnhanced EEPROM (EEE) Block Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig.EEEBlockSize/ONE_KB), (unsigned int)flashSSDConfig.EEEBlockSize);
      printf("\n\r*\tEnhanced EEPROM (EEE) Base Address:\t0x%x", (unsigned int)flashSSDConfig.EERAMBlockBase);
    }
    else
    {
      printf("\n\r*\tNo Enhanced EEPROM (EEE) Present on this Device..."); 
    }    
    
    /**************************************/
    /* Is Swap Supported on this device ? */
    /**************************************/
#if (defined(SWAP))
    
    printf("\n\r*\tSwap is Supported on this Device..."); 
    
#else
    
    printf("\n\r*\tSwap is NOT Supported on this Device...");

#endif  
    
    printf("\n\r*****************************************************************");
    
    printf("\n\n\r....................Now Running Demo..................\n");   
    
    /*********************************************/
    /* END: print welcome message for flash demo */
    /*********************************************/    
        
    /**************************************************************************
      * Set CallBack to callback function 
    ***************************************************************************/
    flashSSDConfig.CallBack = (PCALLBACK)RelocateFunction((UINT32)__ram_for_callback , CALLBACK_SIZE , (UINT32)callback);     
    g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)RelocateFunction((UINT32)__ram_func , LAUNCH_CMD_SIZE ,(UINT32)FlashCommandSequence);         
                 
    /**************************************************************************
    *       Erase only select areas because we are running from Flash
    ***************************************************************************/
    
    /**************************************************************************
    ***************************************************************************
    *       Demo:   FlashEraseSector()  and FlashVerifySection()              *
    ***************************************************************************
    ***************************************************************************/
    /* Debug message for user */
    printf("\n\n\r---->Demo: Running FlashEraseSector() and FlashVerifySection()...");
    
    /************************************************************************/
    /* Erase several sectors on upper pflash block where there is no code */    
    /************************************************************************/
    destination = flashSSDConfig.PFlashBlockBase + BYTE2WORD(flashSSDConfig.PFlashBlockSize - 6*FTFx_PSECTOR_SIZE);
    end = destination + 3*FTFx_PSECTOR_SIZE;    /* erase and program two sectors */
    while ((destination + BYTE2WORD(FTFx_PSECTOR_SIZE)) < end)
    {
        size = FTFx_PSECTOR_SIZE;
        ret = FlashEraseSector(&flashSSDConfig, destination, size, g_FlashLaunchCommand);        
        if (FTFx_OK != ret)
        {
            ErrorTrap(ret);
        }

        /* Verify section for several sector of PFLASH */
        number = FTFx_PSECTOR_SIZE/PRD1SEC_ALIGN_SIZE;
        for(margin_read_level = 0; margin_read_level < 0x2; margin_read_level++)
        {
            ret = FlashVerifySection(&flashSSDConfig, destination, number, margin_read_level, g_FlashLaunchCommand);
            if (FTFx_OK != ret)
            {
                ErrorTrap(ret);
            }
        }        
 
        /* print message for user */
        printf("\n\r\tDemo:  Successfully Erased Sector 0x%x -> 0x%x", (unsigned int)destination, (unsigned int)(destination+size));
        
        destination += BYTE2WORD(size);                
    }  
    
    /**************************************************************************
    *                          FlashGetSecurityState()                        *
    ***************************************************************************/
    securityStatus = 0x0;
    ret = FlashGetSecurityState(&flashSSDConfig, &securityStatus);

    if (FTFx_OK != ret)
    {
        ErrorTrap(ret);
    }
    
    /**************************************************
    Message to user on flash security state     
      #define FLASH_NOT_SECURE                   0x01
      #define FLASH_SECURE_BACKDOOR_ENABLED      0x02
      #define FLASH_SECURE_BACKDOOR_DISABLED     0x04
    ****************************************************/
    switch(securityStatus)
    {
      case 1:
      default:
          printf("\n\n\r---->Flash is UNSECURE!");
          break;
      case 2:
          printf("\n\n\r---->Flash is SECURE, BACKDOOR is ENABLED!");
          break;
      case 3:
        printf("\n\n\r---->Flash is SECURE, BACKDOOR is DISABLED!");
        break;
    }

    /**************************************************************************
     *                          FlashReadResource()                            *
     ***************************************************************************/
     /* Read on P-Flash */
     destination = flashSSDConfig.PFlashBlockBase + PFLASH_IFR; /* Start address of Program Once Field */
     ret = FlashReadResource(&flashSSDConfig, destination, DataArray, 0x0, g_FlashLaunchCommand);

     if (FTFx_OK != ret)
     {
         ErrorTrap(ret);
     }
     
      /* Message to user */
      p_data = (UINT32 *)&DataArray;
      printf("\n\n\r---->Reading flash IFR @ location 0x%x: 0x%x", (unsigned int)destination, (unsigned int)(*p_data));
    
#if (defined(SWAP))
    
    /* program application data if we have not yet initialized the device */  
    if (DEMO_LOCATIONS_ARE_BLANK)
    {
      /************************************************************************
      *
      *
      *       Copy (program) Lower block to Upper block so we can swap 
      *
      *       Example:  
      *           K64F:   Swap is supported
      *                   Lower block: 0x0000 -> 0x8000
      *                   Upper block: 0x8000 -> 0x1_0000
      *           K22F:   Swap is NOT supported
      *
      *       Details:
      *
      *     Swap allows either half of program flash to exist at relative 
      *     address 0x0000.  So, different applicaton code can run out of reset,
      *     following a swap.  The Swap command must run from SRAM to avoid
      *     read-while-write errors when running from Flash.  
      *     The application can still run from Flash, but launching the 
      *     command (clearing CCIF bit) must be executed from SRAM.        
      *
      */
      /************************************************************************/
      /* Message to user */
      printf("\n\n\r---->Programming this application to upper block before swap....");
      
      /* setup parameters to program upper block */
      size = 6*FTFx_PSECTOR_SIZE;                         /* size of application */
      destination = UPPER_BLOCK_START_ADDRESS;            /* beginning of upper block */
      source_data = (UINT8 *)LOWER_BLOCK_START_ADDRESS;   /* beginning of lower block */
      end = destination + size;
      
      /***************************************************************************/
      /* Erase upper block */
      /***************************************************************************/
      ret = FlashEraseBlock(&flashSSDConfig, destination, g_FlashLaunchCommand);
      if (FTFx_OK != ret)
      {
          printf("\n\rFlash Erase Upper Block Error, Address: 0x%x", (int)destination);
          ErrorTrap(ret);
      }                
      
      /*******************************************************************/
      /* Program upper block with exact same data from lower block. */    
      /* This includes security information, for use when we swap blocks */
      /*******************************************************************/
      ret = FlashProgram(&flashSSDConfig, destination, size, \
                             source_data, g_FlashLaunchCommand);
      if (FTFx_OK != ret)
      {
          printf("\n\rFlashProgram Error, Block program, Address: 0x%x", (int)destination);
          ErrorTrap(ret);
      }

      /*************************************************/
      /* Verify application was programmed correctly  */
      /*************************************************/
      p_source = (UINT32 *)LOWER_BLOCK_START_ADDRESS;
      p_destination = (UINT32 *)UPPER_BLOCK_START_ADDRESS;
      do
      {       
        if (*p_source++ != *p_destination++)
        {
            ErrorTrap(destination);
        }
                                               
      }  while (p_destination < (UINT32 *)end);
      
      printf("\n\r\tSuccessfully programmed and verified upper block!");
    
    } /* setup data to swap */
    
    /*************************************
    * Execute Swap here
    * Blocks will swap at the next reset 
    ***************************************/    
    /* Message to user */
    printf("\n\n\r........ Swapping Flash Blocks! ..........");

    /* Run Swap */
    ret = flash_swap();
    
    /* Check error and message user */
    if (FTFx_OK != ret)
    {
      printf("\n\n\r....Flash Swap Demo Failed!  Check hardware and/or software!....");
      ErrorTrap(ret);
    }
    else
    {
      printf("\n\n\r\t---->Flash Swap Demo Success!<----");    
      print_swap_application_data();
      printf("\n\n\r\t---->Application data will swap locations after next reset...");
    }       

#else  /* defined(SWAP) */
    
    /********************************************************************
    *   For devices without SWAP, program some data for demo purposes 
    *********************************************************************/
    destination = flashSSDConfig.PFlashBlockBase + BYTE2WORD(flashSSDConfig.PFlashBlockSize - 6*FTFx_PSECTOR_SIZE);
    end = flashSSDConfig.PFlashBlockBase + BYTE2WORD(flashSSDConfig.PFlashBlockSize - 4*FTFx_PSECTOR_SIZE);
    for (i = 0; i < BUFFER_SIZE_BYTE; i++)
    {
        /* Set source buffer */
        program_buffer[i] = i;
    }
    size = BUFFER_SIZE_BYTE;
       
    /* message for user */
    printf("\n\n\r---->Running FlashProgram() and FlashProgramCheck()...");
    
    while ((destination + BYTE2WORD(size)) < end)
    {
        ret = FlashProgram(&flashSSDConfig, destination, size, \
                                       program_buffer, g_FlashLaunchCommand);
        if (FTFx_OK != ret)
        {
            ErrorTrap(ret);
        }
               
        /* Program Check user margin levels*/
        for (margin_read_level = 1; margin_read_level < 0x2; margin_read_level++)
        {
            ret = FlashProgramCheck(&flashSSDConfig, destination, size, program_buffer, \
                                        &FailAddr, margin_read_level, g_FlashLaunchCommand);
            if (FTFx_OK != ret)
            {
                ErrorTrap(ret);
            }
        }
        
        printf("\n\r\tSuccessfully Programmed and Verified Location 0x%x -> 0x%x", (unsigned int)destination, (unsigned int)(destination + size));
        
        destination += BYTE2WORD(BUFFER_SIZE_BYTE);
    }
    
#endif /* defined(SWAP) */
         
    /* Message to user */
    printf("\n\n\n\r--------------------------------");
    printf("\n\r----- Flash Demo Complete! -----");
    printf("\n\r--------------------------------");
           
    while(1);            
}

/*********************************************************************
*
*  Function Name    : ErrorTrap
*  Description      : Gets called when an error occurs.
*  Arguments        : UINT32
*  Return Value     :
*
*********************************************************************/
void ErrorTrap(UINT32 ret)
{
    printf("\n\n\n\r\t---- HALTED DUE TO FLASH ERROR! ----"); 
    while (1)
    {
        ;
    }
}

/*********************************************************************
*
*  Function Name    : callback
*  Description      : callback function for flash operations
*  Arguments        : none
*  Return Value     :
*
*********************************************************************/
void callback(void)
{
    /* just increase this variable to observer that this callback() func has been invoked */
    gCallBackCnt++;
}

/*******/
/* EOF */
/*******/
