/**HEADER********************************************************************
 Copyright (c) 2012-2013 Freescale Semiconductor, Inc.
 ALL RIGHTS RESERVED.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT, 
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
*                                                                           
*        Standard Software Flash Driver For FTFx                            
*                                                                           
* FILE NAME     :  FTFx_DX_(64_48_32_16)K_0K_0K_1K_0K.h                     
* DATE          :  Dec 25, 2013                                             
*                                                                           
* AUTHOR        :  FPT Team                                                 
* E-mail        :  r56611@freescale.com                                     
*                                                                           
******************************************************************************
0.0.1       08.07.2012      FPT Team      Initial Version
0.1.0       03.16.2013      FPT Team      Update prototype for FlashReadResource(),
                                          FlashProgramLongword() functions.
                                          Add new macros and remove unneccessary ones.
                                          Add FlashLaunchCommand() prototype.
0.1.1       06.20.2013      FPT Team      Update function prototype of 
                                          FlashProgramCheck by removing pFailData
1.0.0       12.25.2013      FPT Team      Modify define macros for checking alignment of a function
*END*************************************************************************/

#ifndef _FTFx_DX_64_48_32_16K_0K_0K_1K_0K_H_
#define _FTFx_DX_64_48_32_16K_0K_0K_1K_0K_H_

#include "SSD_Types.h"
#include "SSD_FTFx_Common.h"

#define FTFA_M  /* this is FTFA module, so some commands are not available */

#define BYTE2WORD(x) ((x)>>1)
#define WORD2BYTE(x) ((x)<<1)

/* Endianness */
#define ENDIANNESS              LITTLE_ENDIAN

/* cpu core */
#define CPU_CORE                DSC_56800EX   

/* PFlash sector size */ 
#define FTFx_PSECTOR_SIZE       0x00000400      /* 1 KB size */
#define DEBLOCK_SIZE            0x00000000      /* 0 KB size */


/* Address offset and size of PFlash IFR and DFlash IFR */
#define PFLASH_IFR_OFFSET       0x00000000
#define PFLASH_IFR_SIZE         0x00000100
#define DFLASH_IFR_OFFSET       0x00000000 
#define DFLASH_IFR_SIZE         0x00000100 

/* Size for checking alignment of a section */
#define PERSSEC_ALIGN_SIZE      FTFx_LONGWORD_SIZE      /* check align of erase sector on PFlash function */
#define PGMCHK_ALIGN_SIZE       FTFx_LONGWORD_SIZE      /* check align of program check function */
#define PRD1SEC_ALIGN_SIZE      FTFx_LONGWORD_SIZE      /* check align of verify section on PFlash function */
#define RDRSRC_ALIGN_SIZE       FTFx_LONGWORD_SIZE      /* check align of read resource function */

#define PGM_SIZE_BYTE           FTFx_LONGWORD_SIZE

#define RESUME_WAIT_CNT         0x20
                                          
/*---------------- Function Prototypes for Flash SSD --------------------*/

/* Flash initialization */
extern UINT32 FlashInit(PFLASH_SSD_CONFIG PSSDConfig);

/* Internal function. Called by driver APIs only */
/* Flash command sequence */
extern UINT32 FlashCommandSequence(PFLASH_SSD_CONFIG pSSDConfig);

/* P-Flash get protection */
extern UINT32 PFlashGetProtection(PFLASH_SSD_CONFIG PSSDConfig, \
                                  UINT32*  protectStatus);      

/* P-Flash set protection */
extern UINT32 PFlashSetProtection(PFLASH_SSD_CONFIG PSSDConfig, \
                                  UINT32  protectStatus);                                                
 
 
/* Flash get security state */
extern UINT32 FlashGetSecurityState(PFLASH_SSD_CONFIG PSSDConfig, \
                                    UINT8* securityState);

/* Flash security bypass */
extern UINT32 FlashSecurityBypass(PFLASH_SSD_CONFIG PSSDConfig, \
                                  UINT8* keyBuffer, \
                                  pFLASHCOMMANDSEQUENCE FlashCommandSequence);

/* Flash erase all Block */
extern UINT32 FlashEraseAllBlock(PFLASH_SSD_CONFIG PSSDConfig, \
                                 pFLASHCOMMANDSEQUENCE FlashCommandSequence);
                                    
/* Flash erase sector */
extern UINT32 FlashEraseSector(PFLASH_SSD_CONFIG PSSDConfig, \
                               UINT32 destination, \
                               UINT32 size, \
                               pFLASHCOMMANDSEQUENCE FlashCommandSequence);

/* Flash erase suspend */
extern UINT32 FlashEraseSuspend(PFLASH_SSD_CONFIG PSSDConfig);

/* Flash erase resume */
extern UINT32 FlashEraseResume(PFLASH_SSD_CONFIG PSSDConfig);                                          
   
/* Flash checksum */
extern UINT32 FlashCheckSum(PFLASH_SSD_CONFIG PSSDConfig, \
                            UINT32 destination, \
                            UINT32 size, \
                            UINT32* PSum);
                                           
/* Flash verify all block */
extern UINT32 FlashVerifyAllBlock(PFLASH_SSD_CONFIG PSSDConfig, \
                                  UINT8 marginLevel, \
                                  pFLASHCOMMANDSEQUENCE FlashCommandSequence);

/* Flash verify section*/
extern UINT32 FlashVerifySection(PFLASH_SSD_CONFIG PSSDConfig, \
                                 UINT32 destination, \
                                 UINT16 Number, \
                                 UINT8 marginLevel, \
                                 pFLASHCOMMANDSEQUENCE FlashCommandSequence);        

/* Flash read once */
extern UINT32 FlashReadOnce(PFLASH_SSD_CONFIG PSSDConfig, \
                            UINT8 recordIndex,\
                            UINT8* pDataArray, \
                            pFLASHCOMMANDSEQUENCE FlashCommandSequence);

/* Flash program once */
extern UINT32 FlashProgramOnce(PFLASH_SSD_CONFIG PSSDConfig, \
                               UINT8 recordIndex,\
                               UINT8* pDataArray, \
                               pFLASHCOMMANDSEQUENCE FlashCommandSequence);
/* Flash program check */
extern UINT32 FlashProgramCheck(PFLASH_SSD_CONFIG PSSDConfig, \
                                UINT32  destination, \
                                UINT32  size, \
                                UINT8*  pExpectedData, \
                                UINT32* pFailAddr, \
                                UINT8   marginLevel, \
                                pFLASHCOMMANDSEQUENCE FlashCommandSequence);
                                        
/* Flash read resource */
extern UINT32 FlashReadResource(PFLASH_SSD_CONFIG PSSDConfig, \
                                UINT32 destination, \
                                UINT8* pDataArray, \
                                UINT8  resourceSelectCode, \
                                pFLASHCOMMANDSEQUENCE FlashCommandSequence); 
                                         
/* Flash program Longword */
extern UINT32 FlashProgram(PFLASH_SSD_CONFIG PSSDConfig, \
                                   UINT32 destination, \
                                   UINT32 size, \
                                   UINT8* pData, \
                                   pFLASHCOMMANDSEQUENCE FlashCommandSequence);
#endif  /* _FTFx_DX_(64_48_32_16)K_0K_0K_1K_0K_H_ */
