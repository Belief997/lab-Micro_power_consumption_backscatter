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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "device/fsl_device_registers.h"
#include "fsl_interrupt_manager.h"
#include "fsl_debug_console.h"
#include "fsl_sim_hal.h"
#include "fsl_port_hal.h"
#include "fsl_tpm_hal.h"
#include "fsl_lpuart_hal.h"
#include "fsl_cmp_hal.h"
#include "fsl_misc_utilities.h"
#include "fsl_clock_manager.h"
#include "fsl_lpuart_hal.h"
#include "board.h"


/*******************************************************************************
 * Defination
 ******************************************************************************/
#define IRC48M_CLK 1
#define IRC48M  48000000U
#define CHANNEL0 0
#define LPUART0_BAUDRATE 1200U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void TPM1_IRQHandler(void);
void LPUART0_IRQHandler(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t u32Cycles = 0;
static uint8_t u8ReceiveData[64];
volatile uint8_t u8SendData[64];
volatile uint8_t u8RXIndex=0;
volatile uint8_t u8TXIndex=0;
volatile uint8_t u8RX_Over=0;
volatile uint8_t u8TX_Over=0;
volatile uint8_t u8SendCommand = 1;
volatile uint8_t u8ReadCommand = 1;
volatile uint8_t u8ReadReady=0;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief main demo function.
 */
int main(void)
{
  uint8_t u8Increase;
  uint8_t u8Err=0; 
  
  uint32_t mcgirclk_freq;

  /* init hardware */
  hardware_init();

  configure_gpio_pins(1);
  GPIO_DRV_Init(NULL, ledPins);
  
  //GPIO_DRV_ClearPinOutput(kGpioLED1);
  //printf("\r\nir_uart demo running...\r\n");

  for(u8Increase=0;u8Increase<64;u8Increase++)
  {
    u8SendData[u8Increase]=u8Increase;
  }
  
  CLOCK_SYS_GetFreq(kMcgIrClock, &mcgirclk_freq);
  
  /*SIM registers configuration for LPUART0*/
  SIM_HAL_EnableLpuartClock(SIM_BASE, HW_LPUART0);    /* enable LPUART0 clock gate */
  SIM_HAL_SetLpUartTxSrcMode(SIM_BASE, HW_LPUART0, kSimUartTxsrcCmp0);/* LPUART0 TX pin is modulated with TPM1 channel 0 */
  CLOCK_HAL_SetSource(SIM_BASE, kClockLpuart0Src, IRC48M_CLK); /* select IRC48M as LPUART0 clock */ 
  //CLOCK_SYS_SetSource(kClockLpuart0Src, 1);
  
  /*SIM registers configuration for TPM1*/    
  SIM_HAL_EnableTpmClock(SIM_BASE, HW_TPM1);              /* enable TPM1 clock gate */         
  CLOCK_HAL_SetSource(SIM_BASE, kClockTpmSrc, IRC48M_CLK);    /* select IRC48M as TPM module clock */
  //CLOCK_HAL_SetSource(SIM_BASE, kClockTpmSrc, 1);    /* select IRC48M as TPM module clock */
  
  /*SIM registers configuration for CLKOUT pin*/    
  //CLOCK_HAL_SetSource(SIM_BASE, kClockClkoutSel, 1); /* select IRC48M as CLKOUT pin */     

  /* TPM1 configuration */ 
  PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAlt2);
  TPM_HAL_SetChnMsnbaElsnbaVal(TPM1_BASE, 0, 0x14);
  TPM_HAL_SetChnCountVal(TPM1_BASE, CHANNEL0, 315); /* set channel value */
  TPM_HAL_SetMod(TPM1_BASE, 631); /* produce 38KHz */   
  TPM_HAL_EnableTimerOverflowInt(TPM1_BASE);
  INT_SYS_EnableIRQ(TPM1_IRQn);   
  TPM_HAL_SetClockMode(TPM1_BASE, kTpmClockSourceModuleClk);
  
  /* configure PTB1 and PTB2 as LPUART0 function */ 
  PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAlt2);   
  PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAlt2); 
  /* LPUART0 module configuration */     
  LPUART_HAL_SetBaudRate(LPUART0_BASE, IRC48M, LPUART0_BAUDRATE); /* set LPUART0 baud rate to 2400bps */
  //LPUART_HAL_SetBaudRate(LPUART0_BASE, mcgirclk_freq, LPUART0_BAUDRATE); /* set LPUART0 baud rate to 2400bps */
  /* Setting TXINV inverts the LPUART_TX output for all cases: data bits, start and stop bits, break, and idle. */
  LPUART_HAL_SetTxRxInversionCmd(LPUART0_BASE, false, true);                     
  LPUART_HAL_EnableTransmitter(LPUART0_BASE);     /* enable transmitter */ 
  LPUART_HAL_EnableReceiver(LPUART0_BASE);        /* enable receiver */
  //LPUART0->CTRL |=LPUART_CTLTE_MASK |LPUART_RE_MASK;
  LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntTxDataRegEmpty, true);  
  LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntRxDataRegFull, true); 
  LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntRxOverrun, true);  
  LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntNoiseErrFlag, true); 
  LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntFrameErrFlag, true);    
  INT_SYS_EnableIRQ(LPUART0_IRQn);  
  
  
  
  /*reset read waiting time*/
  u32Cycles=0; 
  u8ReadReady=0;
  
  /*set board debug port baud rate to 2400bps as ir baud rate is limited  */     
  while(1)
  {  
    if(u8ReadReady)
    {  
      if(!(u8TX_Over))
      {           
        //printf("\r\ntransimitter is timeout\r\n");
        GPIO_DRV_ClearPinOutput(kGpioLED1);
      }  
      else
      {             
        /* set one second delay as timeout*/ 
        u8ReadReady=0; 
        while(!u8ReadReady);     
        if(u8RX_Over)
        {                 
          for(u8Increase=0;u8Increase<64;u8Increase++)
          {  
           if(u8SendData[u8Increase] != u8ReceiveData[u8Increase])      
           {  
             u8Err++;   
           }
          }
          if(u8Err)        
           {         
             //printf("\r\ncommunications fail\r\n");
             GPIO_DRV_ClearPinOutput(kGpioLED3);			 
           }            
          else          
          {         
            //printf("\r\ncommunications success\r\n");
            GPIO_DRV_ClearPinOutput(kGpioLED4);			
          }
        }        
        else    
        {    
         //printf("\r\nreceiver is timeout\r\n");     
         GPIO_DRV_ClearPinOutput(kGpioLED2);
        }   
      }         
              
      for(u8Increase=0;u8Increase<64;u8Increase++)           
      {      
        u8ReceiveData[u8Increase]=0; 
      }
       /*reset transmitter and receiver status, start next round communication*/
      u8ReadCommand=1;
      u8SendCommand=1;
      u8ReadReady=0;      
      u8TXIndex=0;
      u8RXIndex=0;
      u32Cycles=0;
      u8RX_Over=0;
      u8TX_Over=0;
      //GPIO_DRV_SetPinOutput(kGpioLED1);
      //GPIO_DRV_SetPinOutput(kGpioLED2);
      //GPIO_DRV_SetPinOutput(kGpioLED3);
      //GPIO_DRV_SetPinOutput(kGpioLED4);
      /*open UART TX and RX interrupt again*/
      LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntTxDataRegEmpty, true);  
      LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntRxDataRegFull, true);    
    }    
  }
}
/*FUNCTION*********************************************************************
 *
 * Function Name : TPM_Handler
 * Description   : TPM interrupt service routine
 *    
 *
 *END*************************************************************************/
void TPM1_IRQHandler(void)
{ 
  if(TPM_HAL_GetTimerOverflowStatus(TPM1_BASE))
  { 
    //TPM_HAL_ClearTimerOverflowFlag(TPM1_BASE);
    TPM1->SC |= TPM_SC_TOF_MASK;
  }
  
  /* send 64 bytes data per second*/
  //if(u32Cycles<38000) 
  if(u32Cycles<79000) 
  { 
    u32Cycles++;
  }   
  else
  {
    u32Cycles=0;  
    u8ReadReady=1;
  }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : LPUART0_IRQHandler
 * Description   : LPUART0 interrupt service routine
 *    
 *
 *END*************************************************************************/
void LPUART0_IRQHandler(void)
{ 
  if(u8SendCommand)
  { 
    if(LPUART_HAL_IsTxDataRegEmpty(LPUART0_BASE))
    {        
      if(u8TXIndex==63)
      {
        LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntTxDataRegEmpty, false);
        LPUART_HAL_Putchar(LPUART0_BASE, u8SendData[u8TXIndex++]);
        u8TXIndex =0;
        u8TX_Over =1;
        u8SendCommand=0;
      }      
      else    
      { 
        LPUART_HAL_Putchar(LPUART0_BASE, u8SendData[u8TXIndex++]);
      }
    } 
  }  
  
  if(u8ReadCommand)
  {      
    if(LPUART_HAL_IsRxDataRegFull(LPUART0_BASE))  
    {              
      if(u8RXIndex==63) 
      {      
        LPUART_HAL_SetIntMode(LPUART0_BASE, kLpuartIntRxDataRegFull, false);
        LPUART_HAL_Getchar(LPUART0_BASE, &u8ReceiveData[u8RXIndex++]);
        u8RXIndex =0;
        u8RX_Over =1;
        u8ReadCommand=0; 
      }
      else
      {
        LPUART_HAL_Getchar(LPUART0_BASE, &u8ReceiveData[u8RXIndex++]);
      }  
    }
  }
      
  /* Handle receive overrun interrupt */
  if (LPUART_HAL_GetStatusFlag(LPUART0_BASE, kLpuartRxOverrun))
  {
    /* Clear the flag, OR the rxDataRegFull will not be set any more */
    LPUART_HAL_ClearStatusFlag(LPUART0_BASE, kLpuartRxOverrun);
  }
  
  /* Handle receive overrun interrupt */
  if (LPUART_HAL_GetStatusFlag(LPUART0_BASE, kLpuartNoiseDetect))
  {
    /* Clear the flag, OR the rxDataRegFull will not be set any more */
    LPUART_HAL_ClearStatusFlag(LPUART0_BASE, kLpuartNoiseDetect);
  }
  
  /* Handle receive overrun interrupt */
  if (LPUART_HAL_GetStatusFlag(LPUART0_BASE, kLpuartFrameErr))
  {
    /* Clear the flag, OR the rxDataRegFull will not be set any more */
    LPUART_HAL_ClearStatusFlag(LPUART0_BASE, kLpuartFrameErr);
  }
}

/********************************************************************/
