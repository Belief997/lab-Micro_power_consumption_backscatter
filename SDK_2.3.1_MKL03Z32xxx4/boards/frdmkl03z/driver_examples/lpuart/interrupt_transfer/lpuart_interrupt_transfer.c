/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "board.h"
#include "fsl_lpuart.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "user_func.h"

#include "fsl_lptmr.h"
#include "fsl_gpio.h"


extern lpuart_handle_t g_lpuartHandle;

uint8_t g_tipString[] =
    "Lpuart interrupt example\r\nBoard receives 8 characters then sends them out\r\nNow please input:\r\n";

extern uint8_t g_txBuffer[ECHO_BUFFER_LENGTH];
extern uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH];
extern volatile bool rxBufferEmpty;
extern volatile bool txBufferFull ;
extern volatile bool txOnGoing ;
extern volatile bool rxOnGoing ;






/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
//#define LPTMR_USEC_COUNT 1000000U
#define LPTMR_USEC_COUNT 1000U

#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint32_t lptmrCounter = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/
void LPTMR_LED_HANDLER(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);
    lptmrCounter++;
    LED_TOGGLE();
    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}



/*!
 * @brief Main function
 */
int main(void)
{
    lpuart_config_t config;
    lpuart_transfer_t xfer;
    lpuart_transfer_t sendXfer;
    lpuart_transfer_t receiveXfer;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    CLOCK_SetLpuart0Clock(0x1U);

    dac_init();
//    dac_test();


// TIMER
    uint32_t currentCounter = 0U;
    lptmr_config_t lptmrConfig;

    LED_INIT();

    /* Board pin, clock, debug console init */
//    BOARD_InitPins();
//    BOARD_BootClockRUN();
//    BOARD_InitDebugConsole();

    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);
//    lptmrConfig¡£prescalerClockSource = kLPTMR_PrescalerClock_0;

    /* Initialize the LPTMR */
    LPTMR_Init(DEMO_LPTMR_BASE, &lptmrConfig);

    /*
     * Set timer period.
     * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
    */
    LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(DEMO_LPTMR_IRQn);

//    PRINTF("Low Power Timer Example\r\n");

    /* Start counting */
    LPTMR_StartTimer(DEMO_LPTMR_BASE);






//    /*
//     * config.baudRate_Bps = 115200U;
//     * config.parityMode = kLPUART_ParityDisabled;
//     * config.stopBitCount = kLPUART_OneStopBit;
//     * config.txFifoWatermark = 0;
//     * config.rxFifoWatermark = 0;
//     * config.enableTx = false;
//     * config.enableRx = false;
//     */
//    LPUART_GetDefaultConfig(&config);
//    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
//    config.enableTx = true;
//    config.enableRx = true;
//
//    LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);
//    LPUART_TransferCreateHandle(DEMO_LPUART, &g_lpuartHandle, LPUART_UserCallback, NULL);
//
//    /* Send g_tipString out. */
//    xfer.data = g_tipString;
//    xfer.dataSize = sizeof(g_tipString) - 1;
//    txOnGoing = true;
//    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);
//
//    /* Wait send finished */
//    while (txOnGoing)
//    {
//    }
//
//    /* Start to echo. */
//    sendXfer.data = g_txBuffer;
//    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
//    receiveXfer.data = g_rxBuffer;
//    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
//
//    while (1)
//    {
//        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
//        if ((!rxOnGoing) && rxBufferEmpty)
//        {
//            rxOnGoing = true;
//            LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, NULL);
//        }
//
//        /* If TX is idle and g_txBuffer is full, start to send data. */
//        if ((!txOnGoing) && txBufferFull)
//        {
//            txOnGoing = true;
//            LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
//        }
//
//        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
//        if ((!rxBufferEmpty) && (!txBufferFull))
//        {
//            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
//            rxBufferEmpty = true;
//            txBufferFull = true;
//        }
//    }
}
