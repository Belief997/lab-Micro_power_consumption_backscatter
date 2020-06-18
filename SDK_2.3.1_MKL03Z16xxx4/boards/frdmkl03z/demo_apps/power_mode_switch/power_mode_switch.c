/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "fsl_common.h"
#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_rcm.h"
#include "fsl_lptmr.h"
#include "fsl_port.h"
#include "power_mode_switch.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "fsl_lpuart.h"
#include "fsl_pmc.h"

#include "fsl_tpm.h"











/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_DEBUG_UART_BAUDRATE 9600 /* Debug console baud rate. */

/* Default debug console clock source. */
#define APP_DEBUG_UART_DEFAULT_CLKSRC_NAME kCLOCK_McgPeriphClk /* MCGPCLK */
#define APP_DEBUG_UART_DEFAULT_CLKSRC 0x01                     /* MCGPCLK */

/* Debug console clock source in VLPR mode. */
#define APP_DEBUG_UART_VLPR_CLKSRC_NAME kCLOCK_McgInternalRefClk /* MCGIRCLK */
#define APP_DEBUG_UART_VLPR_CLKSRC 0x03                          /* MCGIRCCLK */

#define LLWU_LPTMR_IDX 0U      /* LLWU_M0IF */
#define LLWU_WAKEUP_PIN_IDX 4U /* LLWU_P4 */
//#define LLWU_WAKEUP_PIN_TYPE kLLWU_ExternalPinFallingEdge
#define LLWU_WAKEUP_PIN_TYPE kLLWU_ExternalPinRisingEdge

#define APP_WAKEUP_BUTTON_GPIO BOARD_SW2_GPIO
#define APP_WAKEUP_BUTTON_PORT BOARD_SW2_PORT
#define APP_WAKEUP_BUTTON_GPIO_PIN BOARD_SW2_GPIO_PIN
#define APP_WAKEUP_BUTTON_IRQ BOARD_SW2_IRQ
#define APP_WAKEUP_BUTTON_IRQ_HANDLER BOARD_SW2_IRQ_HANDLER
#define APP_WAKEUP_BUTTON_NAME BOARD_SW2_NAME
//#define APP_WAKEUP_BUTTON_IRQ_TYPE kPORT_InterruptFallingEdge
#define APP_WAKEUP_BUTTON_IRQ_TYPE kPORT_InterruptRisingEdge

/* Debug console RX pin: PORTB1 MUX: 2 */
#define DEBUG_CONSOLE_RX_PORT PORTB
#define DEBUG_CONSOLE_RX_GPIO GPIOB
#define DEBUG_CONSOLE_RX_PIN 1
#define DEBUG_CONSOLE_RX_PINMUX kPORT_MuxAlt2
/* Debug console TX pin: PORTB2 MUX: 2 */
#define DEBUG_CONSOLE_TX_PORT PORTB
#define DEBUG_CONSOLE_TX_GPIO GPIOB
#define DEBUG_CONSOLE_TX_PIN 2
#define DEBUG_CONSOLE_TX_PINMUX kPORT_MuxAlt2
#define CORE_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void APP_PowerPreSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);
void APP_PowerPostSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);

/*
 * Set the clock configuration for RUN mode from VLPR mode.
 */
extern void APP_SetClockRunFromVlpr(void);

/*
 * Set the clock configuration for VLPR mode.
 */
extern void APP_SetClockVlpr(void);

/*
 * Hook function called before power mode switch.
 */
extern void APP_PowerPreSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);

/*
 * Hook function called after power mode switch.
 */
extern void APP_PowerPostSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_wakeupTimeout;            /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */



// pwm setting

/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool brightnessUp = true; /* Indicate LED is brighter or dimmer */
volatile uint8_t updatedDutycycle = 50U;
volatile uint8_t getCharValue = 0U;



void user_showFreqList()
{
    PRINTF("kCLOCK_CoreSysClk %d.\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("kCLOCK_PlatClk %d.\r\n", CLOCK_GetFreq(kCLOCK_PlatClk));
    PRINTF("kCLOCK_BusClk %d.\r\n", CLOCK_GetFreq(kCLOCK_BusClk));
    PRINTF("kCLOCK_FlashClk %d.\r\n", CLOCK_GetFreq(kCLOCK_FlashClk));
    PRINTF("kCLOCK_Er32kClk %d.\r\n", CLOCK_GetFreq(kCLOCK_Er32kClk));
    PRINTF("kCLOCK_Osc0ErClk %d.\r\n", CLOCK_GetFreq(kCLOCK_Osc0ErClk));
    PRINTF("kCLOCK_McgInternalRefClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgInternalRefClk));
    PRINTF("kCLOCK_McgPeriphClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgPeriphClk));
    PRINTF("kCLOCK_McgIrc48MClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgIrc48MClk));
    PRINTF("kCLOCK_LpoClk %d.\r\n", CLOCK_GetFreq(kCLOCK_LpoClk));


}
















/*******************************************************************************
 * Code
 ******************************************************************************/
extern void APP_SetClockRunFromVlpr(void);

static void APP_InitDefaultDebugConsole(void)
{
    uint32_t uartDefaultClkSrcFreq;
    CLOCK_SetLpuart0Clock(APP_DEBUG_UART_DEFAULT_CLKSRC);
    uartDefaultClkSrcFreq = CLOCK_GetFreq(APP_DEBUG_UART_DEFAULT_CLKSRC_NAME);
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, APP_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartDefaultClkSrcFreq);
    CLOCK_EnableClock(kCLOCK_Rtc0);
    /* Enable the RTC 32KHz oscillator */
    RTC->IER = 0x00000000;
    CLOCK_DisableClock(kCLOCK_Rtc0);
}

static void APP_InitVlprDebugConsole(void)
{
    uint32_t uartVlprClkSrcFreq;
    CLOCK_SetLpuart0Clock(APP_DEBUG_UART_VLPR_CLKSRC);
    uartVlprClkSrcFreq = CLOCK_GetFreq(APP_DEBUG_UART_VLPR_CLKSRC_NAME);
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, APP_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartVlprClkSrcFreq);
}


void APP_PowerPreSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    /* Wait for debug console output finished. */
    while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
    {
    }
    DbgConsole_Deinit();

    if ((kAPP_PowerModeRun != targetMode) && (kAPP_PowerModeVlpr != targetMode))
    {
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to disable.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, kPORT_PinDisabledOrAnalog);
    }
}

void APP_PowerPostSwitchHook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    smc_power_state_t powerState = SMC_GetPowerModeState(SMC);

    /*
     * For some other platforms, if enter LLS mode from VLPR mode, when wakeup, the
     * power mode is VLPR. But for some platforms, if enter LLS mode from VLPR mode,
     * when wakeup, the power mode is RUN. In this case, the clock setting is still
     * VLPR mode setting, so change to RUN mode setting here.
     */
    if ((kSMC_PowerStateVlpr == originPowerState) && (kSMC_PowerStateRun == powerState))
    {
        APP_SetClockRunFromVlpr();
    }

    if ((kAPP_PowerModeRun != targetMode) && (kAPP_PowerModeVlpr != targetMode))
    {
        /*
         * Debug console RX pin is set to disable for current leakage, nee to re-configure pinmux.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, DEBUG_CONSOLE_RX_PINMUX);
    }

    /* Set debug console clock source. */
    if (kSMC_PowerStateVlpr == powerState)
    {
        APP_InitVlprDebugConsole();
    }
    else
    {
        APP_InitDefaultDebugConsole();
    }
}

/*!
 * @brief LLWU interrupt handler.
 */
void LLWU_IRQHandler(void)
{
    /* If wakeup by external pin. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, LLWU_WAKEUP_PIN_IDX))
    {
        PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT, (1U << APP_WAKEUP_BUTTON_GPIO_PIN));
        LLWU_ClearExternalWakeupPinFlag(LLWU, LLWU_WAKEUP_PIN_IDX);
    }
}

void LPTMR0_IRQHandler(void)
{
    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR0))
    {
        LPTMR_DisableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
        LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
        LPTMR_StopTimer(LPTMR0);
    }
}

void APP_WAKEUP_BUTTON_IRQ_HANDLER(void)
{
    if ((1U << APP_WAKEUP_BUTTON_GPIO_PIN) & PORT_GetPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT))
    {
        /* Disable interrupt. */
        PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_PORT, (1U << APP_WAKEUP_BUTTON_GPIO_PIN));
    }
}

/*!
 * @brief Get input from user about wakeup timeout
 */
static uint8_t APP_GetWakeupTimeout(void)
{
    uint8_t timeout;

    while (1)
    {
        PRINTF("Select the wake up timeout in seconds.\r\n");
        PRINTF("The allowed range is 1s ~ 9s.\r\n");
        PRINTF("Eg. enter 5 to wake up in 5 seconds.\r\n");
        PRINTF("\r\nWaiting for input timeout value...\r\n\r\n");

        timeout = GETCHAR();
        PRINTF("%c\r\n", timeout);
        if ((timeout > '0') && (timeout <= '9'))
        {
            return timeout - '0';
        }
        PRINTF("Wrong value!\r\n");
    }
}

/* Get wakeup source by user input. */
static app_wakeup_source_t APP_GetWakeupSource(void)
{
    uint8_t ch;

    while (1)
    {
        PRINTF("Select the wake up source:\r\n");
        PRINTF("Press T for LPTMR - Low Power Timer\r\n");
        PRINTF("Press S for switch/button %s. \r\n", APP_WAKEUP_BUTTON_NAME);

        PRINTF("\r\nWaiting for key press..\r\n\r\n");

        ch = GETCHAR();

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }

        if (ch == 'T')
        {
            return kAPP_WakeupSourceLptmr;
        }
        else if (ch == 'S')
        {
            return kAPP_WakeupSourcePin;
        }
        else
        {
            PRINTF("Wrong value!\r\n");
        }
    }
}

/* Get wakeup timeout and wakeup source. */
void APP_GetWakeupConfig(app_power_mode_t targetMode)
{
    /* Get wakeup source by user input. */
    if ((targetMode == kAPP_PowerModeVlls0) || (targetMode == kAPP_PowerModeVlls1) ||
        (targetMode == kAPP_PowerModeVlls3))
    {
        /* In VLLS0 mode, the LPO is disabled, LPTMR could not work. */
        PRINTF("Not support LPTMR wakeup because LPO is disabled in VLLS0 mode.\r\n");
        s_wakeupSource = kAPP_WakeupSourcePin;
    }
    else
    {
        /* Get wakeup source by user input. */
        s_wakeupSource = APP_GetWakeupSource();
    }

    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        /* Wakeup source is LPTMR, user should input wakeup timeout value. */
        s_wakeupTimeout = APP_GetWakeupTimeout();
        PRINTF("Will wakeup in %d seconds.\r\n", s_wakeupTimeout);
    }
    else
    {
        PRINTF("Press %s to wake up.\r\n", APP_WAKEUP_BUTTON_NAME);
    }
}

void APP_SetWakeupConfig(app_power_mode_t targetMode)
{
    /* Set LPTMR timeout value. */
    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        LPTMR_SetTimerPeriod(LPTMR0, (1000U * s_wakeupTimeout) - 1U);
        LPTMR_StartTimer(LPTMR0);
    }

    /* Set the wakeup module. */
    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
    }
    else
    {
        PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, APP_WAKEUP_BUTTON_IRQ_TYPE);
    }

    /* If targetMode is VLLS/LLS, setup LLWU. */
    if ((kAPP_PowerModeWait != targetMode) && (kAPP_PowerModeVlpw != targetMode) &&
        (kAPP_PowerModeVlps != targetMode) && (kAPP_PowerModeStop != targetMode))
    {
        LLWU_SetExternalWakeupPinMode(LLWU, LLWU_WAKEUP_PIN_IDX, LLWU_WAKEUP_PIN_TYPE);
        NVIC_EnableIRQ(LLWU_IRQn);
    }
}

void APP_ShowPowerMode(smc_power_state_t powerMode)
{
    switch (powerMode)
    {
        case kSMC_PowerStateRun:
            PRINTF("    Power mode: RUN\r\n");
            break;
        case kSMC_PowerStateVlpr:
            PRINTF("    Power mode: VLPR\r\n");
            break;
        default:
            PRINTF("    Power mode wrong\r\n");
            break;
    }
}

/*
 * Check whether could switch to target power mode from current mode.
 * Return true if could switch, return false if could not switch.
 */
bool APP_CheckPowerMode(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    bool modeValid = true;

    /*
     * Check wether the mode change is allowed.
     *
     * 1. If current mode is HSRUN mode, the target mode must be RUN mode.
     * 2. If current mode is RUN mode, the target mode must not be VLPW mode.
     * 3. If current mode is VLPR mode, the target mode must not be HSRUN/WAIT/STOP mode.
     * 4. If already in the target mode.
     */
    switch (curPowerState)
    {
        case kSMC_PowerStateRun:
            if (kAPP_PowerModeVlpw == targetPowerMode)
            {
                PRINTF("Could not enter VLPW mode from RUN mode.\r\n");
                modeValid = false;
            }
            break;

        case kSMC_PowerStateVlpr:
            if ((kAPP_PowerModeWait == targetPowerMode) || (kAPP_PowerModeStop == targetPowerMode))
            {
                PRINTF("Could not enter HSRUN/STOP/WAIT modes from VLPR mode.\r\n");
                modeValid = false;
            }
            break;
        default:
            PRINTF("Wrong power state.\r\n");
            modeValid = false;
            break;
    }

    if (!modeValid)
    {
        return false;
    }

    /* Don't need to change power mode if current mode is already the target mode. */
    if (((kAPP_PowerModeRun == targetPowerMode) && (kSMC_PowerStateRun == curPowerState)) ||
        ((kAPP_PowerModeVlpr == targetPowerMode) && (kSMC_PowerStateVlpr == curPowerState)))
    {
        PRINTF("Already in the target power mode.\r\n");
        return false;
    }

    return true;
}

/*
 * Power mode switch.
 */
void APP_PowerModeSwitch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    smc_power_mode_vlls_config_t vlls_config;
    vlls_config.enablePorDetectInVlls0 = true;
    vlls_config.enableLpoClock = true; /*!< Enable LPO clock in VLLS mode */

    switch (targetPowerMode)
    {
        case kAPP_PowerModeVlpr:
            APP_SetClockVlpr();
            SMC_SetPowerModeVlpr(SMC);
            while (kSMC_PowerStateVlpr != SMC_GetPowerModeState(SMC))
            {
            }
            break;

        case kAPP_PowerModeRun:

            /* Power mode change. */
            SMC_SetPowerModeRun(SMC);
            while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC))
            {
            }

            /* If enter RUN from VLPR, change clock after the power mode change. */
            if (kSMC_PowerStateVlpr == curPowerState)
            {
                APP_SetClockRunFromVlpr();
            }
            break;

//        case kAPP_PowerModeWait:
//            SMC_PreEnterWaitModes();
//            SMC_SetPowerModeWait(SMC);
//            SMC_PostExitWaitModes();
//            break;

//        case kAPP_PowerModeStop:
//            SMC_PreEnterStopModes();
//            SMC_SetPowerModeStop(SMC, kSMC_PartialStop);
//            SMC_PostExitStopModes();
//            break;

//        case kAPP_PowerModeVlpw:
//            SMC_PreEnterWaitModes();
//            SMC_SetPowerModeVlpw(SMC);
//            SMC_PostExitWaitModes();
//            break;

//        case kAPP_PowerModeVlps:
//            SMC_PreEnterStopModes();
//            SMC_SetPowerModeVlps(SMC);
//            SMC_PostExitStopModes();
//            break;

//        case kAPP_PowerModeVlls0:
//            vlls_config.subMode = kSMC_StopSub0;
//            SMC_PreEnterStopModes();
//            SMC_SetPowerModeVlls(SMC, &vlls_config);
//            SMC_PostExitStopModes();
//            break;

//        case kAPP_PowerModeVlls1:
//            vlls_config.subMode = kSMC_StopSub1;
//            SMC_PreEnterStopModes();
//            SMC_SetPowerModeVlls(SMC, &vlls_config);
//            SMC_PostExitStopModes();
//            break;

        case kAPP_PowerModeVlls3:
            vlls_config.subMode = kSMC_StopSub3;
            SMC_PreEnterStopModes();
            SMC_SetPowerModeVlls(SMC, &vlls_config);

            SMC_PostExitStopModes();
            break;

        default:
            PRINTF("Wrong value");
            break;
    }
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 90000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

#define USER_PWM_NUM  2
/*!
 * @brief main demo function.
 */
int main(void)
{
    uint32_t freq = 0;
    uint8_t ch;
    smc_power_state_t curPowerState;
    app_power_mode_t targetPowerMode;
    bool needSetWakeup; /* Need to set wakeup. */
    lptmr_config_t lptmrConfig;

    /* Power related. */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PMC_ClearPeriphIOIsolationFlag(PMC);
        NVIC_ClearPendingIRQ(LLWU_IRQn);
    }

    /*******************************************************************************
     *
     *  pwm init
     *
     *  *******************************************************************************/
    // tpm config init
#ifndef TPM_LED_ON_LEVEL
#define TPM_LED_ON_LEVEL kTPM_LowTrue
#endif
#if USER_PWM_NUM == 1
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam;



    /* Configure tpm params with frequency 24kHZ */
    tpmParam.chnlNumber = (tpm_chnl_t)BOARD_TPM_CHANNEL;
    tpmParam.level = TPM_LED_ON_LEVEL;
    tpmParam.dutyCyclePercent = updatedDutycycle;
    
#elif USER_PWM_NUM == 2
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

#ifndef TPM_LED_ON_LEVEL  
      #define TPM_LED_ON_LEVEL kTPM_LowTrue
#endif    
#define BOARD_FIRST_TPM_CHANNEL 0U
#define BOARD_SECOND_TPM_CHANNEL 1U

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL;
    tpmParam[0].level = TPM_LED_ON_LEVEL;
    tpmParam[0].dutyCyclePercent = updatedDutycycle;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL;
    tpmParam[1].level = TPM_LED_ON_LEVEL;
    tpmParam[1].dutyCyclePercent = updatedDutycycle;

#else
    #error "pwm num set err!"
#endif

/******************************************************************************/

    /* Define the init structure for the output ENABLE pin*/
    gpio_pin_config_t enable_config = {
        kGPIO_DigitalOutput, 0,
    };
    BOARD_InitPins();
    /* Init output ENABLE GPIO. */
    GPIO_PinInit(GPIOA, 7U, &enable_config);
//    GPIO_PinInit(GPIOB, 3U, &enable_config);

    // api to write gpio value
//    GPIO_WritePinOutput(GPIOA, 7U, 1);


    BOARD_BootClockRUN();
    APP_InitDefaultDebugConsole();

    /* Setup LPTMR. */
//    LPTMR_GetDefaultConfig(&lptmrConfig);
//    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1; /* Use LPO as clock source. */
//    lptmrConfig.bypassPrescaler = true;
//
//    LPTMR_Init(LPTMR0, &lptmrConfig);

    NVIC_EnableIRQ(LLWU_IRQn);
//    NVIC_EnableIRQ(LPTMR0_IRQn);

    NVIC_EnableIRQ(APP_WAKEUP_BUTTON_IRQ);

    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PRINTF("\r\nMCU wakeup from VLLS modes...\r\n");
    }


//    user_showFreqList();

    // mode now is run 48 MHz


    // update duty
//    updatedDutycycle = 5 * 10U;
//
//    /* Disable channel output before updating the dutycycle */
//    TPM_UpdateChnlEdgeLevelSelect(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, 0U);
//
//    /* Update PWM duty cycle */
//    TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, kTPM_CenterAlignedPwm,
//                           updatedDutycycle);
//
//    /* Start channel output with updated dutycycle */
//    TPM_UpdateChnlEdgeLevelSelect(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, TPM_LED_ON_LEVEL);


//    while (0)
    {
        curPowerState = SMC_GetPowerModeState(SMC);

        freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);

        PRINTF("\r\n####################  Power Mode Switch Demo ####################\n\r\n");
        PRINTF("    Core Clock = %dHz \r\n", freq);

        APP_ShowPowerMode(curPowerState);

//        PRINTF("\r\nSelect the desired operation \n\r\n");
//        PRINTF("Press  %c for enter: RUN      - Normal RUN mode\r\n", kAPP_PowerModeRun);
//        PRINTF("Press  %c for enter: WAIT     - Wait mode\r\n", kAPP_PowerModeWait);
//        PRINTF("Press  %c for enter: STOP     - Stop mode\r\n", kAPP_PowerModeStop);
//        PRINTF("Press  %c for enter: VLPR     - Very Low Power Run mode\r\n", kAPP_PowerModeVlpr);
//        PRINTF("Press  %c for enter: VLPW     - Very Low Power Wait mode\r\n", kAPP_PowerModeVlpw);
//        PRINTF("Press  %c for enter: VLPS     - Very Low Power Stop mode\r\n", kAPP_PowerModeVlps);
//        PRINTF("Press  %c for enter: VLLS0    - Very Low Leakage Stop 0 mode\r\n", kAPP_PowerModeVlls0);
//        PRINTF("Press  %c for enter: VLLS1    - Very Low Leakage Stop 1 mode\r\n", kAPP_PowerModeVlls1);
//        PRINTF("Press  %c for enter: VLLS3    - Very Low Leakage Stop 3 mode\r\n", kAPP_PowerModeVlls3);
//
//        PRINTF("1\r\n");
//        PRINTF("\r\nWaiting for power mode select..\r\n\r\n");

        /* Wait for user response */

//        ch = GETCHAR();
//
//        if ((ch >= 'a') && (ch <= 'z'))
//        {
//            ch -= 'a' - 'A';
//        }

//        targetPowerMode = (app_power_mode_t)ch;

        // set default powermode as vlpr

        if(curPowerState != kSMC_PowerStateVlpr)
        {
			targetPowerMode = (app_power_mode_t)kAPP_PowerModeVlpr;
        }
        else
        {
        	ch = GETCHAR();
        	if(ch == '3')
        	{
        		targetPowerMode = (app_power_mode_t)kAPP_PowerModeVlls3;
        	}
        	else
        	{
        		targetPowerMode = (app_power_mode_t)kAPP_PowerModeVlpr;
        	}
        }
        // A:run, D:vlpr, I:vlls3
        PRINTF("\r\nTarget %c \r\n", targetPowerMode);

		if ((targetPowerMode > kAPP_PowerModeMin) && (targetPowerMode < kAPP_PowerModeMax))
		{
			/* If could not set the target power mode, loop continue. */
			if (!APP_CheckPowerMode(curPowerState, targetPowerMode))
			{
//				continue;
//				break;
			}

			/* If target mode is RUN/VLPR/HSRUN, don't need to set wakeup source. */
			if ((kAPP_PowerModeRun == targetPowerMode) || (kAPP_PowerModeVlpr == targetPowerMode))
			{
				needSetWakeup = false;
			}

			else
			{
				needSetWakeup = true;
			}

			if (needSetWakeup)
			{
				APP_GetWakeupConfig(targetPowerMode);
			}

			APP_PowerPreSwitchHook(curPowerState, targetPowerMode);

			if (needSetWakeup)
			{
				APP_SetWakeupConfig(targetPowerMode);
			}

			APP_PowerModeSwitch(curPowerState, targetPowerMode);
			APP_PowerPostSwitchHook(curPowerState, targetPowerMode);

//			PRINTF("\r\nNext loop\r\n");
		}

    };

    // show state
//    {
//        curPowerState = SMC_GetPowerModeState(SMC);

//        freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);

//        PRINTF("\r\n####################  Power Mode Switch Demo ####################\n\r\n");
//        PRINTF("    Core Clock = %dHz \r\n", freq);

//        APP_ShowPowerMode(curPowerState);

//    }

//    user_showFreqList();

    // init and run pwm here
    /* Select the clock source for the TPM counter as kCLOCK_McgInternalRefClk */
//    CLOCK_SetTpmClock(1U);
    CLOCK_SetTpmClock(3U);

    TPM_GetDefaultConfig(&tpmInfo);
    /* Initialize TPM module */
    TPM_Init(BOARD_TPM_BASEADDR, &tpmInfo);

//    TPM_SetupPwm(BOARD_TPM_BASEADDR, &tpmParam, 1U, kTPM_CenterAlignedPwm, 1000000U, TPM_SOURCE_CLOCK);
//    TPM_SetupPwm(BOARD_TPM_BASEADDR, &tpmParam, 1U, kTPM_CenterAlignedPwm, 250000U, 2000000); // 2M
    TPM_SetupPwm(BOARD_TPM_BASEADDR, &tpmParam, USER_PWM_NUM, kTPM_CenterAlignedPwm, 250000U, 2000000); // 2M
    
    TPM_StartTimer(BOARD_TPM_BASEADDR, kTPM_SystemClock);

    uint8_t i_delay = 0;
    while(i_delay++ < 120)// about 120s
    {
        delay();
        PRINTF(". ");
    }

    {
        curPowerState = kSMC_PowerStateVlpr;
        targetPowerMode = (app_power_mode_t)kAPP_PowerModeVlls3;
        //needSetWakeup = true;
        APP_GetWakeupConfig(targetPowerMode);
        APP_PowerPreSwitchHook(curPowerState, targetPowerMode);
        APP_SetWakeupConfig(targetPowerMode);
        APP_PowerModeSwitch(curPowerState, targetPowerMode);
        APP_PowerPostSwitchHook(curPowerState, targetPowerMode);
    }

    return 0;
}
