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
#include "fsl_adc16.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "fsl_lpuart.h"
#include "fsl_pmc.h"

#include "fsl_tpm.h"

#include "user.h"


#ifndef ECHO_BUFFER_SIZE
#define ECHO_BUFFER_SIZE 1U
#endif





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


/*  ADC & LPTMR  */
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 8U
// tmp
//#define kAdcChannelTemperature (26U) /*! ADC channel of temperature sensor */
//#define kAdcChannelBandgap (27U)     /*! ADC channel of BANDGAP */

#define DEMO_ADC16_IRQ_ID ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define DEMO_LPTMR_BASE LPTMR0
#define DEMO_LPTMR_IRQn LPTMR0_IRQn
//#define LPTMR_LED_HANDLER LPTMR0_IRQHandler


/*
 * These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */
//#define ADCR_VDD (65535U) /* Maximum value when use 16b resolution */
//#define V_BG (1000U)      /* BANDGAP voltage in mV (trim to 1.0V) */
//#define V_TEMP25 (716U)   /* Typical VTEMP25 in mV */
//#define M (1620U)         /* Typical slope: (mV x 1000)/oC */
//#define STANDARD_TEMP (25U)

#define LED1_INIT() LED_RED_INIT(LOGIC_LED_OFF)
#define LED1_ON() LED_RED_ON()
#define LED1_OFF() LED_RED_OFF()

//#define UPDATE_BOUNDARIES_TIME                                                         \
//    (20U) /*! This value indicates the number of cycles needed to update boundaries. \ \
//              To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/

//#define LPTMR_COMPARE_VALUE (500U) /* Low Power Timer interrupt time in miliseconds */
//#define LPTMR_COMPARE_VALUE (500U) // 8k square  @8M clk
//#define LPTMR_COMPARE_VALUE (2000U) // 500 square  @2M clk
//#define LPTMR_COMPARE_VALUE (8000U) // 1ms  @2M clk

#define LPTMR_COMPARE_VALUE (2000U) // 1ms  @2M clk

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


/*  ADC & LPTMR  */
void BOARD_ConfigTriggerSource(void);
/*!
 * @brief ADC stop conversion
 *
 * @param base The ADC instance number
 */
//static void ADC16_PauseConversion(ADC_Type *base);

/*!
 * @brief calibrate parameters: VDD and ADCR_TEMP25
 *
 * @param base The ADC instance number
 */
static void ADC16_CalibrateParams(ADC_Type *base);

/*!
 * @brief User-defined function to init trigger source  of LPTimer
 *
 * @param base The LPTMR instance number
 */
static void LPTMR_InitTriggerSourceOfAdc(LPTMR_Type *base);

/*!
 * @brief Initialize the ADCx for HW trigger.
 *
 * @param base The ADC instance number
 *
 * @return true if success
 */
static bool ADC16_InitHardwareTrigger(ADC_Type *base);


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


// adc & lptmr
volatile static uint32_t adcValue = 0; /*! ADC value */
//static uint32_t adcrTemp25 = 0;        /*! Calibrated ADCR_TEMP25 */
//static uint32_t adcr100m = 0;

volatile bool conversionCompleted = false; /*! Conversion is completed Flag */



  // user defined func
void user_showFreqList(void)
{
	PRINTF("\r\n--------------------------\r\n");
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

//void LPTMR0_IRQHandler(void)
//{
//    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR0))
//    {
//        LPTMR_DisableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
//        LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
//        LPTMR_StopTimer(LPTMR0);
//    }
//}

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
//        PRINTF("Not support LPTMR wakeup because LPO is disabled in VLLS0 mode.\r\n");
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
//        PRINTF("Press %s to wake up.\r\n", APP_WAKEUP_BUTTON_NAME);
    	PRINTF("Pull up PTB0 to wake up.\r\n");
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

void BOARD_ConfigTriggerSource(void)
{
    /* Configure SIM for ADC hw trigger source selection */
    SIM->SOPT7 |= 0x0000008EU;
}
/* Enable the trigger source of LPTimer */
static void LPTMR_InitTriggerSourceOfAdc(LPTMR_Type *base)
{
    lptmr_config_t lptmrUserConfig;

    /*
     * lptmrUserConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrUserConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrUserConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrUserConfig.enableFreeRunning = false;
     * lptmrUserConfig.bypassPrescaler = true;
     * lptmrUserConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrUserConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrUserConfig);
    /* Init LPTimer driver */
    LPTMR_Init(base, &lptmrUserConfig);

    /* Set the LPTimer period */
    LPTMR_SetTimerPeriod(base, LPTMR_COMPARE_VALUE);

    /* Start the LPTimer */
//    LPTMR_StartTimer(base);

    /* Configure SIM for ADC hw trigger source selection */
//    BOARD_ConfigTriggerSource();
}
/*!
 * @brief ADC stop conversion
 */
//static void ADC16_PauseConversion(ADC_Type *base)
//{
//    adc16_channel_config_t adcChnConfig;

//    adcChnConfig.channelNumber = 31U; /*!< AD31 channel */
//    adcChnConfig.enableInterruptOnConversionCompleted = false;
//#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
//    adcChnConfig.enableDifferentialConversion = false;
//#endif
//    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);
//}

/*!
 * @brief calibrate parameters: VDD and ADCR_TEMP25
 */
static void ADC16_CalibrateParams(ADC_Type *base)
{
    uint32_t bandgapValue = 0; /*! ADC value of BANDGAP */
    uint32_t vdd = 0;          /*! VDD in mV */

    adc16_config_t adcUserConfig;
    adc16_channel_config_t adcChnConfig;
    pmc_bandgap_buffer_config_t pmcBandgapConfig;

    pmcBandgapConfig.enable = true;

#if (defined(FSL_FEATURE_PMC_HAS_BGEN) && FSL_FEATURE_PMC_HAS_BGEN)
    pmcBandgapConfig.enableInLowPowerMode = false;
#endif
#if (defined(FSL_FEATURE_PMC_HAS_BGBDS) && FSL_FEATURE_PMC_HAS_BGBDS)
    pmcBandgapConfig.drive = kPmcBandgapBufferDriveLow;
#endif
    /* Enable BANDGAP reference voltage */
    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);

    /*
    * Initialization ADC for
    * 16bit resolution, interrupt mode, hw trigger disabled.
    * normal convert speed, VREFH/L as reference,
    * disable continuous convert mode
    */
    /*
     * adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adcUserConfig.enableAsynchronousClock = true;
     * adcUserConfig.clockDivider = kADC16_ClockDivider8;
     * adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
     * adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
     * adcUserConfig.enableHighSpeed = false;
     * adcUserConfig.enableLowPower = false;
     * adcUserConfig.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adcUserConfig);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    adcUserConfig.resolution = kADC16_Resolution16Bit;
#else
    adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
#endif
    adcUserConfig.enableContinuousConversion = false;
    adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
    adcUserConfig.enableLowPower = 1;
    adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
#ifdef BOARD_ADC_USE_ALT_VREF
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(base, &adcUserConfig);

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    if (kStatus_Success == ADC16_DoAutoCalibration(base))
    {
//        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
        PRINTF("ADC16_DoAutoCalibration Done.\r\n");
    }
    else
    {
//        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
        PRINTF("ADC16_DoAutoCalibration Failed.\r\n");
    }
#endif

#if defined(FSL_FEATURE_ADC16_HAS_HW_AVERAGE) && FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    /* Use hardware average to increase stability of the measurement  */
    ADC16_SetHardwareAverage(base, kADC16_HardwareAverageCount32);
#endif /* FSL_FEATURE_ADC16_HAS_HW_AVERAGE */

//    adcChnConfig.channelNumber = kAdcChannelBandgap;
//#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
//    adcChnConfig.enableDifferentialConversion = false;
//#endif
//    adcChnConfig.enableInterruptOnConversionCompleted = false;
//    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);

//    /* Wait for the conversion to be done */
//    while (!ADC16_GetChannelStatusFlags(base, DEMO_ADC16_CHANNEL_GROUP))
//    {
//    }

//    /* Get current ADC BANDGAP value */
//    bandgapValue = ADC16_GetChannelConversionValue(base, DEMO_ADC16_CHANNEL_GROUP);

//    ADC16_PauseConversion(base);

//    /* Get VDD value measured in mV: VDD = (ADCR_VDD x V_BG) / ADCR_BG */
//    vdd = ADCR_VDD * V_BG / bandgapValue;
//    /* Calibrate ADCR_TEMP25: ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD */
//    adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
//    /* ADCR_100M = ADCR_VDD x M x 100 / VDD */
//    adcr100m = (ADCR_VDD * M) / (vdd * 10);

//    /* Disable BANDGAP reference voltage */
//    pmcBandgapConfig.enable = false;
//    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);
}

/*!
 * @brief Initialize the ADCx for Hardware trigger.
 */
static bool ADC16_InitHardwareTrigger(ADC_Type *base)
{
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    uint16_t offsetValue = 0; /*!< Offset error from correction value. */
#endif
    adc16_config_t adcUserConfig;
    adc16_channel_config_t adcChnConfig;

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    if (kStatus_Success != ADC16_DoAutoCalibration(base))
    {
        return false;
    }
    offsetValue = base->OFS;
    ADC16_SetOffsetValue(base, offsetValue);
#endif
    /*
    * Initialization ADC for
    * 12bit resolution, interrupt mode, hw trigger enabled.
    * normal convert speed, VREFH/L as reference,
    * disable continuous convert mode.
    */
    /*
     * adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adcUserConfig.enableAsynchronousClock = true;
     * adcUserConfig.clockDivider = kADC16_ClockDivider8;
     * adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
     * adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
     * adcUserConfig.enableHighSpeed = false;
     * adcUserConfig.enableLowPower = false;
     * adcUserConfig.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adcUserConfig);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    adcUserConfig.resolution = kADC16_Resolution16Bit;
#else
    adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
#endif
    /* enabled hardware trigger  */
    ADC16_EnableHardwareTrigger(base, true);
    adcUserConfig.enableContinuousConversion = false;
    adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;

//    adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
    adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
    adcUserConfig.enableLowPower = 1;
#if ((defined BOARD_ADC_USE_ALT_VREF) && BOARD_ADC_USE_ALT_VREF)
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(base, &adcUserConfig);

//    adcChnConfig.channelNumber = kAdcChannelTemperature;
    adcChnConfig.channelNumber = DEMO_ADC16_USER_CHANNEL;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    adcChnConfig.enableInterruptOnConversionCompleted = true;
    /* Configure channel 0 */
    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);
    return true;
}


void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    /* Get current ADC value */
    adcValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);

    {
    	ADC_DATA adc_data = {0};
    	adc_data.adcValue = adcValue;
    	data_enqueueadc(&adc_data);

    }

//    GPIO_PortToggle(GPIOA, 1u << 5U);
//    GPIO_PortToggle(GPIOB, 1u << 3U);
    /* Set conversionCompleted flag. This prevents an wrong conversion in main function */
    conversionCompleted = true;
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}


/*!
 * @brief main function
 */

#define DEMO_LPTMR_IRQn LPTMR0_IRQn
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
volatile u8 dataBuf[SENSOR_DATA_LEN];

volatile u8 status = STATUS_WAIT_TRIG;
volatile u8 status_rec = REC_WAIT;
volatile u8 status_send = SEND_WAIT;

void LPTMR_LED_HANDLER(void)
//void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(DEMO_LPTMR_BASE, kLPTMR_TimerCompareFlag);

    // debug
//    GPIO_PortToggle(GPIOB, 1u << 3U);
//    char debug_buf[6]={0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
    if(STATUS_WAIT_SEND == status)
    {
    	static u8 cnt_bit = 0;
    	static u8 cnt_byte = 0;
    	static u8 checkSum = 0;
    	static u8 islastBit = FALSE;

    	do{
			if(islastBit)
			{
				islastBit = FALSE;
				GPIO_PinWrite(GPIOB, 3U, 0);
				status_send = SEND_SUCCESS;
				break;
			}

			if(cnt_byte < SENSOR_HEADER_LEN)
			{
				GPIO_PinWrite(GPIOB, 3U, SENSOR_HEADER & (0x80 >> cnt_bit));
				checkSum = 0;
			}
			else if(cnt_byte < SENSOR_HEADER_LEN + SENSOR_DATA_LEN)
			{
	    		GPIO_PinWrite(GPIOB, 3U, dataBuf[cnt_byte - SENSOR_HEADER_LEN] & (0x80 >> cnt_bit));
//				GPIO_PinWrite(GPIOB, 3U, debug_buf[cnt_byte - SENSOR_HEADER_LEN] & (0x80 >> cnt_bit));
				if(!cnt_bit)
				{
	    			checkSum += dataBuf[cnt_byte - SENSOR_HEADER_LEN];
//					checkSum += debug_buf[cnt_byte - SENSOR_HEADER_LEN];
				}
			}
			else
			{
				GPIO_PinWrite(GPIOB, 3U, checkSum & (0x80 >> cnt_bit));
			}

			cnt_byte = (cnt_bit == 7)? (cnt_byte + 1) % SENSOR_FRAME_LEN : cnt_byte;
			cnt_bit =  (cnt_bit + 1) % 8;

			if(0 == cnt_bit && 0 == cnt_byte)
			{
				islastBit = TRUE;
			}
			else
			{
				status_send = SEND_WAIT;
			}
			break;
    	}while(1);
    }


    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}


lpuart_handle_t g_lpuartHandle;
uint8_t g_tipString[] = "LPUART RX ring buffer example\r\nSend back received data\r\nEcho every 8 types\r\n";
uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */

extern uint8_t g_rxBuffer[ECHO_BUFFER_SIZE]; /* Buffer for receive data to echo. */
extern uint8_t g_txBuffer[ECHO_BUFFER_SIZE]; /* Buffer for send data to echo. */
extern volatile bool rxBufferEmpty;
extern volatile bool txBufferFull;
extern volatile bool txOnGoing;
extern volatile bool rxOnGoing;


#define USER_PWM_NUM  1
#define WAKEUP_ENABLE 0

#include "string.h"
void user_showFreq(void)
{
//	PRINTF("\r\n--------------------------\r\n");
//    PRINTF("kCLOCK_CoreSysClk %d.\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
//    PRINTF("kCLOCK_PlatClk %d.\r\n", CLOCK_GetFreq(kCLOCK_PlatClk));
//    PRINTF("kCLOCK_BusClk %d.\r\n", CLOCK_GetFreq(kCLOCK_BusClk));
//    PRINTF("kCLOCK_FlashClk %d.\r\n", CLOCK_GetFreq(kCLOCK_FlashClk));
//    PRINTF("kCLOCK_Er32kClk %d.\r\n", CLOCK_GetFreq(kCLOCK_Er32kClk));
//    PRINTF("kCLOCK_Osc0ErClk %d.\r\n", CLOCK_GetFreq(kCLOCK_Osc0ErClk));
//    PRINTF("kCLOCK_McgInternalRefClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgInternalRefClk));
//    PRINTF("kCLOCK_McgPeriphClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgPeriphClk));
//    PRINTF("kCLOCK_McgIrc48MClk %d.\r\n", CLOCK_GetFreq(kCLOCK_McgIrc48MClk));
//    PRINTF("kCLOCK_LpoClk %d.\r\n", CLOCK_GetFreq(kCLOCK_LpoClk));


	// 2M kCLOCK_CoreSysClk
	// 2M kCLOCK_McgInternalRefClk
	// 0  kCLOCK_McgPeriphClk
    u32 freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    char buf[32] = "\0";

    sprintf(buf, "%d\n", freq);


    /* Send g_tipString out. */
    lpuart_transfer_t xfer;
    xfer.data = buf;
    xfer.dataSize = strlen(buf);
    txOnGoing = true;
    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing)
    {
    }

}


void user_timerInit(void)
{
	lptmr_config_t lptmrUserConfig;

	LPTMR_GetDefaultConfig(&lptmrUserConfig);
	/* Init LPTimer driver */
	LPTMR_Init(DEMO_LPTMR_BASE, &lptmrUserConfig);

	/* Set the LPTimer period */
	LPTMR_SetTimerPeriod(DEMO_LPTMR_BASE, LPTMR_COMPARE_VALUE);

	/* Enable timer interrupt */
	LPTMR_EnableInterrupts(DEMO_LPTMR_BASE, kLPTMR_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(DEMO_LPTMR_IRQn);
	LPTMR_StartTimer(DEMO_LPTMR_BASE);
}
void user_pwmInit(void)
{
	#ifndef TPM_LED_ON_LEVEL

	#define TPM_LED_ON_LEVEL kTPM_LowTrue
	#endif
	#if USER_PWM_NUM == 1
		tpm_config_t tpmInfo;
		tpm_chnl_pwm_signal_param_t tpmParam;

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

		tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TPM_CHANNEL;
		tpmParam[0].level = TPM_LED_ON_LEVEL;
		tpmParam[0].dutyCyclePercent = updatedDutycycle;

		tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TPM_CHANNEL;
		tpmParam[1].level = TPM_LED_ON_LEVEL;
		tpmParam[1].dutyCyclePercent = updatedDutycycle;

	#else
		#error "pwm num set err!"
	#endif

	/* Select the clock source for the TPM counter as kCLOCK_McgInternalRefClk */
	//    CLOCK_SetTpmClock(1U);
	CLOCK_SetTpmClock(3U);

	TPM_GetDefaultConfig(&tpmInfo);
	/* Initialize TPM module */
	TPM_Init(BOARD_TPM_BASEADDR, &tpmInfo);

	// redefine ch number by micro, set freq divider here
	TPM_SetupPwm(BOARD_TPM_BASEADDR, &tpmParam, USER_PWM_NUM, kTPM_CenterAlignedPwm, 500000U, 2000000); //  2M clock source @VLPR
	TPM_StartTimer(BOARD_TPM_BASEADDR, kTPM_SystemClock);
}

void user_VLPR(smc_power_state_t *pcurPowerState, app_power_mode_t *ptargetPowerMode,  bool *pneedSetWakeup)
{
	*pcurPowerState = SMC_GetPowerModeState(SMC);

//        APP_ShowPowerMode(curPowerState);
//        user_showFreqList();

    // set default powermode as vlpr
    if(*pcurPowerState != kSMC_PowerStateVlpr)
    {
    	*ptargetPowerMode = (app_power_mode_t)kAPP_PowerModeVlpr;
    }
    else
    {
    	*ptargetPowerMode = (app_power_mode_t)kAPP_PowerModeVlpr;
    }
    // A:run, D:vlpr, I:vlls3
//        PRINTF("\r\nTarget %c \r\n", targetPowerMode);

	if ((*ptargetPowerMode > kAPP_PowerModeMin) && (*ptargetPowerMode < kAPP_PowerModeMax))
	{
		/* If could not set the target power mode, loop continue. */
		if (!APP_CheckPowerMode(*pcurPowerState, *ptargetPowerMode))
		{
//				PRINTF("\r\n curPowerState %x \r\n", curPowerState);
//				continue;
//				break;
		}

		/* If target mode is RUN/VLPR/HSRUN, don't need to set wakeup source. */
		if ((kAPP_PowerModeRun == *ptargetPowerMode) || (kAPP_PowerModeVlpr == *ptargetPowerMode))
		{
			*pneedSetWakeup = false;
		}
		else
		{
			*pneedSetWakeup = true;
		}

		if (*pneedSetWakeup)
		{
			APP_GetWakeupConfig(*ptargetPowerMode);
		}

		APP_PowerPreSwitchHook(*pcurPowerState, *ptargetPowerMode);

		if (*pneedSetWakeup)
		{
			APP_SetWakeupConfig(*ptargetPowerMode);
		}

		APP_PowerModeSwitch(*pcurPowerState, *ptargetPowerMode);
		APP_PowerPostSwitchHook(*pcurPowerState, *ptargetPowerMode);
	}

}

void user_gpioInit(void)
{
    gpio_pin_config_t config_output_L = {kGPIO_DigitalOutput, 0};
    gpio_pin_config_t config_output_H = {kGPIO_DigitalOutput, 1};
    gpio_pin_config_t config_input    = {kGPIO_DigitalInput, 0};

    /* Init output ENABLE GPIO. */
    // GPIOA
    GPIO_PinInit(GPIOA, 7U, &config_output_L);
    GPIO_PinInit(GPIOA, 5U, &config_output_L);

    // GPIOB
    GPIO_PinInit(GPIOB, 3U, &config_output_L);

}

void delay(void)
{
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
}

void delay_n(uint16_t time)
{
    volatile uint32_t i = 0;
    for (i = 0; i < time; ++i)
    {
    	delay();
    }
}

//#define DEBUG_END
int main(void)
{
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

#ifndef DEBUG_END
    lpuart_config_t config;
    lpuart_transfer_t xfer;
    lpuart_transfer_t sendXfer;
    lpuart_transfer_t receiveXfer;
    size_t receivedBytes = 0U;
    uint32_t i;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    CLOCK_SetLpuart0Clock(0x1U);

    // gpio init
    user_gpioInit();

    // timer init 1ms
    user_timerInit();


    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(DEMO_LPUART, &config, DEMO_LPUART_CLK_FREQ);
    LPUART_TransferCreateHandle(DEMO_LPUART, &g_lpuartHandle, LPUART_UserCallback, NULL);
    LPUART_TransferStartRingBuffer(DEMO_LPUART, &g_lpuartHandle, g_rxRingBuffer, RX_RING_BUFFER_SIZE);


    NVIC_EnableIRQ(LLWU_IRQn);
    NVIC_EnableIRQ(APP_WAKEUP_BUTTON_IRQ);

    // enter vlpr
    user_VLPR(&curPowerState, &targetPowerMode,  &needSetWakeup);

    /* Send g_tipString out. */
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;
    txOnGoing = true;
    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing)
    {
    }


    user_showFreq();

    /* Start to echo. */
    sendXfer.data = g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_SIZE;
    receiveXfer.data = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_SIZE;

    u8 uart_rx[32] = "\0";
    u8 cnt_rx = 0;
    u8 cnt_rx_last = 0;
    u8 cnt_uart_sample = 0;

    while (1)
    {
        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
        if ((!rxBufferEmpty) && (!txBufferFull))
        {
            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_SIZE);
            rxBufferEmpty = true;
            txBufferFull = true;
        }

        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
        if ((!rxOnGoing) && rxBufferEmpty)
        {
            rxOnGoing = true;
            LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, &receivedBytes);
            if (ECHO_BUFFER_SIZE == receivedBytes)
            {
                rxBufferEmpty = false;
                rxOnGoing = false;
            }
//            memcpy(uart_rx + cnt_rx, g_rxBuffer, 1);
            uart_rx[cnt_rx] = g_rxBuffer[0];
            cnt_rx++;
        }

//        if(cnt_uart_sample ++ >= 100)
//        {
//
//        	if(cnt_rx == cnt_rx_last && cnt_rx != 0)
//            {
////        		if(cnt_rx % 6 == 0)
//        		if(cnt_rx >= 6)
//        		{
//        			memcpy(dataBuf, uart_rx, SENSOR_DATA_LEN);
//        			status_rec = REC_SUCCESS;
//        		}
//        		else
//        		{
//        			status_rec = REC_FAIL;
//        		}
//        		cnt_rx = 0;
//            }
//        	cnt_rx_last = cnt_rx;
//        }


        if(STATUS_WAIT_DATA == status)
        {

			if(cnt_rx == 6)
			{
				memcpy(dataBuf, uart_rx, SENSOR_DATA_LEN);
				status_rec = REC_SUCCESS;
				cnt_rx = 0;
				cnt_uart_sample = 0;
			}
			else if(cnt_rx > 6)
			{
				status_rec = REC_FAIL;
				cnt_rx = 0;
				cnt_uart_sample = 0;
			}
			else
			{
				cnt_uart_sample ++ ;
				status_rec = REC_WAIT;
				if(cnt_uart_sample > 999999)
				{
					status_rec = REC_FAIL;
					cnt_rx = 0;
					cnt_uart_sample = 0;
				}
			}
        }
        else
        {
        	cnt_rx = 0;
        	cnt_uart_sample = 0;
        }


        /* If TX is idle and g_txBuffer is full, start to send data. */
        if ((!txOnGoing) && txBufferFull)
        {
            txOnGoing = true;
            LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
        }

        if(STATUS_WAIT_TRIG == status)
        {
        	// IDLE
        	GPIO_PinWrite(GPIOB, 3U, 0);

        	GPIO_PinWrite(GPIOA, 5U, 1);
        	delay_n(100);
        	GPIO_PinWrite(GPIOA, 5U, 0);

//        	GPIO_PinWrite(GPIOA, 5U, 1);

        	status = STATUS_WAIT_DATA;
        	status_rec = REC_WAIT;
        }

        if(STATUS_WAIT_DATA == status)
        {
        	// IDLE
        	GPIO_PinWrite(GPIOB, 3U, 0);

        	if(REC_SUCCESS == status_rec)
        	{
        		GPIO_PinWrite(GPIOA, 5U, 1);

        		status = STATUS_WAIT_SEND;
				status_rec = REC_WAIT;
        	}
        	else if(REC_FAIL == status_rec)
        	{
        		GPIO_PinWrite(GPIOA, 5U, 1);

        		status = STATUS_WAIT_TRIG;
        		status_rec = REC_WAIT;
        	}
        	else
        	{
        		status = STATUS_WAIT_DATA;
        	}
        }

        if(STATUS_WAIT_SEND == status)
        {
        	if(SEND_SUCCESS == status_send)
        	{
        		status_send = SEND_WAIT;
        		status = STATUS_WAIT_TRIG;
        	}
        	else
        	{
        		status = STATUS_WAIT_SEND;
        	}

        }


    }


#endif


#ifdef DEBUG_END


/******************************************************************************/
    //
    BOARD_InitPins();
    user_gpioInit();

    BOARD_BootClockRUN();
    APP_InitDefaultDebugConsole();

    NVIC_EnableIRQ(LLWU_IRQn);
    NVIC_EnableIRQ(APP_WAKEUP_BUTTON_IRQ);

    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PRINTF("\r\nMCU wakeup from VLLS modes...\r\n");
    }

// lptmr init
    user_timerInit();

 // enter vlpr
    user_VLPR(&curPowerState, &targetPowerMode,  &needSetWakeup);

    // debug: show state to check
//    {
//    	u32 freq;
//        curPowerState = SMC_GetPowerModeState(SMC);
//        freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);
//        APP_ShowPowerMode(curPowerState);
//
//    	  user_showFreqList();
//    }

// pwm init
    user_pwmInit();

    while(1);
/******************************************************************************/

    // debug
//    user_showFreqList();

#if WAKEUP_ENABLE
#define SLEEP_CNT 990000
    	u32 debug_cnt = 0;
#endif
    	u8 cntBit = 0;
        while (1)
        {
            /* Prevents the use of wrong values */
            while (!conversionCompleted);

#if WAKEUP_ENABLE
            // hold output for about xxx sec

			if(debug_cnt++ > SLEEP_CNT)
			{
				debug_cnt = 0;
				PRINTF("  SLEEP.. \r\n");
				break;
			}
			else if(debug_cnt++ == SLEEP_CNT/2)
			{
				PRINTF("  SLEEP half time pass... \r\n");
            }

#endif

            // debug
//            if(0)
            {
            	static ADC_PACK adc_pack = {0};
            	static u32 adc_sum = 0;

            	ADC_DATA adc = {0};

            	data_dequeueadc(&adc);
            	adc_sum += adc.adcValue;

            	// 第一次发或发完一帧
            	if(cntBit % ADC_PACK_LEN == 0)
            	{
            		adc_pack.header = ADC_HEADER;
            		adc_pack.data = cntBit ? adc_sum / ADC_PACK_LEN : adc_sum ;
            		adc_sum = 0;

            		cntBit = 0;
            	}

            	// send header
            	if(cntBit < ADC_HEADER_LEN)
            	{
            		GPIO_WritePinOutput(GPIOB, 3U, (adc_pack.header >> (ADC_HEADER_LEN - cntBit - 1)) & 0x01);
            		adc_pack.check = 0;
            	}
            	else if(cntBit < ADC_HEADER_LEN + ADC_DATA_LEN)
            	{
            		u8 sendBit = (adc_pack.data >> (ADC_HEADER_LEN + ADC_DATA_LEN - cntBit - 1)) & 0x01;
            		GPIO_WritePinOutput(GPIOB, 3U, sendBit);
            		adc_pack.check ^= sendBit & 0x01;
            	}
            	else
            	{
            		GPIO_WritePinOutput(GPIOB, 3U, adc_pack.check);
//            		PRINTF("%x %x %x\n\r", adc_pack.header, adc_pack.data, adc_pack.check);
            	}

            	cntBit ++;
//            	PRINTF("%d\n\r", cntBit);
            }
            conversionCompleted = false;

        }

#if WAKEUP_ENABLE
    // enter vlls3 mode
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
#endif



#endif
    return 0;
}

