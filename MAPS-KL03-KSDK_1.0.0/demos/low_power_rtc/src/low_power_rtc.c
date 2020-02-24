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


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include "device/fsl_device_registers.h"
#include "fsl_gpio_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"
#include "fsl_port_hal.h"
#include "fsl_sim_hal.h"
#include "fsl_debug_console.h"
#include "fsl_smc_hal.h"
#include "fsl_rtc_hal.h"
#include "fsl_rtc_driver.h"
#include "fsl_pmc_hal.h"
#include "fsl_mcglite_hal.h"
#include "fsl_llwu_hal.h"
#include "fsl_rcm_hal.h"
#include "board.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#ifndef FALSE
#define FALSE 0
#endif
    
#ifndef TRUE
#define TRUE 1
#endif

#define DEBUG 0



/*******************************************************************************
* Global Variables
******************************************************************************/
static rtc_datetime_t rtcDatetime_alarm;          /*!<Structure of simple "date" format.  */
static uint8_t rtc_enable = 0;
static uint8_t flag_sleep;

/*******************************************************************************
* Prototypes
******************************************************************************/
static void print_reset_source(void);							  /*!< print reset source routinue */

/*!<Interrupt routinues */ 
static void portb_isr(void);					/*!<PortB interrupt routinue */
static void porta_isr(void);					/*!<PortA interrupt routinue */
static void llwu_isr(void);						/*!<Llwu interrupt routinue */
static void rtc_alarm_isr(void);				/*!<rtc alarm interrupt routinue */

static void port_config_lowpower(void);  /*!<config all ports to make chip enter low power demo */

/*!<Low power mode entry routinues */ 
static void enter_stop(smc_pstop_option_t partialStopOpt);    /*!<Normal stop/Pstop1/Pstop2 mode entry routinue */
static void enter_vllsx(smc_por_option_t PORPOValue,smc_stop_submode_t VLLSValue);    /*!<VLLSx mode entry routinue */
static uint32_t enter_vlpr(void);								   /*!<VLPR mode entry routinue */
static void enter_vlps(void);									   /*!<VLPS mode entry routinue */
static void enter_wait(power_modes_t pMode);                       /*!<Normal WAIT and VLPW mode entry routinue */






/*** user func declear ***/
void sleep_setSW(uint8_t enSlp);






/*******************************************************************************
* Code
******************************************************************************/
//          RUN -> VLPR -> VLLS3
//           |               |
//           ------------- RESET               
int main (void)
{
    // init gpio  B5
    hardware_init();
    
    // init lpuart
    // ALT2: B1(TX) B2(RX)  9600
    dbg_uart_init();

    printf("\n\rLow power demo Start!\n\r");

    // 判断复位原因
    print_reset_source();
    
    CLOCK_HAL_SetSource(SIM_BASE,kClockOsc32kSel,0x0);/*set system oscillator(OSC32KCLK) as RTC clock*/
    CLOCK_SYS_EnableRtcClock(0); 
    /* Configure the RTC date and time */
//    rtcDatetime_alarm.year = 2014;
//    rtcDatetime_alarm.month = 3;
//    rtcDatetime_alarm.day = 21;
//    rtcDatetime_alarm.hour = 5;
//    rtcDatetime_alarm.minute = 22;
//    rtcDatetime_alarm.second = 0;
//    /* Set the Date and Time */
//    RTC_DRV_SetDatetime(0,&rtcDatetime_alarm);
//    
//    /* Stop timer just in case */
//    RTC_HAL_EnableCounter(RTC_BASE, false);
//    
//    /* Turn on the RTC Oscillator */
//    RTC_HAL_SetOscillatorCmd(RTC_BASE, true);
//    
//    /* Install the ISR function for seconds */
//    INT_SYS_InstallHandler(RTC_IRQn, &rtc_alarm_isr);
//    INT_SYS_EnableIRQ(RTC_IRQn);
    
//    /* Set the Alarm Time */
//    rtcDatetime_alarm.second = 2;
//    RTC_DRV_SetAlarm(0, &rtcDatetime_alarm, true);
//    
//    /* Stop the timer */
//    RTC_HAL_EnableCounter(RTC_BASE, false);
    
    /* Configure both pushbuttons for falling edge interrupts */
    // A0 B5
//    llwuWakeupPins[0].config.interrupt = kPortIntRisingEdge; /* K2 on Main */
//    llwuWakeupPins[1].config.interrupt = kPortIntFallingEdge; /* K1 on Dock*/

    // B0 llwu
    llwuWakeupPins[0].config.interrupt = kPortIntRisingEdge; 
    GPIO_DRV_Init(llwuWakeupPins, NULL); // INPUT , OUTPUT

//    // set pin irq, clear flag in isr
//    INT_SYS_InstallHandler(PORTB_IRQn, &portb_isr);
//    INT_SYS_EnableIRQ(PORTB_IRQn);
//    INT_SYS_InstallHandler(PORTA_IRQn, &porta_isr);
//    INT_SYS_EnableIRQ(PORTA_IRQn);



    // LLWU PIN   A0/B0
//    PORT_HAL_SetMuxMode(PORTA_BASE, 0u, kPortMuxAsGpio);
//    LLWU_HAL_SetExternalInputPinMode(LLWU_BASE,kLlwuExternalPinRisingEdge,7u);
//    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, 7u);
//    INT_SYS_InstallHandler(LLWU_IRQn, &llwu_isr);
//    INT_SYS_EnableIRQ(LLWU_IRQn);

    // B0 llwu 上升沿唤醒
    PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);
    LLWU_HAL_SetExternalInputPinMode(LLWU_BASE,kLlwuExternalPinRisingEdge,7u);
    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, 7u);
    INT_SYS_InstallHandler(LLWU_IRQn, &llwu_isr);
    INT_SYS_EnableIRQ(LLWU_IRQn);
    
    while(1) 
    {
#if DEBUG
        
        //port_config_lowpower();
        printf("\n\r ********************************************************* \n");
        printf("\n\r KL03 Power Managerment Test.");
        printf("\n\r 1.Normal STOP mode");
        printf("\n\r 2.PSTOP1  mode");
        printf("\n\r 3.PSTOP2  mode");
        printf("\n\r 4.VLLS0 mode with POR enabled!");
        printf("\n\r 5.VLLS0 mode with POR disabled!");
        printf("\n\r 6.VLLS1 mode");
        printf("\n\r 7.VLLS3 mode");
        printf("\n\r 8.VLPR mode");
        printf("\n\r 9.Normal WAIT mode");
        printf("\n\r a.VLPW mode");      
        printf("\n\r b.VLPS mode");
        printf("\n\r c.Enable RTC as wakeup source,wait about 3 seconds to wakeup chip from low power mode!");
        printf("\n\r d.Disable RTC!");
        printf("\n\r Notes1:K2 on Main board can be used as STOP/WAIT/VLPS/VLLSx Wakeup source!");
        printf("\n\r Notes2:K1(Select) on Dock board can be used as STOP/WAIT/VLPS Wakeup source!");
        printf("\n\r Notes3:RTC can be used as STOP/WAIT/VLPS/VLLSx Wakeup source,except VLLS0!");
        printf("\n\r ******************************************************** \n");
        printf("\n\r Enter one key number to choose low power mode: "); 
#endif

//        if(rtc_enable == 1)
//        {
//            RTC_HAL_EnableCounter(RTC_BASE, true);
//        }
//        else
//        {
//            RTC_HAL_EnableCounter(RTC_BASE, false);
//        }
        
//        case 5:  
//            printf("\n\rVLLS0 mode with POR disabled entered!\n\r");
//            enter_vllsx(kSmcPorDisabled,kSmcStopSub0);
//            break;
//        case 6:  
//            printf("\n\rVLLS1 mode entered!\n\r");
//            enter_vllsx((smc_por_option_t)NULL,kSmcStopSub1);
//            break;


        // VLPR
        /*if in LIRC8Mhz */
//            CLOCK_HAL_SetOutDividers(SIM_BASE,1,0,0,3);//Busclk = 8Mhz/2 /2 /4= 500K
        CLOCK_HAL_SetOutDividers(SIM_BASE,1,0,0,3);//Busclk = 8Mhz/2 /2 /4= 500K
        if ((enter_vlpr())== kStatVlpr)
        {
            printf("\n\rVLPR mode entered!\n ");

            flag_sleep = TRUE;
        } 
        else 
        {
            printf("\n\rFAIL!VLPR mode didn't enter successfully!\n\r");
        }


        //  sleep in VLLS3
        if(flag_sleep)
        {
            printf("\n\rVLLS3 mode entered!\n\r");
            enter_vllsx((smc_por_option_t)NULL,kSmcStopSub3);
        }
            
//        RTC_HAL_EnableCounter(RTC_BASE, false);
//        // stop uart
//        PORT_HAL_SetMuxMode(PORTB_BASE,1u,kPortPinDisabled);
//        PORT_HAL_SetMuxMode(PORTB_BASE,2u,kPortPinDisabled);
//        DbgConsole_DeInit();
//        dbg_uart_init();
        
        if (SMC_HAL_GetStat(SMC_BASE) == kStatVlpr)
        {   
            printf("\n\rIn VLPR Mode Now\n"); 
        }  
        else 
        {   
            printf("\n\rIn RUN Mode Now\n");
        }  
        
    }
    
}


void sleep_setSW(uint8_t enSlp)
{
    // 配置唤醒 pin 使能

}




/*!
* @brief portb interrupt routinue
*/
static void portb_isr(void)
{
    GPIO_DRV_ClearPinIntFlag(kGpioSW1);
}

/*!
* @brief porta interrupt routinue
*/
static void porta_isr(void)
{
    GPIO_DRV_ClearPinIntFlag(kGpiollwuWakeup);
}


/*!
* @brief llwu interrupt routinue
*/
static void llwu_isr(void){
    
    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, 7u);
    
}

/*!
* @brief Normal stop/Pstop/Pstop2 mode entry routinue
*/
static void enter_stop(smc_pstop_option_t partialStopOpt)
{
    smc_power_mode_config_t smcConfig;
    smcConfig.porOption = false;
    smcConfig.pstopOption = true;
    smcConfig.powerModeName = kPowerModeStop;
    smcConfig.pstopOptionValue = partialStopOpt;
    SMC_HAL_SetMode(SMC_BASE, &smcConfig);
}


/*!
* @brief VLLSx mode entry routinue
*/
static void enter_vllsx(smc_por_option_t PORPOValue,smc_stop_submode_t VLLSValue)
{
    smc_power_mode_config_t smcConfig;
    
    /* set power mode to specific Run mode */
    smcConfig.porOption = true;
    smcConfig.porOptionValue = (smc_por_option_t)PORPOValue;
    smcConfig.powerModeName = kPowerModeVlls;
    smcConfig.stopSubMode = (smc_stop_submode_t)VLLSValue;
    SMC_HAL_SetMode(SMC_BASE, &smcConfig);
}


/*!
* @brief VLPR mode entry routinue
*/
static uint32_t enter_vlpr(void)
{
    smc_power_mode_config_t smcConfig;
    int32_t i;
    uint32_t returnValue = 0;  /*default return value = indicates error */
    if (SMC_HAL_GetStat(SMC_BASE) == kStatVlpr){
        
        printf("\n\r Chip is already in VLPR Mode !\n ");
        return kStatVlpr;
    }
    smcConfig.porOption = false;
    smcConfig.powerModeName = kPowerModeVlpr;
    SMC_HAL_SetMode(SMC_BASE, &smcConfig);
    /* Wait for VLPS regulator mode to be confirmed */
    for (i = 0 ; i < 10000 ; i++)
    {
        if(PMC_HAL_GetRegulatorStatus(PMC_BASE)==PMC_REGSC_REGONS_MASK){
        }
        else
        {
            break;
        }
    }
    if(PMC_HAL_GetRegulatorStatus(PMC_BASE)==PMC_REGSC_REGONS_MASK)
    {
        returnValue = 0x24;
    }
    
    if (SMC_HAL_GetStat(SMC_BASE) == kStatVlpr)
    {
        returnValue = SMC_HAL_GetStat(SMC_BASE);
    }
    return (returnValue);
}


/*!
* @brief Normal WAIT and VLPW mode entry routinue
*/
static void enter_wait(power_modes_t pMode)
{
    smc_power_mode_config_t smcConfig;
    smcConfig.porOption = false;
    smcConfig.powerModeName = pMode;
    SMC_HAL_SetMode(SMC_BASE, &smcConfig);
}

/*!
* @brief VLPS mode entry routinue
*/
static void enter_vlps(void)
{
    smc_power_mode_config_t smcConfig;
    smcConfig.porOption = false;
    smcConfig.powerModeName = kPowerModeVlps;
    SMC_HAL_SetMode(SMC_BASE, &smcConfig);
}

/*!
* @brief print reset source routinue
*/
static void print_reset_source(void){					
    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmStopModeAckErr))
    {   
        printf("Stop Mode Acknowledge Error Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmSystem))
    {    
        printf("MDM-AP Reset\n\r");
    }
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmSoftware))
    {    
        printf("Software Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmCoreLockup))
    {    
        printf("Core Lockup Event Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmPowerOn))
    {    
        printf("Power-on Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmExternalPin))
    {    
        printf("External Pin Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmWatchDog))
    {    
        printf("Watchdog(COP) Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmLowVoltDetect))
    {    
        printf("Low-voltage Detect Reset\n\r");
    }    
    if (RCM_HAL_GetSrcStatusCmd(RCM_BASE, kRcmWakeup))
    {
        printf("\r[outSRS]Wakeup bit set from low power mode exit\n\r");
        
        if ((SMC_HAL_GetStopMode(SMC_BASE)== kSmcVlls) && (SMC_HAL_GetStopSubMode(SMC_BASE)== kSmcStopSub0))
        {    
            printf("\r[outSRS] VLLS0 exit \n\r") ;
        }  
        if ((SMC_HAL_GetStopMode(SMC_BASE)== kSmcVlls) && (SMC_HAL_GetStopSubMode(SMC_BASE)== kSmcStopSub1))
        {    
            printf("\r[outSRS] VLLS1 exit \n\r") ;
        }  
        if ((SMC_HAL_GetStopMode(SMC_BASE)== kSmcVlls) && (SMC_HAL_GetStopSubMode(SMC_BASE)== kSmcStopSub2))
        {    
            printf("\r[outSRS] VLLS2 exit \n\r") ;
        }  
        if ((SMC_HAL_GetStopMode(SMC_BASE)== kSmcVlls) && (SMC_HAL_GetStopSubMode(SMC_BASE)== kSmcStopSub3))
        {    
            printf("\r[outSRS] VLLS3 exit \n\r") ; 
        }  
    }
}


/*!
* @brief rtc alarm interrupt routinue
*/
static void rtc_alarm_isr(void)
{
    if(RTC_HAL_HasAlarmOccured(RTC_BASE))
    {
        rtcDatetime_alarm.second = 0;
        RTC_DRV_SetDatetime(0,&rtcDatetime_alarm);
        rtcDatetime_alarm.second = 2;
        RTC_DRV_SetAlarm(0, &rtcDatetime_alarm, true);
    }
}

/*!
* @brief port configuration for low power
*/
static void port_config_lowpower(void)
{
    
    /*PORTA mux config*/
    PORT_HAL_SetMuxMode(PORTA_BASE,0u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,1u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,2u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,3u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,4u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,5u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,6u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,7u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,8u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,9u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE,12u,kPortMuxAsGpio);
    
    /*PORTB mux config*/
    PORT_HAL_SetMuxMode(PORTB_BASE,0u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,3u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,4u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,5u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,6u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,7u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,10u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,11u,kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE,13u,kPortMuxAsGpio);
    
    /*PORTA output config*/
    GPIO_HAL_SetPinDir(PTA_BASE, 0u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 1u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 2u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 3u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 4u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 5u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 6u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 7u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 8u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 9u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTA_BASE, 12u,kGpioDigitalOutput);
    
    /*PORTB output config*/
    GPIO_HAL_SetPinDir(PTB_BASE, 0u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 3u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 4u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 5u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 6u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 7u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 10u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 11u,kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(PTB_BASE, 13u,kGpioDigitalOutput);
    
    /*PORTA output level 0*/
    GPIO_HAL_ClearPinOutput(PTA_BASE, 0u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 1u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 2u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 3u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 4u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 5u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 6u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 7u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 8u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 9u);
    GPIO_HAL_ClearPinOutput(PTA_BASE, 12u);
    
    /*PORTB output level 0*/	
    GPIO_HAL_ClearPinOutput(PTB_BASE, 0u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 3u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 4u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 5u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 6u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 7u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 10u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 11u);
    GPIO_HAL_ClearPinOutput(PTB_BASE, 13u);
    
}

/********************************************************************/
/********************************************************************/
