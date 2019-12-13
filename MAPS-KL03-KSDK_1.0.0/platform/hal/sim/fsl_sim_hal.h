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

#if !defined(__FSL_SIM_HAL_H__)
#define __FSL_SIM_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"


/*! @addtogroup sim_hal*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum _clock_names {

   /* default clocks*/
   kCoreClock,                         /**/
   kSystemClock,                       /**/
   kPlatformClock,                     /**/
   kBusClock,                          /**/
   kFlexBusClock,                      /**/
   kFlashClock,                        /**/

   /* other internal clocks used by peripherals*/
   /* osc clock*/
   kOsc32kClock,
   kOsc0ErClock,
   kOsc1ErClock,

   /* irc 48Mhz clock */
   kIrc48mClock,

   /* rtc clock*/
   kRtc32kClock,
   kRtc1hzClock,

   /* lpo clcok*/
   kLpoClock,

   /* mcg clocks*/
   kMcgFfClock,
   kMcgFllClock,
   kMcgPll0Clock,
   kMcgPll1Clock,
   kMcgOutClock,
   kMcgIrClock,

   /* constant clocks (provided in other header files?)*/
   kSDHC0_CLKIN,
   kENET_1588_CLKIN,
   kEXTAL_Clock,
   kEXTAL1_Clock,
   kUSB_CLKIN,

   /* reserved value*/
   kReserved,

   kClockNameCount
} clock_names_t;

/*! @brief Clock source and sel names */
typedef enum _clock_source_names {
    kClockNfcSrc,                   /* NFCSRC*/
    kClockEsdhcSrc,                 /* ESDHCSRC K70*/
    kClockSdhcSrc,                  /* SDHCSRC  K64*/
    kClockLcdcSrc,                  /* LCDCSRC*/
    kClockTimeSrc,                  /* TIMESRC*/
    kClockRmiiSrc,                  /* RMIISRC*/
    kClockUsbfSrc,                  /* USBFSRC  K70*/
    kClockUsbSrc,                   /* USBSRC   K64, KL25, KV31, and K22*/
    kClockUsbhSrc,                  /* USBHSRC*/
    kClockUart0Src,                 /* UART0SRC*/
    kClockLpuartSrc,                /* LPUARTSRC K22, KV31 */
    kClockLpuart0Src,               /* LPUART0SRC KL03 */
    kClockTpmSrc,                   /* TPMSRC*/
    kClockOsc32kSel,                /* OSC32KSEL*/
    kClockOsc32kOut,                /* OSC32KOUT*/
    kClockUsbfSel,                  /* USBF_CLKSEL*/
    kClockPllfllSel,                /* PLLFLLSEL*/
    kClockNfcSel,                   /* NFC_CLKSEL*/
    kClockLcdcSel,                  /* LCDC_CLKSEL*/
    kClockTraceSel,                 /* TRACE_CLKSEL*/
    kClockClkoutSel,                /* CLKOUTSEL*/
    kClockRtcClkoutSel,             /* RTCCLKOUTSEL */
    kClockSourceMax
} clock_source_names_t;

/*! @brief Clock Divider names*/
typedef enum _clock_divider_names {
    kClockDividerOutdiv1,           /* OUTDIV1*/
    kClockDividerOutdiv2,           /* OUTDIV2*/
    kClockDividerOutdiv3,           /* OUTDIV3*/
    kClockDividerOutdiv4,           /* OUTDIV4*/
    kClockDividerUsbFrac,           /* (USBFRAC + 1) / (USBDIV + 1)*/
    kClockDividerUsbDiv,
    kClockDividerUsbfsFrac,         /* (USBFSFRAC + 1) / (USBFSDIV) + 1)*/
    kClockDividerUsbfsDiv,
    kClockDividerUsbhsFrac,         /* (USBHSFRAC + 1) / (USBHSDIV + 1)*/
    kClockDividerUsbhsDiv,
    kClockDividerLcdcFrac,          /* (LCDCFRAC + 1) / (LCDCDIV + 1)*/
    kClockDividerLcdcDiv,
    kClockDividerNfcFrac,           /* (NFCFRAC + 1) / (NFCDIV + 1)*/
    kClockDividerNfcDiv,
    kClockDividerSpecial1,          /* special divider 1*/
    kClockDividerMax
} clock_divider_names_t;

/*! @brief SIM USB voltage regulator in standby mode setting during stop modes */
typedef enum _sim_usbsstby_stop
{
    kSimUsbsstbyNoRegulator,        /* regulator not in standby during Stop modes */
    kSimUsbsstbyWithRegulator       /* regulator in standby during Stop modes */
} sim_usbsstby_stop_t;

/*! @brief SIM USB voltage regulator in standby mode setting during VLPR and VLPW modes */
typedef enum _sim_usbvstby_stop
{
    kSimUsbvstbyNoRegulator,        /* regulator not in standby during VLPR and VLPW modes */
    kSimUsbvstbyWithRegulator       /* regulator in standby during VLPR and VLPW modes */
} sim_usbvstby_stop_t;

/*! @brief SIM CMT/UART pad drive strength */
typedef enum _sim_cmtuartpad_strengh
{
    kSimCmtuartSinglePad,           /* Single-pad drive strength for CMT IRO or UART0_TXD */
    kSimCmtuartDualPad              /* Dual-pad drive strength for CMT IRO or UART0_TXD */
} sim_cmtuartpad_strengh_t;

/*! @brief SIM PTD7 pad drive strength */
typedef enum _sim_ptd7pad_strengh
{
    kSimPtd7padSinglePad,           /* Single-pad drive strength for PTD7 */
    kSimPtd7padDualPad              /* Dual-pad drive strength for PTD7 */
} sim_ptd7pad_strengh_t;

/*! @brief SIM FlexBus security level */
typedef enum _sim_flexbus_security_level
{
    kSimFbslLevel0,                 /* All off-chip accesses (op code and data) via the FlexBus */
                                    /* and DDR controller are disallowed */
    kSimFbslLevel1,                 /* Undefined */
    kSimFbslLevel2,                 /* Off-chip op code accesses are disallowed. Data accesses */
                                    /* are allowed */
    kSimFbslLevel3                  /* Off-chip op code accesses and data accesses are allowed */
} sim_flexbus_security_level_t;

/*! @brief SIM ADCx pre-trigger select */
typedef enum _sim_pretrgsel
{
    kSimAdcPretrgselA,              /* Pre-trigger A selected for ADCx */
    kSimAdcPretrgselB               /* Pre-trigger B selected for ADCx */
} sim_pretrgsel_t;

/*! @brief SIM ADCx trigger select */
typedef enum _sim_trgsel
{
    kSimAdcTrgselExt,               /* External trigger */
    kSimAdcTrgSelHighSpeedComp0,    /* High speed comparator 0 asynchronous interrupt */
    kSimAdcTrgSelHighSpeedComp1,    /* High speed comparator 1 asynchronous interrupt */
    kSimAdcTrgSelHighSpeedComp2,    /* High speed comparator 2 asynchronous interrupt */
    kSimAdcTrgSelPit0,              /* PIT trigger 0 */
    kSimAdcTrgSelPit1,              /* PIT trigger 1 */
    kSimAdcTrgSelPit2,              /* PIT trigger 2 */
    kSimAdcTrgSelPit3,              /* PIT trigger 3 */
    kSimAdcTrgSelTpm0,              /* TPM0 trigger */
    kSimAdcTrgSelTpm1,              /* TPM1 trigger */
    kSimAdcTrgSelFtm2,              /* FTM2 trigger */
    kSimAdcTrgSelFtm3,              /* FTM3 trigger */
    kSimAdcTrgSelRtcAlarm,          /* RTC alarm */
    kSimAdcTrgSelRtcSec,            /* RTC seconds */
    kSimAdcTrgSelLptimer,           /* Low-power timer trigger */
    kSimAdcTrgSelHigSpeedComp3      /* High speed comparator 3 asynchronous interrupt */
} sim_trgsel_t;

/*! @brief SIM receive data source select */
typedef enum _sim_uart_rxsrc
{
    kSimUartRxsrcPin,               /* UARTx_RX Pin */
    kSimUartRxsrcCmp0,              /* CMP0 */
    kSimUartRxsrcCmp1,              /* CMP1 */
    kSimUartRxsrcReserved           /* Reserved */
} sim_uart_rxsrc_t;

/*! @brief SIM transmit data source select */
typedef enum _sim_uart_txsrc
{
    kSimUartTxsrcPin,               /* UARTx_TX Pin */
    kSimUartTxsrcCmp0,              /* UARTx_TX pin modulated with FTM1 channel 0 output */
    kSimUartTxsrcCmp1,              /* UARTx_TX pin modulated with FTM2 channel 0 output */
    kSimUartTxsrcReserved           /* Reserved */
} sim_uart_txsrc_t;

/*! @brief SIM FlexTimer x trigger y select */
typedef enum _sim_ftm_trg_src
{
    kSimFtmTrgSrc0,                 /* FlexTimer x trigger y select 0 */
    kSimFtmTrgSrc1                  /* FlexTimer x trigger y select 1 */
} sim_ftm_trg_src_t;

/*! @brief SIM FlexTimer external clock select */
typedef enum _sim_ftm_clk_sel
{
    kSimFtmClkSel0,                 /* FTM CLKIN0 pin. */
    kSimFtmClkSel1                  /* FTM CLKIN1 pin. */
} sim_ftm_clk_sel_t;

/*! @brief SIM FlexTimer x channel y input capture source select */
typedef enum _sim_ftm_ch_src
{
    kSimFtmChSrc0,                 /* See RM for details of each selection for each channel */
    kSimFtmChSrc1,                 /* See RM for details of each selection for each channel */
    kSimFtmChSrc2,                 /* See RM for details of each selection for each channel */
    kSimFtmChSrc3                  /* See RM for details of each selection for each channel */
} sim_ftm_ch_src_t;

/*! @brief SIM FlexTimer x Fault y select */
typedef enum _sim_ftm_flt_sel
{
    kSimFtmFltSel0,                 /* FlexTimer x fault y select 0 */
    kSimFtmFltSel1                  /* FlexTimer x fault y select 1 */
} sim_ftm_flt_sel_t;

/*! @brief SIM Timer/PWM external clock select */
typedef enum _sim_tpm_clk_sel
{
    kSimTpmClkSel0,                 /* Timer/PWM TPM_CLKIN0 pin. */
    kSimTpmClkSel1                  /* Timer/PWM TPM_CLKIN1 pin. */
} sim_tpm_clk_sel_t;

/*! @brief SIM Timer/PWM x channel y input capture source select */
typedef enum _sim_tpm_ch_src
{
    kSimTpmChSrc0,                 /* TPMx_CH0 signal */
    kSimTpmChSrc1                  /* CMP0 output */
} sim_tpm_ch_src_t;

/*! @brief SIM HAL API return status*/
typedef enum _sim_hal_status {
    kSimHalSuccess,
    kSimHalFail,
    kSimHalNoSuchModule,
    kSimHalNoSuchClockSrc,
    kSimHalNoSuchDivider
} sim_hal_status_t;

/*! @brief Clock name configuration table structure*/
typedef struct ClockNameConfig {
    bool                            useOtherRefClock;     /*!< if it  uses the other ref clock*/
    clock_names_t                   otherRefClockName;    /*!< other ref clock name*/
    clock_divider_names_t           dividerName;          /*!< clock divider name*/
} clock_name_config_t;

/*! @brief clock name configuration table for specified CPU defined in fsl_clock_module_names_Kxxx.h*/
extern const clock_name_config_t kClockNameConfigTable[];


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name clock-related feature APIs*/
/*@{*/

/*!
 * @brief Sets the clock source setting.
 *
 * This function  sets the settings for a specified clock source. Each clock 
 * source has its own clock selection settings. See the chip reference manual for 
 * clock source detailed settings and the clock_source_names_t 
 * for clock sources.
 *
 * @param baseAddr    Base address for current SIM instance.
 * @param clockSource Clock source name defined in sim_clock_source_names_t
 * @param setting     Setting value
 * @return status     If the clock source doesn't exist, it returns an error.
 */
sim_hal_status_t CLOCK_HAL_SetSource(uint32_t baseAddr, clock_source_names_t clockSource, uint8_t setting);

/*!
 * @brief Gets the clock source setting.
 *
 * This function  gets the settings for a specified clock source. Each clock
 * source has its own clock selection settings. See the reference manual for
 * clock source detailed settings and the clock_source_names_t
 * for clock sources.
 *
 * @param baseAddr    Base address for current SIM instance.
 * @param clockSource Clock source name
 * @param setting     Current setting pointer for the clock source
 * @return status     If the clock source doesn't exist, it returns an error.
 */
sim_hal_status_t CLOCK_HAL_GetSource(uint32_t baseAddr, clock_source_names_t clockSource, 
                                            uint8_t *setting);

/*!
 * @brief Sets the clock divider setting.
 *
 * This function  sets the setting for a specified clock divider. See the
 * reference manual for a supported clock divider and value range and the
 * clock_divider_names_t for dividers.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param clockDivider Clock divider name
 * @param setting      Divider setting
 * @return status      If the clock divider doesn't exist, it  returns an error.
 */
sim_hal_status_t CLOCK_HAL_SetDivider(uint32_t baseAddr, clock_divider_names_t clockDivider, 
                                             uint32_t setting);

/*!
 * @brief Sets the clock out dividers setting.
 *
 * This function  sets the setting for all clock out dividers at the same time.
 * See the reference manual for a supported clock divider and value range and the
 * clock_divider_names_t for clock out dividers.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
void CLOCK_HAL_SetOutDividers(uint32_t baseAddr, uint32_t outdiv1, uint32_t outdiv2,
                                      uint32_t outdiv3, uint32_t outdiv4);

/*!
 * @brief Gets the clock divider setting.
 *
 * This function  gets the setting for a specified clock divider. See the
 * reference manual for a supported clock divider and value range and the 
 * clock_divider_names_t for dividers.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param clockDivider Clock divider name
 * @param setting      Divider value pointer
 * @return status      If the clock divider doesn't exist, it  returns an error.
 */
sim_hal_status_t CLOCK_HAL_GetDivider(uint32_t baseAddr, clock_divider_names_t clockDivider,
                                             uint32_t *setting);

/*@}*/

/*! @name individual field access APIs*/
/*@{*/








/*!
 * @brief Sets the ADCx alternate trigger enable setting.
 *
 * This function  enables/disables the alternative conversion triggers for ADCx.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param enable Enable alternative conversion triggers for ADCx
 *               - true: Select alternative conversion trigger.
 *               - false: Select PDB trigger.
 */
void SIM_HAL_SetAdcAlternativeTriggerCmd(uint32_t baseAddr, uint8_t instance, bool enable);

/*!
 * @brief Gets the  ADCx alternate trigger enable setting.
 *
 * This function  gets the  ADCx alternate trigger enable setting.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @return enabled True if  ADCx alternate trigger is enabled
 */
bool SIM_HAL_GetAdcAlternativeTriggerCmd(uint32_t baseAddr, uint8_t instance);

/*!
 * @brief Sets the ADCx pre-trigger select setting.
 *
 * This function  selects the ADCx pre-trigger source when the alternative triggers
 * are enabled through ADCxALTTRGEN.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select pre-trigger select setting for ADCx
 *               - 0: Pre-trigger A selected for ADCx.
 *               - 1: Pre-trigger B selected for ADCx.
 */
void SIM_HAL_SetAdcPreTriggerMode(uint32_t baseAddr, uint8_t instance, sim_pretrgsel_t select);

/*!
 * @brief Gets the ADCx pre-trigger select setting.
 *
 * This function  gets the ADCx pre-trigger select setting.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select ADCx pre-trigger select setting
 */
sim_pretrgsel_t SIM_HAL_GetAdcPreTriggerMode(uint32_t baseAddr, uint8_t instance);

/*!
 * @brief Sets the ADCx trigger select setting.
 *
 * This function  selects the ADCx trigger source when alternative triggers
 * are enabled through ADCxALTTRGEN.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select trigger select setting for ADCx
 *               - 0000: External trigger
 *               - 0001: High speed comparator 0 asynchronous interrupt
 *               - 0010: High speed comparator 1 asynchronous interrupt
 *               - 0011: High speed comparator 2 asynchronous interrupt
 *               - 0100: PIT trigger 0
 *               - 0101: PIT trigger 1
 *               - 0110: PIT trigger 2
 *               - 0111: PIT trigger 3
 *               - 1000: FTM0 trigger
 *               - 1001: FTM1 trigger
 *               - 1010: FTM2 trigger
 *               - 1011: FTM3 trigger
 *               - 1100: RTC alarm
 *               - 1101: RTC seconds
 *               - 1110: Low-power timer trigger
 *               - 1111: High speed comparator 3 asynchronous interrupt
*/
void SIM_HAL_SetAdcTriggerMode(uint32_t baseAddr, uint8_t instance, sim_trgsel_t select);

/*!
 * @brief Gets the ADCx trigger select setting.
 *
 * This function  gets the ADCx trigger select setting.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select ADCx trigger select setting
 */
sim_pretrgsel_t SIM_HAL_GetAdcTriggerMode(uint32_t baseAddr, uint8_t instance);


#ifdef HW_LPUART_INSTANCE_COUNT
/*!
 * @brief Sets the LPUARTx receive data source select setting.
 *
 * This function  selects the source for the LPUARTx receive data.
 *
 * @param select the source for the LPUARTx receive data
 *               - 00: LPUARTx_RX pin.
 *               - 01: CMP0.
 *               - 11: Reserved.
 */
void SIM_HAL_SetLpUartRxSrcMode(uint32_t baseAddr, uint8_t instance, sim_uart_rxsrc_t select);

/*!
 * @brief Gets the LPUARTx receive data source select setting.
 *
 * This function  gets the LPUARTx receive data source select setting.
 *
 * @return select UARTx receive data source select setting
 */
sim_uart_rxsrc_t SIM_HAL_GetLpUartRxSrcMode(uint32_t baseAddr, uint8_t instance);

/*!
 * @brief Sets the LPUARTx transmit data source select setting.
 *
 * This function  selects the source for the LPUARTx transmit data.
 *
 * @param select the source for the UARTx transmit data
 *               - 00: LPUARTx_TX pin.
 *               - 01: LPUARTx_TX pin modulated with FTM1 channel 0 output.
 *               - 11: Reserved.
 */
void SIM_HAL_SetLpUartTxSrcMode(uint32_t baseAddr, uint8_t instance, sim_uart_txsrc_t select);

/*!
 * @brief Gets the LPUARTx transmit data source select setting.
 *
 * This function  gets the LPUARTx transmit data source select setting.
 *
 * @return select UARTx transmit data source select setting
 */
sim_uart_txsrc_t SIM_HAL_GetLpUartTxSrcMode(uint32_t baseAddr, uint8_t instance);

/*!
 * @brief Sets the LPUARTx Open Drain Enable setting.
 *
 * This function  enables/disables the LPUARTx Open Drain.
 *
 * @param enable Enable/disable LPUARTx Open Drain
 *               - True: Enable LPUARTx Open Drain
 *               - False: Disable LPUARTx Open Drain
 */
void SIM_HAL_SetLpUartOpenDrainCmd(uint32_t baseAddr, uint8_t instance, bool enable);

/*!
 * @brief Gets the LPUARTx Open Drain Enable setting.
 *
 * This function  gets the LPUARTx Open Drain Enable setting.
 *
 * @return enabled True if LPUARTx Open Drain is enabled.
 */
bool SIM_HAL_GetLpUartOpenDrainCmd(uint32_t baseAddr, uint8_t instance);
#endif



/*!
 * @brief Sets the Timer/PWM x external clock pin select setting.
 *
 * This function  selects the source of the Timer/PWM x external clock pin select.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select Timer/PWM x external clock pin select
 *               - 0: Timer/PWM x external clock driven by the TPM_CLKIN0 pin.
 *               - 1: Timer/PWM x external clock driven by the TPM_CLKIN1 pin.
 */
void SIM_HAL_SetTpmExternalClkPinSelMode(uint32_t baseAddr, uint8_t instance, sim_tpm_clk_sel_t select);

/*!
 * @brief Gets the Timer/PWM x external clock pin select setting.
 *
 * This function  gets the Timer/PWM x external clock pin select setting.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select Timer/PWM x external clock pin select setting
 */
sim_tpm_clk_sel_t SIM_HAL_GetTpmExternalClkPinSelMode(uint32_t baseAddr, uint8_t instance);

/*!
 * @brief Sets the Timer/PWM x channel y input capture source select setting.
 *
 * This function  selects the Timer/PWM x channel y input capture source.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param channel      TPM channel y
 * @param select Timer/PWM x channel y input capture source
 *               - 0: TPMx_CH0 signal
 *               - 1: CMP0 output
 */
void SIM_HAL_SetTpmChSrcMode(uint32_t baseAddr, uint8_t instance, uint8_t channel, sim_tpm_ch_src_t select);

/*!
 * @brief Gets the Timer/PWM x channel y input capture source select setting.
 *
 * This function  gets the Timer/PWM x channel y input capture source select setting.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param instance     device instance.
 * @param channel      Tpm channel y
 * @return select Timer/PWM x channel y input capture source select setting
 */
sim_tpm_ch_src_t SIM_HAL_GetTpmChSrcMode(uint32_t baseAddr, uint8_t instance, uint8_t channel);


/*!
 * @brief Gets the Kinetis Sub-Family ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Sub-Family ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Sub-Family ID
 */
static inline uint32_t SIM_HAL_GetSubFamilyId(uint32_t baseAddr)
{
    return BR_SIM_SDID_SUBFAMID(baseAddr);
}

/*!
 * @brief Gets the Kinetis SeriesID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Series ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Series ID
 */
static inline uint32_t SIM_HAL_GetSeriesId(uint32_t baseAddr)
{
    return BR_SIM_SDID_SERIESID(baseAddr);
}

/*!
 * @brief Gets the Kinetis Fam ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Fam ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Fam ID
 */
static inline uint32_t SIM_HAL_GetFamId(uint32_t baseAddr)
{
    return BR_SIM_SDID_FAMID(baseAddr);
}

/*!
 * @brief Gets the Kinetis Pincount ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Pincount ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Pincount ID
 */
static inline uint32_t SIM_HAL_GetPinCntId(uint32_t baseAddr)
{
    return BR_SIM_SDID_PINID(baseAddr);
}

/*!
 * @brief Gets the Kinetis Revision ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Revision ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Revision ID
 */
static inline uint32_t SIM_HAL_GetRevId(uint32_t baseAddr)
{
    return BR_SIM_SDID_REVID(baseAddr);
}

/*!
 * @brief Gets the Kinetis Die ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Die ID in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis Die ID
 */
static inline uint32_t SIM_HAL_GetDieId(uint32_t baseAddr)
{
    return BR_SIM_SDID_DIEID(baseAddr);
}

/*!
 * @brief Gets the Kinetis SRAM size in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis SRAM Size in System Device ID register.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return id Kinetis SRAM Size
 */
static inline uint32_t SIM_HAL_GetSramSize(uint32_t baseAddr)
{
    return BR_SIM_SDID_SRAMSIZE(baseAddr);
}


/*!
 * @brief Gets the program flash size in  the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  gets the program flash size in the Flash Configuration Register 1.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return size Program flash Size
 */
static inline uint32_t SIM_HAL_GetProgramFlashSize(uint32_t baseAddr)
{
    return BR_SIM_FCFG1_PFSIZE(baseAddr);
}



/*!
 * @brief Sets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  sets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param setting Flash Doze setting
 */
static inline void SIM_HAL_SetFlashDoze(uint32_t baseAddr, uint32_t setting)
{
    BW_SIM_FCFG1_FLASHDOZE(baseAddr, setting);
}

/*!
 * @brief Gets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  gets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return setting Flash Doze setting
 */
static inline uint32_t SIM_HAL_GetFlashDoze(uint32_t baseAddr)
{
    return BR_SIM_FCFG1_FLASHDOZE(baseAddr);
}

/*!
 * @brief Sets the Flash disable setting in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  sets the Flash disable setting in the Flash Configuration Register 1.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @param disable      Flash disable setting
 */
static inline void SIM_HAL_SetFlashDisableCmd(uint32_t baseAddr, bool disable)
{
    BW_SIM_FCFG1_FLASHDIS(baseAddr, disable);
}

/*!
 * @brief Gets the Flash disable setting in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  gets the Flash disable setting in the Flash Configuration Register 1.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return setting Flash disable setting
 */
static inline bool SIM_HAL_GetFlashDisableCmd(uint32_t baseAddr)
{
    return (bool)BR_SIM_FCFG1_FLASHDIS(baseAddr);
}

/*!
 * @brief Gets the Flash maximum address block 0 in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  gets the Flash maximum block 0 in Flash Configuration Register 2.
 *
 * @param baseAddr     Base address for current SIM instance.
 * @return address Flash maximum block 0 address
 */
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock0(uint32_t baseAddr)
{
    return BR_SIM_FCFG2_MAXADDR0(baseAddr);
}





/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/


/*
 * Include the CPU-specific clock API header files.
 */

    #define KL03Z4_SERIES

    /* Clock System Level API header file */
    #include "MKL03Z4/fsl_sim_hal_KL03Z4.h"


#endif /* __FSL_SIM_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

