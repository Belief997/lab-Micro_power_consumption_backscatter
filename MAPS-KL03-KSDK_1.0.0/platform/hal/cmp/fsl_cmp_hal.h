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

#ifndef __FSL_CMP_HAL_H__
#define __FSL_CMP_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"


/*!
 * @addtogroup cmp_hal
 * @{
 */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/*!
 * @brief CMP status return codes.
 */
typedef enum _cmp_status 
{
    kStatus_CMP_Success         = 0U, /*!< Success. */
    kStatus_CMP_InvalidArgument = 1U, /*!< Invalid argument existed. */
    kStatus_CMP_Failed          = 2U  /*!< Execution failed. */
} cmp_status_t;

/*!
 * @brief Defines the hard block hysteresis control level selections.
 * 
 * The hysteresis control level indicates the smallest window between the two
 * inputs when asserting the change of output. See the appropriate
 * Data Sheet for detailed electrical characteristics. Generally, the lower level
 * represents the smaller window.
 */
typedef enum _cmp_hysteresis_mode
{
    kCmpHystersisOfLevel0 = 0U, /*!< Level 0. */
    kCmpHystersisOfLevel1 = 1U, /*!< Level 1. */
    kCmpHystersisOfLevel2 = 2U, /*!< Level 2. */
    kCmpHystersisOfLevel3 = 3U, /*!< Level 3. */
} cmp_hysteresis_mode_t;

/*!
 * @brief Defines the filter sample counter selections.
 * 
 * The selection item represents the number of consecutive samples that must
 * agree prior to the comparator output filter accepting a new output state.
 */
typedef enum _cmp_filter_counter_mode_t
{
    kCmpFilterCountSampleOf0 = 0U, /*!< Disable the filter.  */
    kCmpFilterCountSampleOf1 = 1U, /*!< One sample must agree. */
    kCmpFilterCountSampleOf2 = 2U, /*!< 2 consecutive samples must agree. */
    kCmpFilterCountSampleOf3 = 3U, /*!< 3 consecutive samples must agree. */
    kCmpFilterCountSampleOf4 = 4U, /*!< 4 consecutive samples must agree. */
    kCmpFilterCountSampleOf5 = 5U, /*!< 5 consecutive samples must agree. */
    kCmpFilterCountSampleOf6 = 6U, /*!< 6 consecutive samples must agree. */
    kCmpFilterCountSampleOf7 = 7U  /*!< 7 consecutive samples must agree. */ 
} cmp_filter_counter_mode_t;

/*!
 * @brief Defines the reference voltage source selections for the internal DAC.
 */
typedef enum _cmp_dac_ref_volt_src_mode_t
{
    kCmpDacRefVoltSrcOf1 = 0U, /*!< Vin1 - Vref_out. */
    kCmpDacRefVoltSrcOf2 = 1U  /*!< Vin2 - Vdd. */
} cmp_dac_ref_volt_src_mode_t;

/*!
 * @brief Defines the CMP channel mux selection.
 */
typedef enum _cmp_chn_mux_mode_t
{
    kCmpInputChn0 = 0U, /*!< Comparator input channel 0.  */
    kCmpInputChn1 = 1U, /*!< Comparator input channel 1.  */
    kCmpInputChn2 = 2U, /*!< Comparator input channel 2.  */
    kCmpInputChn3 = 3U, /*!< Comparator input channel 3.  */
    kCmpInputChn4 = 4U, /*!< Comparator input channel 4.  */
    kCmpInputChn5 = 5U, /*!< Comparator input channel 5.  */
    kCmpInputChn6 = 6U, /*!< Comparator input channel 6.  */
    kCmpInputChn7 = 7U, /*!< Comparator input channel 7.  */
    kCmpInputChnDac = kCmpInputChn7 /*!< Comparator input channel 7.  */
} cmp_chn_mux_mode_t;


/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Resets the CMP registers to a known state.
 *
 * This function resets the CMP's registers to a known state. This state is
 * defined in Reference Manual, which is power on reset value.
 *
 * @param baseAddr Register base address for the module.
 */
void CMP_HAL_Init(uint32_t baseAddr);

/*!
 * @brief Sets the filter sample count.
 *
 * This function sets the filter sample count. The value of the filter sample
 * count represents the number of consecutive samples that must agree prior to
 * the comparator output filter accepting a new output state. 
 *
 * @param baseAddr Register base address for the module.
 * @param mode Filter count value mode, see to "cmp_filter_counter_mode_t".
 */
static inline void CMP_HAL_SetFilterCounterMode(uint32_t baseAddr, cmp_filter_counter_mode_t mode)
{
    BW_CMP_CR0_FILTER_CNT(baseAddr, (uint8_t)(mode));
}

/*!
 * @brief Sets the programmable hysteresis level.
 *
 * This function defines the programmable hysteresis level. The hysteresis
 * values associated with each level are device-specific. See the Data Sheet of
 * the device for the exact values. Also, see the "cmp_hysteresis_mode_t" for
 * some additional information.
 *
 * @param baseAddr Register base address for the module.
 * @param mode Hysteresis level, see to "cmp_hysteresis_mode_t".
 */
static inline void CMP_HAL_SetHysteresisMode(uint32_t baseAddr, cmp_hysteresis_mode_t mode)
{
    BW_CMP_CR0_HYSTCTR(baseAddr, (uint8_t)(mode));
}

/*!
 * @brief Enables the comparator in the CMP module.
 *
 * This function enables the comparator in the CMP module. The analog 
 * comparator is the core component in the CMP module. Only when it is enabled, all
 * the other functions for advanced features are meaningful.
 *
 * @param baseAddr Register base address for the module.
 */
static inline void CMP_HAL_Enable(uint32_t baseAddr)
{
    BW_CMP_CR1_EN(baseAddr, 1U);
}

/*!
 * @brief Disables the comparator in the CMP module.
 *
 * This function disables the comparator in CMP module. The analog 
 * comparator is the core component in CMP module. When the it is disabled, it
 * remains in the off state, and consumes no power. 
 *
 * @param baseAddr Register base address for the module.
 */
static inline void CMP_HAL_Disable(uint32_t baseAddr)
{
    BW_CMP_CR1_EN(baseAddr, 0U);
}

/*!
 * @brief Switches to enable the compare output signal connecting to pin.
 *
 * This function switches to enable the compare output signal connecting to
 * pin. The comparator output (CMPO) is driven out on the associated CMPO
 * output pin if the comparator owns the pin. 
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetOutputPinCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_OPE(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Switches to enable the filter for output of compare logic in CMP module.
 *
 * This function switches to enable the filter for output of compare logic
 * in CMP module. When enabled, it sets the unfiltered comparator output
 * (CMPO) to equal COUT. When disabled, it sets the filtered comparator
 * output(CMPO) to equal COUTA.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetUnfilteredOutCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_COS(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Switches to enable the polarity of the analog comparator function.
 *
 * This function switches to enable the polarity of the analog comparator
 * function. When enabled, it  inverts the comparator output logic.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetInvertLogicCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_INV(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Switches to enable the power (speed) comparison mode in CMP module.
 *
 * This function switches to enable the power (speed) comparison mode in
 * CMP module. When enabled, it  selects the High-Speed (HS) comparison
 * mode. In this mode, CMP has faster output propagation delay and higher
 * current consumption.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetHighSpeedCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_PMODE(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Switches to enable the trigger mode in CMP module.
 *
 * This function switches to enable the trigger mode in the CMP module. CMP and
 * internal 6-bit DAC are configured to CMP Trigger mode when this function is
 * enabled. In addition, the CMP should be enabled. If the DAC is to be used as
 * a reference to the CMP, it should also be enabled. CMP Trigger mode depends
 * on an external timer resource to periodically enable the CMP and 6-bit DAC
 * in order to generate a triggered compare. Upon setting trigger mode, the CMP
 * and DAC are placed in a standby state until an external timer resource
 * trigger is received. See the MCU configuration chapter in reference manual
 * for details about the external timer resource.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetTriggerModeCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_TRIGM(baseAddr, (enable ? 1U : 0U) );
}


/*!
 * @brief Switches to enable the sample mode in CMP module.
 *
 * This function switches to enable the sample mode in CMP module.
 * When any sampled mode is active, COUTA is sampled whenever a rising-edge of 
 * filter block clock input or WINDOW/SAMPLE signal is detected.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetSampleModeCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_CR1_SE(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Sets the filter sample period in CMP module.
 *
 * This function sets the filter sample period in CMP module.
 * The setting value specifies the sampling period, in bus clock cycles, of the
 * comparator output filter, when sample mode is disabled. Setting the value to
 * 0x0 disables the filter. This API has no effect when sample mode is
 * enabled. In that case, the external SAMPLE signal is used to determine the
 * sampling period.
 *
 * @param baseAddr Register base address for the module.
 * @param value Count of bus clock cycles for per sample.
 */
static inline void CMP_HAL_SetFilterPeriodValue(uint32_t baseAddr, uint8_t value)
{
    BW_CMP_FPR_FILT_PER(baseAddr, value);
}

/*!
 * @brief Gets the comparator logic output in CMP module.
 *
 * This function gets the comparator logic output in CMP module.
 * It returns the current value of the analog comparator output. The value
 *  is reset to 0 and read as de-assert value when the CMP module is
 * disabled. When setting to invert mode, the comparator logic output  is also
 * inverted.
 *
 * @param baseAddr Register base address for the module.
 * @return The logic output is assert or not.
 */
static inline bool CMP_HAL_GetOutputLogic(uint32_t baseAddr)
{
    return ( 1U == BR_CMP_SCR_COUT(baseAddr) );
}

/*!
 * @brief Gets the logic output falling edge event in the CMP module.
 *
 * This function gets the logic output falling edge event in the CMP module.
 * It detects a falling-edge on COUT and returns the asserted state when the 
 * falling-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @return The falling-edge on COUT has occurred or not.
 */
static inline bool CMP_HAL_GetOutputFallingFlag(uint32_t baseAddr)
{
    return ( 1U == BR_CMP_SCR_CFF(baseAddr) );
}

/*!
 * @brief Clears the logic output falling edge event in the CMP module.
 *
 * This function clears the logic output falling edge event in the CMP module.
 *
 * @param baseAddr Register base address for the module.
 */
static inline void CMP_HAL_ClearOutputFallingFlag(uint32_t baseAddr)
{
    BW_CMP_SCR_CFF(baseAddr, 1U);
}

/*!
 * @brief Gets the logic output rising edge event in the CMP module.
 *
 * This function gets the logic output rising edge event in the CMP module.
 * It detects a rising-edge on COUT and returns the asserted state when the
 * rising-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @return The rising-edge on COUT has occurred or not.
 */
static inline bool CMP_HAL_GetOutputRisingFlag(uint32_t baseAddr)
{
    return ( 1U == BR_CMP_SCR_CFR(baseAddr) );
}

/*!
 * @brief Clears the logic output rising edge event in the CMP module.
 *
 * This function clears the logic output rising edge event in the CMP module.
 *
 * @param baseAddr Register base address for the module.
 */
static inline void CMP_HAL_ClearOutputRisingFlag(uint32_t baseAddr)
{
    BW_CMP_SCR_CFR(baseAddr, 1U);
}

/*!
 * @brief Switches to enable the requesting interrupt when the falling-edge on COUT has occurred.
 *
 * This function switches to enable the requesting interrupt  when the falling-edge
 * on COUT has occurred. When enabled, it generates an interrupt request
 * when falling-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetOutputFallingIntCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_SCR_IEF(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Gets the interrupt request switcher on COUT falling-edge.
 *
 * This function gets the switcher of the interrupt request on COUT
 * falling-edge. When it is asserted, an interrupt request is generated when
 * falling-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @return The status of switcher to enable the feature.
 */
static inline bool CMP_HAL_GetOutputFallingIntCmd(uint32_t baseAddr)
{
    return ( 1U == BR_CMP_SCR_IEF(baseAddr) );
}

/*!
 * @brief Gets the interrupt request switcher on COUT rising-edge.
 *
 * This function gets the switcher of the interrupt request on COUT
 * rising-edge. When it is asserted, an interrupt request is generated when the
 * rising-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @return The status of switcher to enable the feature.
 */
static inline void CMP_HAL_SetOutputRisingIntCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_SCR_IER(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Gets the interrupt request switcher on COUT rising-edge.
 *
 * This function gets the switcher of the interrupt request on COUT
 * rising-edge. When it is asserted, an interrupt request is generated when the
 * rising-edge on COUT has occurred.
 *
 * @param baseAddr Register base address for the module.
 * @return The status of switcher to enable the feature.
 */
static inline bool CMP_HAL_GetOutputRisingIntCmd(uint32_t baseAddr)
{
    return ( 1U == BR_CMP_SCR_IER(baseAddr) );
}


/*!
 * @brief Switches to enable the internal 6-bit DAC in the CMP module.
 *
 * This function switches to enable the internal 6-bit DAC in the CMP module. When
 * enabled, the internal 6-bit DAC can be used as an input channel to the
 * analog comparator.
 *
 * @param baseAddr Register base address for the module.
 * @param enable Switcher to enable the feature.
 */
static inline void CMP_HAL_SetDacCmd(uint32_t baseAddr, bool enable)
{
    BW_CMP_DACCR_DACEN(baseAddr, (enable ? 1U : 0U) );
}

/*!
 * @brief Sets the reference voltage source for the internal 6-bit DAC in the CMP module.
 *
 * This function sets the reference voltage source for the internal 6-bit DAC
 * in the CMP module.
 *
 * @param baseAddr Register base address for the module.
 * @param mode Selection of the feature, see to "cmp_dac_ref_volt_src_mode_t".
 */
static inline void CMP_HAL_SetDacRefVoltSrcMode(uint32_t baseAddr, cmp_dac_ref_volt_src_mode_t mode)
{
    BW_CMP_DACCR_VRSEL(baseAddr, (uint8_t)mode );
}

/*!
 * @brief Sets the output value for the internal 6-bit DAC in the CMP module.
 *
 * This function sets the output value for the internal 6-bit DAC in the CMP module.
 * The output voltage of DAC is DACO = (V in /64) * (value + 1)  and the
 * DACO range is from Vin/64 to Vin.
 *
 * @param baseAddr Register base address for the module.
 * @param value Setting value, 6-bit available.
 */
static inline void CMP_HAL_SetDacValue(uint32_t baseAddr, uint8_t value)
{
    BW_CMP_DACCR_VOSEL(baseAddr, value);
}

/*!
 * @brief Sets the plus channel for the analog comparator.
 *
 * This function sets the plus channel for the analog comparator. 
 * The plus and minus input channels come from the same channel
 * mux. When an inappropriate operation selects the same input for both muxes,
 * the comparator automatically shuts down to prevent becoming a
 * noise generator. For channel use cases, see the appropriate data sheet for each
 * SoC.
 *
 * @param baseAddr Register base address for the module.
 * @param mode Channel mux mode, see to "cmp_chn_mux_mode_t".
 */
static inline void CMP_HAL_SetPlusInputChnMuxMode(uint32_t baseAddr, cmp_chn_mux_mode_t mode)
{
    BW_CMP_MUXCR_PSEL(baseAddr, (uint8_t)(mode) );
}

/*!
 * @brief Sets the minus channel for the analog comparator.
 *
 * This function sets the minus channel for the analog comparator. 
 * The plus and minus input channels  come from the same channel
 * mux. When an inappropriate operation selects the same input for both muxes,
 * the comparator automatically shuts down to prevent becoming a
 * noise generator. For channel use cases, see the appropriate data sheet for each
 * SoC.
 *
 * @param baseAddr Register base address for the module.
 * @param mode Channel mux mode, see to "cmp_chn_mux_mode_t".
 */
static inline void CMP_HAL_SetMinusInputChnMuxMode(uint32_t baseAddr, cmp_chn_mux_mode_t mode)
{
    BW_CMP_MUXCR_MSEL(baseAddr, (uint8_t)(mode) );
}


#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* __FSL_CMP_HAL_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/

