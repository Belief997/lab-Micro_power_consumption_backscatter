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

#ifndef __FSL_CMP_DRIVER_H__
#define __FSL_CMP_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_cmp_hal.h"

/*!
 * @addtogroup cmp_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Defines the structure to configure the comparator in the CMP module.
 *
 * This type of structure keeps the configuration for the comparator
 * inside the CMP module. With the configuration, the CMP can be set as a
 * normal comparator without additional features. 
 */
typedef struct CmpUserConfig
{
    cmp_hysteresis_mode_t hysteresisMode; /*!< Set the hysteresis level. */
    bool pinoutEnable; /*!< Enable to output the CMPO to pin. */
    bool pinoutUnfilteredEnable; /*!< Enable to output unfiltered result to CMPO. */
    bool invertEnable; /*!< Enable to invert the comparator's result. */
    bool highSpeedEnable; /*!< Enable to work in speed mode. */
    bool risingIntEnable; /*!< Enable to use CMPO rising interrupt. */
    bool fallingIntEnable; /*!< Enable to use CMPO falling interrupt. */
    cmp_chn_mux_mode_t plusChnMux; /*!< Set the Plus side input to comparator. */
    cmp_chn_mux_mode_t minusChnMux; /*!< Set the Minus side input to comparator. */
    bool triggerEnable; /*!< Enable trigger mode.  */
} cmp_user_config_t;

/*!
* @brief Defines sample and filter mode selections in the CMP module.
*
* The comparator sample/filter is available in several modes. Use the enumeration
* to identify the comparator's condition:
*
* kCmpContinuousMode - Continuous Mode:
        Both window control and filter blocks are completely bypassed. The
        output of the comparator is updated continuously. 
* kCmpSampleWithNoFilteredMode - Sample, Non-Filtered Mode:
        Window control is completely bypassed. The output of the comparator is
        sampled whenever a rising-edge is detected on the filter block clock
        input. The filter clock prescaler can be configured as the
        divider from the bus clock.
* kCmpSampleWithFilteredMode - Sample, Filtered Mode:
        Similar to "Sample, Non-Filtered Mode", but the filter is active in
        this mode. The filter counter value becomes
        configurable as well.
* kCmpWindowedMode - Windowed Mode:
        In Window Mode, only the output of the analog comparator is passed when
        the Window signal is high. The last latched value is held when the Window
        signal is low.
* kCmpWindowedFilteredMode - Window/Filtered Mode:
        This mode complex and uses both window and filtering
        features. It also has the highest latency of all modes. This can be 
        approximated as follows: up to 1 bus clock synchronization in the window function
        + ( ( filter counter * filter prescaler ) + 1) bus clock for the
        filter function.
*/
typedef enum _cmp_sample_filter_mode
{
    kCmpContinuousMode = 0U, /*!< Continuous Mode.*/
    kCmpSampleWithNoFilteredMode = 1U, /*!< Sample, Non-Filtered Mode. */
    kCmpSampleWithFilteredMode = 2U, /*!< Sample, Filtered Mode. */
    kCmpWindowedMode = 3U, /*!< Windowed Mode. */
    kCmpWindowedFilteredMode = 4U /*!< Window/Filtered Mode. */
} cmp_sample_filter_mode_t;

/*!
* @brief Define structure of configuring Window/Filter in CMP module.
*
* This type of structure keeps the configuration for Window/Filter inside
* the CMP module. With the configuration, the CMP module can work in an
* advanced mode.
*/
typedef struct CmpSampleFilterConfig
{
    cmp_sample_filter_mode_t workMode; /*!< Sample/Filter's work mode. */
    bool useExtSampleOrWindow; /*!< Switcher to use external WINDOW/SAMPLE signal. */
    uint8_t filterClkDiv; /*!< Filter's prescaler which divides from the bus clock.  */
    cmp_filter_counter_mode_t filterCount; /*!< Sample count for filter, see to "cmp_filter_counter_mode_t". */
} cmp_sample_filter_config_t;

/*!
 * @brief Define structure of configuring internal DAC in CMP module.
 *
 * This type of structure is to keep the configuration for DAC
 * inside the CMP module. With the configuration, the internal DAC would
 * provide a reference voltage level, it is chosen as the input of CMP.
 */
typedef struct CmpDacConfig
{
    cmp_dac_ref_volt_src_mode_t refVoltSrcMode; /*!< Select the reference voltage source for internal DAC. */
    uint8_t dacValue; /*!< Set the value for internal DAC. */
} cmp_dac_config_t;

/*!
 * @brief Define type of flags for CMP event in CMP module.
 */
typedef enum _cmp_flag
{
    kCmpFlagOfCoutRising  = 0U, /*!< Identifier to indicate if the COUT change from logic zero to one. */
    kCmpFlagOfCoutFalling = 1U  /*!< Identifier to indicate if the COUT change from logic one to zero. */
} cmp_flag_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Define type of user-defined callback function in CMP module.
 */
typedef void (*cmp_callback_t)(void);

/*!
 * @brief Internal driver state information.
 *
 * The contents of this structure are internal to the driver and should not be
 *  modified by users. Also, contents of the structure are subject to change in
 *  future releases.
 */
typedef struct CmpState
{
    bool isInUsed; /* If the CMP instance is in used. All the CMP instances share
        * the same clock gate. They would be aligned to use the clock.*/
    cmp_callback_t userCallbackFunc;
    /* Set user-defined callback function. This function is executed
        * if CMP interrupt occurs.*/
} cmp_state_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*!
 * @brief Fills the initial user configuration for a default setting. 
 *
 * This function fills the initial user configuration for a default setting. 
 * The default setting makes the CMP module to be a comparator.
 * It includes the setting of :
 *     .hysteresisMode = kCmpHystersisOfLevel0
 *     .pinoutEnable = true
 *     .pinoutUnfilteredEnable = true
 *     .invertEnable = false
 *     .highSpeedEnable = false
 *     .dmaEnable = false
 *     .risingIntEnable = false
 *     .fallingIntEnable = false
 *     .triggerEnable = false
 * However, it is still recommended to fill some fields of the structure, such as
 * channel mux, according to application. Note that this API does not set the
 * configuration to hardware.
 *
 * @param userConfigPtr Pointer to structure of configuration, see to "cmp_user_config_t".
 * @param plusInput Plus Input mux selection, see to "cmp_chn_mux_mode_t".
 * @param minusInput Minus Input mux selection, see to "cmp_chn_mux_mode_t".
 * @return Execution status.
 */
cmp_status_t CMP_DRV_StructInitUserConfigDefault(cmp_user_config_t *userConfigPtr,
    cmp_chn_mux_mode_t plusInput, cmp_chn_mux_mode_t minusInput);

/*!
 * @brief Initializes the CMP module. 
 *
 * This function  initializes the CMP module. It  enables the clock and
 * sets the interrupt switcher for CMP module. The CMP module is
 * configured for the basic comparator.
 *
 * @param instance CMP instance id.
 * @param userConfigPtr Pointer to structure of configuration, see to "cmp_user_config_t".
 * @param userStatePtr Pointer to structure of context, see to "cmp_state_t".
 * @return Execution status.
 */
cmp_status_t CMP_DRV_Init(uint32_t instance, cmp_user_config_t *userConfigPtr,
    cmp_state_t *userStatePtr);

/*!
 * @brief De-initializes the CMP module. 
 *
 * This function de-initializes the CMP module. It shuts down the CMP's
 * clock and disables the interrupt. This API should be called when CMP is no
 * longer used in application as it helps to reduce the power consumption.
 *
 * @param instance CMP instance id.
 */
void CMP_DRV_Deinit(uint32_t instance);

/*!
 * @brief Starts the CMP module. 
 *
 * This function starts the CMP module. The configuration does not take
 * effect until the module is started.
 *
 * @param instance CMP instance id.
 */
void CMP_DRV_Start(uint32_t instance);

/*!
 * @brief Stops the CMP module. 
 *
 * This function stops the CMP module. Note that this API slowly brings the features to a stop.
 *
 * @param instance CMP instance id.
 */
void CMP_DRV_Stop(uint32_t instance);

/*!
 * @brief Enables the internal DAC in CMP module. 
 *
 * This function enables the internal DAC in CMP module. It takes
 * effect only when the internal DAC is been chosen as one of the input
 * channels for the comparator. Then, the DAC channel can be programmed to provide
 * a reference voltage level.
 *
 * @param instance CMP instance id.
 * @param dacConfigPtr Pointer to structure of configuration, see to "cmp_dac_config_t".
 * @return Execution status.
 */
cmp_status_t CMP_DRV_EnableDac(uint32_t instance, cmp_dac_config_t *dacConfigPtr);

/*!
 * @brief Disables the internal DAC in the CMP module. 
 *
 * This function disables the internal DAC in CMP module. It should be
 * called if the internal DAC is no longer used in application.
 *
 * @param instance CMP instance id.
 */
void CMP_DRV_DisableDac(uint32_t instance);

/*!
 * @brief Configures the Sample\Filter feature in CMP module. 
 *
 * This function configures the CMP working in Sample\Filter modes. These
 * modes are advanced features in addition to the basic comparator. They may
 * be about Windowed Mode, Filter Mode and so on. See the 
 * "cmp_sample_filter_config_t" for more information.
 *
 * @param instance CMP instance id.
 * @param cmp_sample_filter_config_t Pointer to structure of configuration.
 *        see to "cmp_sample_filter_config_t".
 * @return Execution status.
 */
cmp_status_t CMP_DRV_ConfigSampleFilter(uint32_t instance, cmp_sample_filter_config_t *configPtr);

/*!
 * @brief Gets output of CMP module. 
 *
 * This function gets the output of CMP module.
 * The output source depends on the configuration when initializing the comparator.
 * When cmp_user_config_t.pinoutUnfilteredEnable = false, the output is
 * processed by a filter. Otherwise, the output is that the signal did not pass
 * the filter.
 *
 * @param instance CMP instance id.
 * @return Output logic's assertion. When there is no invert, plus side > minus side, it will be true.
 */
bool CMP_DRV_GetOutput(uint32_t instance);

/*!
 * @brief Gets a state of CMP module. 
 *
 * This function gets the state of CMP module. It  returns if the indicated
 * event has been detected.
 *
 * @param instance CMP instance id.
 * @param flag Represent events or states, see to "cmp_flag_t".
 * @return Assertion if indicated event occurs.
 */
bool CMP_DRV_GetFlag(uint32_t instance, cmp_flag_t flag);

/*!
 * @brief Clears event record of CMP module. 
 *
 * This function clears the event record of CMP module. 
 *
 * @param instance CMP instance ID.
 * @param flag Represent events or states, see to "cmp_flag_t".
 */
void CMP_DRV_ClearFlag(uint32_t instance, cmp_flag_t flag);

/*!
 * @brief Installs the user-defined callback in CMP module. 
 *
 * This function installs the user-defined callback in CMP module.
 * When an CMP interrupt request is served, the callback is executed 
 * inside the ISR. 
 *
 * @param instance CMPinstance id.
 * @param userCallback User-defined callback function.
 */
cmp_status_t CMP_DRV_InstallCallback(uint32_t instance, cmp_callback_t userCallback);

/*!
 * @brief Driver-defined ISR in CMP module. 
 *
 * This function is the driver-defined ISR in CMP module. 
 * It includes the process for interrupt mode defined by driver. Currently, it
 * is called inside the system-defined ISR.
 *
 * @param instance CMP instance id.
 */
void CMP_DRV_IRQHandler(uint32_t instance);

#if defined(__cplusplus)
extern }
#endif

/*!
 *@}
 */

#endif /* __FSL_CMP_DRIVER_H__ */
/******************************************************************************
 * EOF
 *****************************************************************************/

