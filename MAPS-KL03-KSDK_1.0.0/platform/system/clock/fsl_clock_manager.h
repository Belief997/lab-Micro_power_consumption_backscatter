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

#if !defined(__FSL_CLOCK_MANAGER_H__)
#define __FSL_CLOCK_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_sim_hal.h"

/*! @addtogroup clock_manager*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* system clocks definition (should be moved to other proper place) */
#define CPU_XTAL1hz_CLK_HZ          1
#define CPU_LPO_CLK_HZ           1000

/* external clock definition (should be moved to other proper place) */

#define SDHC0_CLKIN                 0   /* kSimSDHC0_CLKIN */ 
#define ENET_1588_CLKIN             0   /* kSimENET_1588_CLKIN */
#define EXTAL_Clock                 0   /* kSimEXTAL_Clock */
#define EXTAL1_Clock                0   /* kSimEXTAL1_Clock */
#define USB_CLKIN                   0   /* kSimUSB_CLKIN */

/* Table of base addresses for instances. */
extern const uint32_t g_simBaseAddr[];

/*!
 * @brief Error code definition for the clock manager APIs
 */
typedef enum _clock_manager_error_code {
    kClockManagerSuccess,                           /*!< success */
    kClockManagerNoSuchClockName,                   /*!< cannot find the clock name */
    kClockManagerNoSuchClockModule,                 /*!< cannot find the clock module name */
    kClockManagerNoSuchClockSource,                 /*!< cannot find the clock source name */
    kClockManagerNoSuchDivider,                     /*!< cannot find the divider name */
    kClockManagerUnknown                            /*!< unknown error*/
} clock_manager_error_code_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Clock Frequencies*/
/*@{*/

/*!
 * @brief Gets the clock frequency for a specific clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_names_t.
 * The MCG must be properly configured before using this function. See
 * the reference manual for supported clock names for different chip families.
 * The returned value is in Hertz. If it cannot find the clock name
 * or the name is not supported for a specific chip family, it returns an
 * error.
 *
 * @param clockName Clock names defined in clock_names_t
 * @param frequency Returned clock frequency value in Hertz
 * @return status   Error code defined in clock_manager_error_code_t
 */
clock_manager_error_code_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                                                        uint32_t *frequency);

/*!
 * @brief Sets the clock source setting.
 *
 * This function sets the settings for a specified clock source. Each clock 
 * source has its own clock selection settings. See the chip reference manual for 
 * clock source detailed settings and the sim_clock_source_names_t 
 * for clock sources.
 *
 * @param clockSource Clock source name defined in sim_clock_source_names_t
 * @param setting     Setting value
 * @return status     If the clock source doesn't exist, it returns an error.
 */
clock_manager_error_code_t CLOCK_SYS_SetSource(clock_source_names_t clockSource, 
                                               uint8_t setting);

/*!
 * @brief Gets the clock source setting.
 *
 * This function gets the settings for a specified clock source. Each clock
 * source has its own clock selection settings. See the reference manual for
 * clock source detailed settings and the sim_clock_source_names_t
 * for clock sources.
 *
 * @param clockSource Clock source name
 * @param setting     Current setting for the clock source
 * @return status     If the clock source doesn't exist, it returns an error.
 */
clock_manager_error_code_t CLOCK_SYS_GetSource(clock_source_names_t clockSource, 
                                               uint8_t *setting);

/*!
 * @brief Sets the clock divider setting.
 *
 * This function sets the setting for a specified clock divider. See the
 * reference manual for a supported clock divider and value range and the
 * sim_clock_divider_names_t for dividers.
 *
 * @param clockDivider Clock divider name
 * @param divider      Divider setting
 * @return status      If the clock divider doesn't exist, it  returns an error.
 */
clock_manager_error_code_t CLOCK_SYS_SetDivider(clock_divider_names_t clockDivider, 
                                                uint32_t setting);

/*!
 * @brief Gets the clock divider setting.
 *
 * This function gets the setting for a specified clock divider. See the
 * reference manual for a supported clock divider and value range and the 
 * clock_divider_names_t for dividers.
 *
 * @param clockDivider Clock divider name
 * @param divider      Divider value pointer
 * @return status      If the clock divider doesn't exist, it returns an error.
 */
clock_manager_error_code_t CLOCK_SYS_GetDivider(clock_divider_names_t clockDivider,
                                                uint32_t *setting);

/*!
 * @brief Sets the clock out dividers setting.
 *
 * This function sets the setting for all clock out dividers at the same time.
 * See the reference manual for a supported clock divider and value range and the
 * clock_divider_names_t for clock out dividers.
 *
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
static inline void CLOCK_SYS_SetOutDividers(uint32_t outdiv1, uint32_t outdiv2,
                                            uint32_t outdiv3, uint32_t outdiv4)
{
    CLOCK_HAL_SetOutDividers(g_simBaseAddr[0], outdiv1, outdiv2, outdiv3, outdiv4);
}

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

/*
 * Include the cpu specific clock API header files.
 */

    #define KL03Z4_SERIES

    /* Clock System Level API header file */
    #include "MKL03Z4/fsl_clock_KL03Z4.h"



#endif /* __FSL_CLOCK_MANAGER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

