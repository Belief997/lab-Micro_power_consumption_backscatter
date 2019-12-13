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

#if !defined(__FSL_CLOCK_KL03Z4_H__)
#define __FSL_CLOCK_KL03Z4__H__

/*! @addtogroup clock_manager*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief Gets the clock frequency for PORT module.
 *
 * This function gets the clock frequence for PORT module.
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetPortFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for FTF module. (Flash Memory)
 *
 * This function gets the clock frequence for FTF module. (Flash Memory)
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetFtfFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for ADC module. 
 *
 * This function gets the clock frequence for ADC module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetAdcFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for CMP module. 
 *
 * This function gets the clock frequence for CMP module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetCmpFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for VREF module. 
 *
 * This function gets the clock frequence for VREF module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetVrefFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for TPM module. (FlexTimer)
 *
 * This function gets the clock frequence for TPM module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetTpmFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for SPI module 
 *
 * This function gets the clock frequence for SPI module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetSpiFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for I2C module 
 *
 * This function gets the clock frequence for I2C module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetI2cFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for LPUART module 
 *
 * This function gets the clock frequence for LPUART module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetLpuartFreq(uint32_t instance);

/*!
 * @brief Gets the clock frequency for GPIO module 
 *
 * This function gets the clock frequence for GPIO module. 
 * @param instance module device instance
 * @return freq    clock frequence for this module
 */
uint32_t CLOCK_SYS_GetGpioFreq(uint32_t instance);

/*!
 * @brief Enable the clock for PORT module.
 *
 * This function enables the clock for PORT moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnablePortClock(uint32_t instance)
{
    SIM_HAL_EnablePortClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for PORT module.
 *
 * This function disables the clock for PORT moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisablePortClock(uint32_t instance)
{
    SIM_HAL_DisablePortClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for PORT module.
 *
 * This function will get the clock gate state for PORT moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetPortGateCmd(uint32_t instance)
{
    return SIM_HAL_GetPortGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for FTF module.
 *
 * This function enables the clock for FTF moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableFtfClock(uint32_t instance)
{
    SIM_HAL_EnableFtfClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for FTF module.
 *
 * This function disables the clock for FTF moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableFtfClock(uint32_t instance)
{
    SIM_HAL_DisableFtfClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for FTF module.
 *
 * This function will get the clock gate state for FTF moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetFtfGateCmd(uint32_t instance)
{
    return SIM_HAL_GetFtfGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for ADC module.
 *
 * This function enables the clock for ADC moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableAdcClock(uint32_t instance)
{
    SIM_HAL_EnableAdcClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for ADC module.
 *
 * This function disables the clock for ADC moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableAdcClock(uint32_t instance)
{
    SIM_HAL_DisableAdcClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for ADC module.
 *
 * This function will get the clock gate state for ADC moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetAdcGateCmd(uint32_t instance)
{
    return SIM_HAL_GetAdcGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for CMP module.
 *
 * This function enables the clock for CMP moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableCmpClock(uint32_t instance)
{
    SIM_HAL_EnableCmpClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for CMP module.
 *
 * This function disables the clock for CMP moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableCmpClock(uint32_t instance)
{
    SIM_HAL_DisableCmpClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for CMP module.
 *
 * This function will get the clock gate state for CMP moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetCmpGateCmd(uint32_t instance)
{
    return SIM_HAL_GetCmpGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for VREF module.
 *
 * This function enables the clock for VREF moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableVrefClock(uint32_t instance)
{
    SIM_HAL_EnableVrefClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for VREF module.
 *
 * This function disables the clock for VREF moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableVrefClock(uint32_t instance)
{
    SIM_HAL_DisableVrefClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for VREF module.
 *
 * This function will get the clock gate state for VREF moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetVrefGateCmd(uint32_t instance)
{
    return SIM_HAL_GetVrefGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for TPM module.
 *
 * This function enables the clock for TPM moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableTpmClock(uint32_t instance)
{
    SIM_HAL_EnableTpmClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for TPM module.
 *
 * This function disables the clock for TPM moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableTpmClock(uint32_t instance)
{
    SIM_HAL_DisableTpmClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for TPM module.
 *
 * This function will get the clock gate state for TPM moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetTpmGateCmd(uint32_t instance)
{
    return SIM_HAL_GetTpmGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for LPTIMER module.
 *
 * This function enables the clock for LPTIMER moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableLptimerClock(uint32_t instance)
{
    SIM_HAL_EnableLptimerClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for LPTIMER module.
 *
 * This function disables the clock for LPTIMER moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableLptimerClock(uint32_t instance)
{
    SIM_HAL_DisableLptimerClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for LPTIMER module.
 *
 * This function will get the clock gate state for LPTIMER moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetLptimerGateCmd(uint32_t instance)
{
    return SIM_HAL_GetLptimerGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for RTC module.
 *
 * This function enables the clock for RTC moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableRtcClock(uint32_t instance)
{
    SIM_HAL_EnableRtcClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for RTC module.
 *
 * This function disables the clock for RTC moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableRtcClock(uint32_t instance)
{
    SIM_HAL_DisableRtcClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for RTC module.
 *
 * This function will get the clock gate state for RTC moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetRtcGateCmd(uint32_t instance)
{
    return SIM_HAL_GetRtcGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for SPI module.
 *
 * This function enables the clock for SPI moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableSpiClock(uint32_t instance)
{
    SIM_HAL_EnableSpiClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for SPI module.
 *
 * This function disables the clock for SPI moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableSpiClock(uint32_t instance)
{
    SIM_HAL_DisableSpiClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for SPI module.
 *
 * This function will get the clock gate state for SPI moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetSpiGateCmd(uint32_t instance)
{
    return SIM_HAL_GetSpiGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for I2C module.
 *
 * This function enables the clock for I2C moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableI2cClock(uint32_t instance)
{
    SIM_HAL_EnableI2cClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for I2C module.
 *
 * This function disables the clock for I2C moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableI2cClock(uint32_t instance)
{
    SIM_HAL_DisableI2cClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for I2C module.
 *
 * This function will get the clock gate state for I2C moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetI2cGateCmd(uint32_t instance)
{
    return SIM_HAL_GetI2cGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Enable the clock for LPUART module.
 *
 * This function enables the clock for LPUART moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableLpuartClock(uint32_t instance)
{
    SIM_HAL_EnableLpuartClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Disable the clock for LPUART module.
 *
 * This function disables the clock for LPUART moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableLpuartClock(uint32_t instance)
{
    SIM_HAL_DisableLpuartClock(g_simBaseAddr[0], instance);
}

/*!
 * @brief Get the the clock gate state for LPUART module.
 *
 * This function will get the clock gate state for LPUART moudle.
 * @param instance module device instance
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool CLOCK_SYS_GetLpuartGateCmd(uint32_t instance)
{
    return SIM_HAL_GetLpuartGateCmd(g_simBaseAddr[0], instance);
}

/*!
 * @brief Set the the clock frequency of the RTC_CLKIN external source.
 *
 * This function will set the clock freq of the RTC_CLKIN external sourc3
 * @param frequency external clock frequency.
 * @return clock manager error code.
 */
clock_manager_error_code_t CLOCK_SYS_SetRtcClkInFreq(uint32_t frequency);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_CLOCK_KL03Z4_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

