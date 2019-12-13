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
#ifndef __FSL_PORT_HAL_H__
#define __FSL_PORT_HAL_H__
 
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "fsl_device_registers.h"
 
/*!
 * @addtogroup port_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Internal resistor pull feature selection*/
typedef enum _port_pull {
    kPortPullDown = 0U,  /*!< internal pull-down resistor is enabled.*/
    kPortPullUp   = 1U   /*!< internal pull-up resistor is enabled.*/
} port_pull_t;

/*! @brief Slew rate selection*/
typedef enum _port_slew_rate {
    kPortFastSlewRate = 0U,  /*!< fast slew rate is configured.*/
    kPortSlowSlewRate = 1U   /*!< slow slew rate is configured.*/
} port_slew_rate_t;

/*! @brief Configures the drive strength.*/
typedef enum _port_drive_strength {
    kPortLowDriveStrength  = 0U, /*!< low drive strength is configured.*/
    kPortHighDriveStrength = 1U  /*!< high drive strength is configured.*/
} port_drive_strength_t;

/*! @brief Pin mux selection*/
typedef enum _port_mux {
    kPortPinDisabled = 0U,   /*!< corresponding pin is disabled as analog.*/
    kPortMuxAsGpio   = 1U,   /*!< corresponding pin is configured as GPIO.*/
    kPortMuxAlt2     = 2U,   /*!< chip-specific*/
    kPortMuxAlt3     = 3U,   /*!< chip-specific*/
    kPortMuxAlt4     = 4U,   /*!< chip-specific*/
    kPortMuxAlt5     = 5U,   /*!< chip-specific*/
    kPortMuxAlt6     = 6U,   /*!< chip-specific*/
    kPortMuxAlt7     = 7U    /*!< chip-specific*/
} port_mux_t;

/*! @brief Digital filter clock source selection*/

/*! @brief Configures the interrupt generation condition.*/
typedef enum _port_interrupt_config {
    kPortIntDisabled    = 0x0U,  /*!< Interrupt/DMA request is disabled.*/
    kPortDmaRisingEdge  = 0x1U,  /*!< DMA request on rising edge.*/
    kPortDmaFallingEdge = 0x2U,  /*!< DMA request on falling edge.*/
    kPortDmaEitherEdge  = 0x3U,  /*!< DMA request on either edge.*/
    kPortIntLogicZero   = 0x8U,  /*!< Interrupt when logic zero. */
    kPortIntRisingEdge  = 0x9U,  /*!< Interrupt on rising edge. */
    kPortIntFallingEdge = 0xAU,  /*!< Interrupt on falling edge. */
    kPortIntEitherEdge  = 0xBU,  /*!< Interrupt on either edge. */
    kPortIntLogicOne    = 0xCU   /*!< Interrupt when logic one. */
} port_interrupt_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Selects the internal resistor as pull-down or pull-up.
 * 
 * Pull configuration is valid in all digital pin muxing modes.
 *
 * @param baseAddr  port base address.
 * @param pin       port pin number
 * @param pullSelect  internal resistor pull feature selection
 *        - kPortPullDown: internal pull-down resistor is enabled.
 *        - kPortPullUp  : internal pull-up resistor is enabled.
 */
static inline void PORT_HAL_SetPullMode(uint32_t baseAddr, 
                                        uint32_t pin, 
                                        port_pull_t pullSelect)
{
    assert(pin < 32U);
    BW_PORT_PCRn_PS(baseAddr, pin, pullSelect);
}

/*!
 * @brief Enables or disables the internal pull resistor.
 *
 * @param baseAddr  port base address
 * @param pin       port pin number
 * @param isPullEnabled  internal pull resistor enable or disable
 *        - true : internal pull resistor is enabled.
 *        - false: internal pull resistor is disabled.
 */
static inline void PORT_HAL_SetPullCmd(uint32_t baseAddr, uint32_t pin, bool isPullEnabled)
{
    assert(pin < 32U);    
    BW_PORT_PCRn_PE(baseAddr, pin, isPullEnabled);
}

/*!
 * @brief Configures the fast/slow slew rate if the pin is used as a digital output.
 * 
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @param rateSelect  slew rate selection
 *        - kPortFastSlewRate: fast slew rate is configured.
 *        - kPortSlowSlewRate: slow slew rate is configured.
 */
static inline void PORT_HAL_SetSlewRateMode(uint32_t baseAddr, 
                                                uint32_t pin, 
                                                port_slew_rate_t rateSelect)
{
    assert(pin < 32U);
    BW_PORT_PCRn_SRE(baseAddr, pin, rateSelect);
}

/*!
 * @brief Configures the passive filter if the pin is used as a digital input.
 * 
 * If enabled, a low pass filter (10 MHz to 30 MHz bandwidth)  is enabled
 * on the digital input path. Disable the Passive Input Filter when supporting
 * high speed interfaces (> 2 MHz) on the pin.
 *
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @param isPassiveFilterEnabled  passive filter configuration
 *        - false: passive filter is disabled.
 *        - true : passive filter is enabled.
 */
static inline void PORT_HAL_SetPassiveFilterCmd(uint32_t baseAddr, 
                                                     uint32_t pin, 
                                                     bool isPassiveFilterEnabled)
{
    assert(pin < 32U);
    BW_PORT_PCRn_PFE(baseAddr, pin, isPassiveFilterEnabled);
}


/*!
 * @brief Configures the drive strength if the pin is used as a digital output.
 * 
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @param driveSelect  drive strength selection
 *        - kLowDriveStrength : low drive strength is configured.
 *        - kHighDriveStrength: high drive strength is configured.
 */
static inline void PORT_HAL_SetDriveStrengthMode(uint32_t baseAddr, 
                                                     uint32_t pin, 
                                                     port_drive_strength_t driveSelect)
{
    assert(pin < 32U);
    BW_PORT_PCRn_DSE(baseAddr, pin, driveSelect);
}

/*!
 * @brief Configures the pin muxing.
 * 
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @param mux  pin muxing slot selection
 *        - kPinDisabled: Pin disabled.
 *        - kMuxAsGpio  : Set as GPIO.
 *        - others      : chip-specific.
 */
static inline void PORT_HAL_SetMuxMode(uint32_t baseAddr, uint32_t pin, port_mux_t mux)
{
    assert(pin < 32U);
    BW_PORT_PCRn_MUX(baseAddr, pin, mux);
}
 


/*!
 * @brief Configures the low half of the pin control register for the same settings.
 *        This function operates pin 0 -15 of one specific port.
 * 
 * @param baseAddr  port base address
 * @param lowPinSelect  update corresponding pin control register or not. For a specific bit:
 *        - 0: corresponding low half of pin control register won't be updated according to configuration.
 *        - 1: corresponding low half of pin control register will be updated according to configuration.
 * @param config  value  is written to a low half port control register bits[15:0]. 
 */
void PORT_HAL_SetLowGlobalPinCtrl(uint32_t baseAddr, uint16_t lowPinSelect, uint16_t config);

/*!
 * @brief Configures the high half of pin control register for the same settings.
 *        This function operates pin 16 -31 of one specific port.
 * 
 * @param baseAddr  port base address
 * @param highPinSelect  update corresponding pin control register or not. For a specific bit:
 *        - 0: corresponding high half of pin control register won't be updated according to configuration.
 *        - 1: corresponding high half of pin control register will be updated according to configuration.
 * @param config  value is  written to a high half port control register bits[15:0]. 
 */
void PORT_HAL_SetHighGlobalPinCtrl(uint32_t baseAddr, uint16_t highPinSelect, uint16_t config);

/*@}*/

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Configures the port pin interrupt/DMA request.
 * 
 * @param baseAddr  port base address.
 * @param pin  port pin number
 * @param intConfig  interrupt configuration
 *        - kIntDisabled   : Interrupt/DMA request disabled.
 *        - kDmaRisingEdge : DMA request on rising edge.
 *        - kDmaFallingEdge: DMA request on falling edge.
 *        - kDmaEitherEdge : DMA request on either edge.
 *        - KIntLogicZero  : Interrupt when logic zero. 
 *        - KIntRisingEdge : Interrupt on rising edge. 
 *        - KIntFallingEdge: Interrupt on falling edge. 
 *        - KIntEitherEdge : Interrupt on either edge. 
 *        - KIntLogicOne   : Interrupt when logic one. 
 */
static inline void PORT_HAL_SetPinIntMode(uint32_t baseAddr, 
                                          uint32_t pin, 
                                          port_interrupt_config_t intConfig)
{
    assert(pin < 32U);
    BW_PORT_PCRn_IRQC(baseAddr, pin, intConfig);
}

/*!
 * @brief Gets the current port pin interrupt/DMA request configuration.
 * 
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @return  interrupt configuration
 *        - kIntDisabled   : Interrupt/DMA request disabled.
 *        - kDmaRisingEdge : DMA request on rising edge.
 *        - kDmaFallingEdge: DMA request on falling edge.
 *        - kDmaEitherEdge : DMA request on either edge.
 *        - KIntLogicZero  : Interrupt when logic zero. 
 *        - KIntRisingEdge : Interrupt on rising edge. 
 *        - KIntFallingEdge: Interrupt on falling edge. 
 *        - KIntEitherEdge : Interrupt on either edge. 
 *        - KIntLogicOne   : Interrupt when logic one. 
 */
static inline port_interrupt_config_t PORT_HAL_GetPinIntMode(uint32_t baseAddr, uint32_t pin)
{
    assert(pin < 32U);
    return (port_interrupt_config_t)BR_PORT_PCRn_IRQC(baseAddr, pin);
}

/*!
 * @brief Reads the individual pin-interrupt status flag.
 * 
 * If a pin is configured to generate the DMA request,  the corresponding flag
 * is cleared automatically at the completion of the requested DMA transfer.
 * Otherwise, the flag remains set until a logic one is written to that flag. 
 * If configured for a level sensitive interrupt that remains asserted, the flag
 * is set again immediately.
 *
 * @param baseAddr  port base address
 * @param pin  port pin number
 * @return current pin interrupt status flag
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 */
static inline bool PORT_HAL_IsPinIntPending(uint32_t baseAddr, uint32_t pin)
{
    assert(pin < 32U);    
    return BR_PORT_PCRn_ISF(baseAddr, pin);
}

/*!
 * @brief Clears the individual pin-interrupt status flag.
 * 
 * @param baseAddr  port base address
 * @param pin  port pin number
 */
static inline void PORT_HAL_ClearPinIntFlag(uint32_t baseAddr, uint32_t pin)
{
    assert(pin < 32U);    
    BW_PORT_PCRn_ISF(baseAddr, pin, 1U);
}

/*!
 * @brief Reads the entire port interrupt status flag.
 * 
 * @param baseAddr  port base address
 * @return all 32 pin interrupt status flags. For specific bit:
 *         - 0: interrupt is not detected.
 *         - 1: interrupt is detected.
 */
static inline uint32_t PORT_HAL_GetPortIntFlag(uint32_t baseAddr)
{
    return HW_PORT_ISFR_RD(baseAddr);
}

/*!
 * @brief Clears the entire port interrupt status flag.
 * 
 * @param baseAddr  port base address
 */
static inline void PORT_HAL_ClearPortIntFlag(uint32_t baseAddr)
{
    HW_PORT_ISFR_WR(baseAddr, ~0U);
}

/*@}*/

#if defined(__cplusplus)
}
#endif
 
/*! @}*/
 
#endif /* __FSL_PORT_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

