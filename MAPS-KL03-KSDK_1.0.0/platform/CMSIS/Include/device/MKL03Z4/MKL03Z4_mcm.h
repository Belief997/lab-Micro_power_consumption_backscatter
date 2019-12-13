/*
** ###################################################################
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    KL03P24M48SF0RM, Rev 2, Apr 2014
**     Version:             rev. 1.3, 2014-06-27
**     Build:               b140715
**
**     Abstract:
**         Extension to the CMSIS register access layer header.
**
**     Copyright (c) 2014 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2013-12-11)
**         Initial version.
**     - rev. 1.1 (2014-04-16)
**         Update of the I2C module (SMBUS feature).
**         Update of the MCG_Light module.
**         Added register file system (RFSYS).
**     - rev. 1.2 (2014-04-30)
**         PEx compatibility macros has been added.
**     - rev. 1.3 (2014-06-27)
**         I2C_S1 register was renamed.
**         GPIO - Modules PTA,PTB renamed to GPIOA,GPIOB.
**
** ###################################################################
*/

/*
 * WARNING! DO NOT EDIT THIS FILE DIRECTLY!
 *
 * This file was generated automatically and any changes may be lost.
 */
#ifndef __HW_MCM_REGISTERS_H__
#define __HW_MCM_REGISTERS_H__

#include "MKL03Z4.h"
#include "fsl_bitaccess.h"

/*
 * MKL03Z4 MCM
 *
 * Core Platform Miscellaneous Control Module
 *
 * Registers defined in this header file:
 * - HW_MCM_PLASC - Crossbar Switch (AXBS) Slave Configuration
 * - HW_MCM_PLAMC - Crossbar Switch (AXBS) Master Configuration
 * - HW_MCM_PLACR - Platform Control Register
 * - HW_MCM_CPO - Compute Operation Control Register
 *
 * - hw_mcm_t - Struct containing all module registers.
 */

#define HW_MCM_INSTANCE_COUNT (1U) /*!< Number of instances of the MCM module. */

/*******************************************************************************
 * HW_MCM_PLASC - Crossbar Switch (AXBS) Slave Configuration
 ******************************************************************************/

/*!
 * @brief HW_MCM_PLASC - Crossbar Switch (AXBS) Slave Configuration (RO)
 *
 * Reset value: 0x0007U
 *
 * PLASC is a 16-bit read-only register identifying the presence/absence of bus
 * slave connections to the device's crossbar switch.
 */
typedef union _hw_mcm_plasc
{
    uint16_t U;
    struct _hw_mcm_plasc_bitfields
    {
        uint16_t ASC : 8;              /*!< [7:0] Each bit in the ASC field indicates
                                        * whether there is a corresponding connection to the crossbar switch's slave
                                        * input port. */
        uint16_t RESERVED0 : 8;        /*!< [15:8]  */
    } B;
} hw_mcm_plasc_t;

/*!
 * @name Constants and macros for entire MCM_PLASC register
 */
/*@{*/
#define HW_MCM_PLASC_ADDR(x)     ((x) + 0x8U)

#define HW_MCM_PLASC(x)          (*(__I hw_mcm_plasc_t *) HW_MCM_PLASC_ADDR(x))
#define HW_MCM_PLASC_RD(x)       (HW_MCM_PLASC(x).U)
/*@}*/

/*
 * Constants & macros for individual MCM_PLASC bitfields
 */

/*!
 * @name Register MCM_PLASC, field ASC[7:0] (RO)
 *
 * Values:
 * - 0 - A bus slave connection to AXBS input port n is absent.
 * - 1 - A bus slave connection to AXBS input port n is present.
 */
/*@{*/
#define BP_MCM_PLASC_ASC     (0U)          /*!< Bit position for MCM_PLASC_ASC. */
#define BM_MCM_PLASC_ASC     (0x00FFU)     /*!< Bit mask for MCM_PLASC_ASC. */
#define BS_MCM_PLASC_ASC     (8U)          /*!< Bit field size in bits for MCM_PLASC_ASC. */

/*! @brief Read current value of the MCM_PLASC_ASC field. */
#define BR_MCM_PLASC_ASC(x)  (HW_MCM_PLASC(x).B.ASC)
/*@}*/

/*******************************************************************************
 * HW_MCM_PLAMC - Crossbar Switch (AXBS) Master Configuration
 ******************************************************************************/

/*!
 * @brief HW_MCM_PLAMC - Crossbar Switch (AXBS) Master Configuration (RO)
 *
 * Reset value: 0x0001U
 *
 * PLAMC is a 16-bit read-only register identifying the presence/absence of bus
 * master connections to the device's crossbar switch.
 */
typedef union _hw_mcm_plamc
{
    uint16_t U;
    struct _hw_mcm_plamc_bitfields
    {
        uint16_t AMC : 8;              /*!< [7:0] Each bit in the AMC field indicates
                                        * whether there is a corresponding connection to the AXBS master input port. */
        uint16_t RESERVED0 : 8;        /*!< [15:8]  */
    } B;
} hw_mcm_plamc_t;

/*!
 * @name Constants and macros for entire MCM_PLAMC register
 */
/*@{*/
#define HW_MCM_PLAMC_ADDR(x)     ((x) + 0xAU)

#define HW_MCM_PLAMC(x)          (*(__I hw_mcm_plamc_t *) HW_MCM_PLAMC_ADDR(x))
#define HW_MCM_PLAMC_RD(x)       (HW_MCM_PLAMC(x).U)
/*@}*/

/*
 * Constants & macros for individual MCM_PLAMC bitfields
 */

/*!
 * @name Register MCM_PLAMC, field AMC[7:0] (RO)
 *
 * Values:
 * - 0 - A bus master connection to AXBS input port n is absent
 * - 1 - A bus master connection to AXBS input port n is present
 */
/*@{*/
#define BP_MCM_PLAMC_AMC     (0U)          /*!< Bit position for MCM_PLAMC_AMC. */
#define BM_MCM_PLAMC_AMC     (0x00FFU)     /*!< Bit mask for MCM_PLAMC_AMC. */
#define BS_MCM_PLAMC_AMC     (8U)          /*!< Bit field size in bits for MCM_PLAMC_AMC. */

/*! @brief Read current value of the MCM_PLAMC_AMC field. */
#define BR_MCM_PLAMC_AMC(x)  (HW_MCM_PLAMC(x).B.AMC)
/*@}*/

/*******************************************************************************
 * HW_MCM_PLACR - Platform Control Register
 ******************************************************************************/

/*!
 * @brief HW_MCM_PLACR - Platform Control Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The speculation buffer in the flash memory controller is configurable via
 * PLACR[15: 14]. The speculation buffer is enabled only for instructions after
 * reset. It is possible to have these states for the speculation buffer: DFCS EFDS
 * Description 0 0 Speculation buffer is on for instruction and off for data. 0 1
 * Speculation buffer is on for instruction and on for data. 1 X Speculation
 * buffer is off.
 */
typedef union _hw_mcm_placr
{
    uint32_t U;
    struct _hw_mcm_placr_bitfields
    {
        uint32_t RESERVED0 : 14;       /*!< [13:0]  */
        uint32_t EFDS : 1;             /*!< [14] Enable Flash Data Speculation */
        uint32_t DFCS : 1;             /*!< [15] Disable Flash Controller Speculation */
        uint32_t ESFC : 1;             /*!< [16] Enable Stalling Flash Controller */
        uint32_t RESERVED1 : 15;       /*!< [31:17]  */
    } B;
} hw_mcm_placr_t;

/*!
 * @name Constants and macros for entire MCM_PLACR register
 */
/*@{*/
#define HW_MCM_PLACR_ADDR(x)     ((x) + 0xCU)

#define HW_MCM_PLACR(x)          (*(__IO hw_mcm_placr_t *) HW_MCM_PLACR_ADDR(x))
#define HW_MCM_PLACR_RD(x)       (HW_MCM_PLACR(x).U)
#define HW_MCM_PLACR_WR(x, v)    (HW_MCM_PLACR(x).U = (v))
#define HW_MCM_PLACR_SET(x, v)   (HW_MCM_PLACR_WR(x, HW_MCM_PLACR_RD(x) |  (v)))
#define HW_MCM_PLACR_CLR(x, v)   (HW_MCM_PLACR_WR(x, HW_MCM_PLACR_RD(x) & ~(v)))
#define HW_MCM_PLACR_TOG(x, v)   (HW_MCM_PLACR_WR(x, HW_MCM_PLACR_RD(x) ^  (v)))
/*@}*/

/*
 * Constants & macros for individual MCM_PLACR bitfields
 */

/*!
 * @name Register MCM_PLACR, field EFDS[14] (RW)
 *
 * Enables flash data speculation.
 *
 * Values:
 * - 0 - Disable flash data speculation.
 * - 1 - Enable flash data speculation.
 */
/*@{*/
#define BP_MCM_PLACR_EFDS    (14U)         /*!< Bit position for MCM_PLACR_EFDS. */
#define BM_MCM_PLACR_EFDS    (0x00004000U) /*!< Bit mask for MCM_PLACR_EFDS. */
#define BS_MCM_PLACR_EFDS    (1U)          /*!< Bit field size in bits for MCM_PLACR_EFDS. */

/*! @brief Read current value of the MCM_PLACR_EFDS field. */
#define BR_MCM_PLACR_EFDS(x) (HW_MCM_PLACR(x).B.EFDS)

/*! @brief Format value for bitfield MCM_PLACR_EFDS. */
#define BF_MCM_PLACR_EFDS(v) ((uint32_t)((uint32_t)(v) << BP_MCM_PLACR_EFDS) & BM_MCM_PLACR_EFDS)

/*! @brief Set the EFDS field to a new value. */
#define BW_MCM_PLACR_EFDS(x, v) (HW_MCM_PLACR_WR(x, (HW_MCM_PLACR_RD(x) & ~BM_MCM_PLACR_EFDS) | BF_MCM_PLACR_EFDS(v)))
/*@}*/

/*!
 * @name Register MCM_PLACR, field DFCS[15] (RW)
 *
 * Disables flash controller speculation.
 *
 * Values:
 * - 0 - Enable flash controller speculation.
 * - 1 - Disable flash controller speculation.
 */
/*@{*/
#define BP_MCM_PLACR_DFCS    (15U)         /*!< Bit position for MCM_PLACR_DFCS. */
#define BM_MCM_PLACR_DFCS    (0x00008000U) /*!< Bit mask for MCM_PLACR_DFCS. */
#define BS_MCM_PLACR_DFCS    (1U)          /*!< Bit field size in bits for MCM_PLACR_DFCS. */

/*! @brief Read current value of the MCM_PLACR_DFCS field. */
#define BR_MCM_PLACR_DFCS(x) (HW_MCM_PLACR(x).B.DFCS)

/*! @brief Format value for bitfield MCM_PLACR_DFCS. */
#define BF_MCM_PLACR_DFCS(v) ((uint32_t)((uint32_t)(v) << BP_MCM_PLACR_DFCS) & BM_MCM_PLACR_DFCS)

/*! @brief Set the DFCS field to a new value. */
#define BW_MCM_PLACR_DFCS(x, v) (HW_MCM_PLACR_WR(x, (HW_MCM_PLACR_RD(x) & ~BM_MCM_PLACR_DFCS) | BF_MCM_PLACR_DFCS(v)))
/*@}*/

/*!
 * @name Register MCM_PLACR, field ESFC[16] (RW)
 *
 * Enables stalling flash controller when flash is busy. When software needs to
 * access the flash memory while a flash memory resource is being manipulated by
 * a flash command, software can enable a stall mechanism to avoid a read
 * collision. The stall mechanism allows software to execute code from the same block on
 * which flash operations are being performed. However, software must ensure the
 * sector the flash operations are being performed on is not the same sector
 * from which the code is executing. ESFC enables the stall mechanism. This bit must
 * be set only just before the flash operation is executed and must be cleared
 * when the operation completes.
 *
 * Values:
 * - 0 - Disable stalling flash controller when flash is busy.
 * - 1 - Enable stalling flash controller when flash is busy.
 */
/*@{*/
#define BP_MCM_PLACR_ESFC    (16U)         /*!< Bit position for MCM_PLACR_ESFC. */
#define BM_MCM_PLACR_ESFC    (0x00010000U) /*!< Bit mask for MCM_PLACR_ESFC. */
#define BS_MCM_PLACR_ESFC    (1U)          /*!< Bit field size in bits for MCM_PLACR_ESFC. */

/*! @brief Read current value of the MCM_PLACR_ESFC field. */
#define BR_MCM_PLACR_ESFC(x) (HW_MCM_PLACR(x).B.ESFC)

/*! @brief Format value for bitfield MCM_PLACR_ESFC. */
#define BF_MCM_PLACR_ESFC(v) ((uint32_t)((uint32_t)(v) << BP_MCM_PLACR_ESFC) & BM_MCM_PLACR_ESFC)

/*! @brief Set the ESFC field to a new value. */
#define BW_MCM_PLACR_ESFC(x, v) (HW_MCM_PLACR_WR(x, (HW_MCM_PLACR_RD(x) & ~BM_MCM_PLACR_ESFC) | BF_MCM_PLACR_ESFC(v)))
/*@}*/

/*******************************************************************************
 * HW_MCM_CPO - Compute Operation Control Register
 ******************************************************************************/

/*!
 * @brief HW_MCM_CPO - Compute Operation Control Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register controls the Compute Operation.
 */
typedef union _hw_mcm_cpo
{
    uint32_t U;
    struct _hw_mcm_cpo_bitfields
    {
        uint32_t CPOREQ : 1;           /*!< [0] Compute Operation Request */
        uint32_t CPOACK : 1;           /*!< [1] Compute Operation Acknowledge */
        uint32_t CPOWOI : 1;           /*!< [2] Compute Operation Wake-up on Interrupt
                                        * */
        uint32_t RESERVED0 : 29;       /*!< [31:3]  */
    } B;
} hw_mcm_cpo_t;

/*!
 * @name Constants and macros for entire MCM_CPO register
 */
/*@{*/
#define HW_MCM_CPO_ADDR(x)       ((x) + 0x40U)

#define HW_MCM_CPO(x)            (*(__IO hw_mcm_cpo_t *) HW_MCM_CPO_ADDR(x))
#define HW_MCM_CPO_RD(x)         (HW_MCM_CPO(x).U)
#define HW_MCM_CPO_WR(x, v)      (HW_MCM_CPO(x).U = (v))
#define HW_MCM_CPO_SET(x, v)     (HW_MCM_CPO_WR(x, HW_MCM_CPO_RD(x) |  (v)))
#define HW_MCM_CPO_CLR(x, v)     (HW_MCM_CPO_WR(x, HW_MCM_CPO_RD(x) & ~(v)))
#define HW_MCM_CPO_TOG(x, v)     (HW_MCM_CPO_WR(x, HW_MCM_CPO_RD(x) ^  (v)))
/*@}*/

/*
 * Constants & macros for individual MCM_CPO bitfields
 */

/*!
 * @name Register MCM_CPO, field CPOREQ[0] (RW)
 *
 * This bit is auto-cleared by vector fetching if CPOWOI = 1.
 *
 * Values:
 * - 0 - Request is cleared.
 * - 1 - Request Compute Operation.
 */
/*@{*/
#define BP_MCM_CPO_CPOREQ    (0U)          /*!< Bit position for MCM_CPO_CPOREQ. */
#define BM_MCM_CPO_CPOREQ    (0x00000001U) /*!< Bit mask for MCM_CPO_CPOREQ. */
#define BS_MCM_CPO_CPOREQ    (1U)          /*!< Bit field size in bits for MCM_CPO_CPOREQ. */

/*! @brief Read current value of the MCM_CPO_CPOREQ field. */
#define BR_MCM_CPO_CPOREQ(x) (HW_MCM_CPO(x).B.CPOREQ)

/*! @brief Format value for bitfield MCM_CPO_CPOREQ. */
#define BF_MCM_CPO_CPOREQ(v) ((uint32_t)((uint32_t)(v) << BP_MCM_CPO_CPOREQ) & BM_MCM_CPO_CPOREQ)

/*! @brief Set the CPOREQ field to a new value. */
#define BW_MCM_CPO_CPOREQ(x, v) (HW_MCM_CPO_WR(x, (HW_MCM_CPO_RD(x) & ~BM_MCM_CPO_CPOREQ) | BF_MCM_CPO_CPOREQ(v)))
/*@}*/

/*!
 * @name Register MCM_CPO, field CPOACK[1] (RO)
 *
 * Values:
 * - 0 - Compute operation entry has not completed or compute operation exit has
 *     completed.
 * - 1 - Compute operation entry has completed or compute operation exit has not
 *     completed.
 */
/*@{*/
#define BP_MCM_CPO_CPOACK    (1U)          /*!< Bit position for MCM_CPO_CPOACK. */
#define BM_MCM_CPO_CPOACK    (0x00000002U) /*!< Bit mask for MCM_CPO_CPOACK. */
#define BS_MCM_CPO_CPOACK    (1U)          /*!< Bit field size in bits for MCM_CPO_CPOACK. */

/*! @brief Read current value of the MCM_CPO_CPOACK field. */
#define BR_MCM_CPO_CPOACK(x) (HW_MCM_CPO(x).B.CPOACK)
/*@}*/

/*!
 * @name Register MCM_CPO, field CPOWOI[2] (RW)
 *
 * Values:
 * - 0 - No effect.
 * - 1 - When set, the CPOREQ is cleared on any interrupt or exception vector
 *     fetch.
 */
/*@{*/
#define BP_MCM_CPO_CPOWOI    (2U)          /*!< Bit position for MCM_CPO_CPOWOI. */
#define BM_MCM_CPO_CPOWOI    (0x00000004U) /*!< Bit mask for MCM_CPO_CPOWOI. */
#define BS_MCM_CPO_CPOWOI    (1U)          /*!< Bit field size in bits for MCM_CPO_CPOWOI. */

/*! @brief Read current value of the MCM_CPO_CPOWOI field. */
#define BR_MCM_CPO_CPOWOI(x) (HW_MCM_CPO(x).B.CPOWOI)

/*! @brief Format value for bitfield MCM_CPO_CPOWOI. */
#define BF_MCM_CPO_CPOWOI(v) ((uint32_t)((uint32_t)(v) << BP_MCM_CPO_CPOWOI) & BM_MCM_CPO_CPOWOI)

/*! @brief Set the CPOWOI field to a new value. */
#define BW_MCM_CPO_CPOWOI(x, v) (HW_MCM_CPO_WR(x, (HW_MCM_CPO_RD(x) & ~BM_MCM_CPO_CPOWOI) | BF_MCM_CPO_CPOWOI(v)))
/*@}*/

/*******************************************************************************
 * hw_mcm_t - module struct
 ******************************************************************************/
/*!
 * @brief All MCM module registers.
 */
#pragma pack(1)
typedef struct _hw_mcm
{
    uint8_t _reserved0[8];
    __I hw_mcm_plasc_t PLASC;              /*!< [0x8] Crossbar Switch (AXBS) Slave Configuration */
    __I hw_mcm_plamc_t PLAMC;              /*!< [0xA] Crossbar Switch (AXBS) Master Configuration */
    __IO hw_mcm_placr_t PLACR;             /*!< [0xC] Platform Control Register */
    uint8_t _reserved1[48];
    __IO hw_mcm_cpo_t CPO;                 /*!< [0x40] Compute Operation Control Register */
} hw_mcm_t;
#pragma pack()

/*! @brief Macro to access all MCM registers. */
/*! @param x MCM module instance base address. */
/*! @return Reference (not a pointer) to the registers struct. To get a pointer to the struct,
 *     use the '&' operator, like <code>&HW_MCM(MCM_BASE)</code>. */
#define HW_MCM(x)      (*(hw_mcm_t *)(x))

#endif /* __HW_MCM_REGISTERS_H__ */
/* EOF */