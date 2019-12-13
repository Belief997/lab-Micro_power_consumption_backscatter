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
#ifndef __HW_NV_REGISTERS_H__
#define __HW_NV_REGISTERS_H__

#include "MKL03Z4.h"
#include "fsl_bitaccess.h"

/*
 * MKL03Z4 NV
 *
 * Flash configuration field
 *
 * Registers defined in this header file:
 * - HW_NV_BACKKEY3 - Backdoor Comparison Key 3.
 * - HW_NV_BACKKEY2 - Backdoor Comparison Key 2.
 * - HW_NV_BACKKEY1 - Backdoor Comparison Key 1.
 * - HW_NV_BACKKEY0 - Backdoor Comparison Key 0.
 * - HW_NV_BACKKEY7 - Backdoor Comparison Key 7.
 * - HW_NV_BACKKEY6 - Backdoor Comparison Key 6.
 * - HW_NV_BACKKEY5 - Backdoor Comparison Key 5.
 * - HW_NV_BACKKEY4 - Backdoor Comparison Key 4.
 * - HW_NV_FPROT3 - Non-volatile P-Flash Protection 1 - Low Register
 * - HW_NV_FPROT2 - Non-volatile P-Flash Protection 1 - High Register
 * - HW_NV_FPROT1 - Non-volatile P-Flash Protection 0 - Low Register
 * - HW_NV_FPROT0 - Non-volatile P-Flash Protection 0 - High Register
 * - HW_NV_FSEC - Non-volatile Flash Security Register
 * - HW_NV_FOPT - Non-volatile Flash Option Register
 *
 * - hw_nv_t - Struct containing all module registers.
 */

#define HW_NV_INSTANCE_COUNT (1U) /*!< Number of instances of the NV module. */

/*******************************************************************************
 * HW_NV_BACKKEY3 - Backdoor Comparison Key 3.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY3 - Backdoor Comparison Key 3. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey3
{
    uint8_t U;
    struct _hw_nv_backkey3_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey3_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY3 register
 */
/*@{*/
#define HW_NV_BACKKEY3_ADDR(x)   ((x) + 0x0U)

#define HW_NV_BACKKEY3(x)        (*(__I hw_nv_backkey3_t *) HW_NV_BACKKEY3_ADDR(x))
#define HW_NV_BACKKEY3_RD(x)     (HW_NV_BACKKEY3(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY3 bitfields
 */

/*!
 * @name Register NV_BACKKEY3, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY3_KEY   (0U)          /*!< Bit position for NV_BACKKEY3_KEY. */
#define BM_NV_BACKKEY3_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY3_KEY. */
#define BS_NV_BACKKEY3_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY3_KEY. */

/*! @brief Read current value of the NV_BACKKEY3_KEY field. */
#define BR_NV_BACKKEY3_KEY(x) (HW_NV_BACKKEY3(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY2 - Backdoor Comparison Key 2.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY2 - Backdoor Comparison Key 2. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey2
{
    uint8_t U;
    struct _hw_nv_backkey2_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey2_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY2 register
 */
/*@{*/
#define HW_NV_BACKKEY2_ADDR(x)   ((x) + 0x1U)

#define HW_NV_BACKKEY2(x)        (*(__I hw_nv_backkey2_t *) HW_NV_BACKKEY2_ADDR(x))
#define HW_NV_BACKKEY2_RD(x)     (HW_NV_BACKKEY2(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY2 bitfields
 */

/*!
 * @name Register NV_BACKKEY2, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY2_KEY   (0U)          /*!< Bit position for NV_BACKKEY2_KEY. */
#define BM_NV_BACKKEY2_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY2_KEY. */
#define BS_NV_BACKKEY2_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY2_KEY. */

/*! @brief Read current value of the NV_BACKKEY2_KEY field. */
#define BR_NV_BACKKEY2_KEY(x) (HW_NV_BACKKEY2(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY1 - Backdoor Comparison Key 1.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY1 - Backdoor Comparison Key 1. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey1
{
    uint8_t U;
    struct _hw_nv_backkey1_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey1_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY1 register
 */
/*@{*/
#define HW_NV_BACKKEY1_ADDR(x)   ((x) + 0x2U)

#define HW_NV_BACKKEY1(x)        (*(__I hw_nv_backkey1_t *) HW_NV_BACKKEY1_ADDR(x))
#define HW_NV_BACKKEY1_RD(x)     (HW_NV_BACKKEY1(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY1 bitfields
 */

/*!
 * @name Register NV_BACKKEY1, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY1_KEY   (0U)          /*!< Bit position for NV_BACKKEY1_KEY. */
#define BM_NV_BACKKEY1_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY1_KEY. */
#define BS_NV_BACKKEY1_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY1_KEY. */

/*! @brief Read current value of the NV_BACKKEY1_KEY field. */
#define BR_NV_BACKKEY1_KEY(x) (HW_NV_BACKKEY1(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY0 - Backdoor Comparison Key 0.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY0 - Backdoor Comparison Key 0. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey0
{
    uint8_t U;
    struct _hw_nv_backkey0_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey0_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY0 register
 */
/*@{*/
#define HW_NV_BACKKEY0_ADDR(x)   ((x) + 0x3U)

#define HW_NV_BACKKEY0(x)        (*(__I hw_nv_backkey0_t *) HW_NV_BACKKEY0_ADDR(x))
#define HW_NV_BACKKEY0_RD(x)     (HW_NV_BACKKEY0(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY0 bitfields
 */

/*!
 * @name Register NV_BACKKEY0, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY0_KEY   (0U)          /*!< Bit position for NV_BACKKEY0_KEY. */
#define BM_NV_BACKKEY0_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY0_KEY. */
#define BS_NV_BACKKEY0_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY0_KEY. */

/*! @brief Read current value of the NV_BACKKEY0_KEY field. */
#define BR_NV_BACKKEY0_KEY(x) (HW_NV_BACKKEY0(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY7 - Backdoor Comparison Key 7.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY7 - Backdoor Comparison Key 7. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey7
{
    uint8_t U;
    struct _hw_nv_backkey7_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey7_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY7 register
 */
/*@{*/
#define HW_NV_BACKKEY7_ADDR(x)   ((x) + 0x4U)

#define HW_NV_BACKKEY7(x)        (*(__I hw_nv_backkey7_t *) HW_NV_BACKKEY7_ADDR(x))
#define HW_NV_BACKKEY7_RD(x)     (HW_NV_BACKKEY7(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY7 bitfields
 */

/*!
 * @name Register NV_BACKKEY7, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY7_KEY   (0U)          /*!< Bit position for NV_BACKKEY7_KEY. */
#define BM_NV_BACKKEY7_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY7_KEY. */
#define BS_NV_BACKKEY7_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY7_KEY. */

/*! @brief Read current value of the NV_BACKKEY7_KEY field. */
#define BR_NV_BACKKEY7_KEY(x) (HW_NV_BACKKEY7(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY6 - Backdoor Comparison Key 6.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY6 - Backdoor Comparison Key 6. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey6
{
    uint8_t U;
    struct _hw_nv_backkey6_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey6_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY6 register
 */
/*@{*/
#define HW_NV_BACKKEY6_ADDR(x)   ((x) + 0x5U)

#define HW_NV_BACKKEY6(x)        (*(__I hw_nv_backkey6_t *) HW_NV_BACKKEY6_ADDR(x))
#define HW_NV_BACKKEY6_RD(x)     (HW_NV_BACKKEY6(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY6 bitfields
 */

/*!
 * @name Register NV_BACKKEY6, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY6_KEY   (0U)          /*!< Bit position for NV_BACKKEY6_KEY. */
#define BM_NV_BACKKEY6_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY6_KEY. */
#define BS_NV_BACKKEY6_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY6_KEY. */

/*! @brief Read current value of the NV_BACKKEY6_KEY field. */
#define BR_NV_BACKKEY6_KEY(x) (HW_NV_BACKKEY6(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY5 - Backdoor Comparison Key 5.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY5 - Backdoor Comparison Key 5. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey5
{
    uint8_t U;
    struct _hw_nv_backkey5_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey5_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY5 register
 */
/*@{*/
#define HW_NV_BACKKEY5_ADDR(x)   ((x) + 0x6U)

#define HW_NV_BACKKEY5(x)        (*(__I hw_nv_backkey5_t *) HW_NV_BACKKEY5_ADDR(x))
#define HW_NV_BACKKEY5_RD(x)     (HW_NV_BACKKEY5(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY5 bitfields
 */

/*!
 * @name Register NV_BACKKEY5, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY5_KEY   (0U)          /*!< Bit position for NV_BACKKEY5_KEY. */
#define BM_NV_BACKKEY5_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY5_KEY. */
#define BS_NV_BACKKEY5_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY5_KEY. */

/*! @brief Read current value of the NV_BACKKEY5_KEY field. */
#define BR_NV_BACKKEY5_KEY(x) (HW_NV_BACKKEY5(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_BACKKEY4 - Backdoor Comparison Key 4.
 ******************************************************************************/

/*!
 * @brief HW_NV_BACKKEY4 - Backdoor Comparison Key 4. (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_backkey4
{
    uint8_t U;
    struct _hw_nv_backkey4_bitfields
    {
        uint8_t KEY : 8;               /*!< [7:0] Backdoor Comparison Key. */
    } B;
} hw_nv_backkey4_t;

/*!
 * @name Constants and macros for entire NV_BACKKEY4 register
 */
/*@{*/
#define HW_NV_BACKKEY4_ADDR(x)   ((x) + 0x7U)

#define HW_NV_BACKKEY4(x)        (*(__I hw_nv_backkey4_t *) HW_NV_BACKKEY4_ADDR(x))
#define HW_NV_BACKKEY4_RD(x)     (HW_NV_BACKKEY4(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_BACKKEY4 bitfields
 */

/*!
 * @name Register NV_BACKKEY4, field KEY[7:0] (RO)
 */
/*@{*/
#define BP_NV_BACKKEY4_KEY   (0U)          /*!< Bit position for NV_BACKKEY4_KEY. */
#define BM_NV_BACKKEY4_KEY   (0xFFU)       /*!< Bit mask for NV_BACKKEY4_KEY. */
#define BS_NV_BACKKEY4_KEY   (8U)          /*!< Bit field size in bits for NV_BACKKEY4_KEY. */

/*! @brief Read current value of the NV_BACKKEY4_KEY field. */
#define BR_NV_BACKKEY4_KEY(x) (HW_NV_BACKKEY4(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_FPROT3 - Non-volatile P-Flash Protection 1 - Low Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FPROT3 - Non-volatile P-Flash Protection 1 - Low Register (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_fprot3
{
    uint8_t U;
    struct _hw_nv_fprot3_bitfields
    {
        uint8_t PROT : 8;              /*!< [7:0] P-Flash Region Protect */
    } B;
} hw_nv_fprot3_t;

/*!
 * @name Constants and macros for entire NV_FPROT3 register
 */
/*@{*/
#define HW_NV_FPROT3_ADDR(x)     ((x) + 0x8U)

#define HW_NV_FPROT3(x)          (*(__I hw_nv_fprot3_t *) HW_NV_FPROT3_ADDR(x))
#define HW_NV_FPROT3_RD(x)       (HW_NV_FPROT3(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FPROT3 bitfields
 */

/*!
 * @name Register NV_FPROT3, field PROT[7:0] (RO)
 */
/*@{*/
#define BP_NV_FPROT3_PROT    (0U)          /*!< Bit position for NV_FPROT3_PROT. */
#define BM_NV_FPROT3_PROT    (0xFFU)       /*!< Bit mask for NV_FPROT3_PROT. */
#define BS_NV_FPROT3_PROT    (8U)          /*!< Bit field size in bits for NV_FPROT3_PROT. */

/*! @brief Read current value of the NV_FPROT3_PROT field. */
#define BR_NV_FPROT3_PROT(x) (HW_NV_FPROT3(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_FPROT2 - Non-volatile P-Flash Protection 1 - High Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FPROT2 - Non-volatile P-Flash Protection 1 - High Register (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_fprot2
{
    uint8_t U;
    struct _hw_nv_fprot2_bitfields
    {
        uint8_t PROT : 8;              /*!< [7:0] P-Flash Region Protect */
    } B;
} hw_nv_fprot2_t;

/*!
 * @name Constants and macros for entire NV_FPROT2 register
 */
/*@{*/
#define HW_NV_FPROT2_ADDR(x)     ((x) + 0x9U)

#define HW_NV_FPROT2(x)          (*(__I hw_nv_fprot2_t *) HW_NV_FPROT2_ADDR(x))
#define HW_NV_FPROT2_RD(x)       (HW_NV_FPROT2(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FPROT2 bitfields
 */

/*!
 * @name Register NV_FPROT2, field PROT[7:0] (RO)
 */
/*@{*/
#define BP_NV_FPROT2_PROT    (0U)          /*!< Bit position for NV_FPROT2_PROT. */
#define BM_NV_FPROT2_PROT    (0xFFU)       /*!< Bit mask for NV_FPROT2_PROT. */
#define BS_NV_FPROT2_PROT    (8U)          /*!< Bit field size in bits for NV_FPROT2_PROT. */

/*! @brief Read current value of the NV_FPROT2_PROT field. */
#define BR_NV_FPROT2_PROT(x) (HW_NV_FPROT2(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_FPROT1 - Non-volatile P-Flash Protection 0 - Low Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FPROT1 - Non-volatile P-Flash Protection 0 - Low Register (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_fprot1
{
    uint8_t U;
    struct _hw_nv_fprot1_bitfields
    {
        uint8_t PROT : 8;              /*!< [7:0] P-Flash Region Protect */
    } B;
} hw_nv_fprot1_t;

/*!
 * @name Constants and macros for entire NV_FPROT1 register
 */
/*@{*/
#define HW_NV_FPROT1_ADDR(x)     ((x) + 0xAU)

#define HW_NV_FPROT1(x)          (*(__I hw_nv_fprot1_t *) HW_NV_FPROT1_ADDR(x))
#define HW_NV_FPROT1_RD(x)       (HW_NV_FPROT1(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FPROT1 bitfields
 */

/*!
 * @name Register NV_FPROT1, field PROT[7:0] (RO)
 */
/*@{*/
#define BP_NV_FPROT1_PROT    (0U)          /*!< Bit position for NV_FPROT1_PROT. */
#define BM_NV_FPROT1_PROT    (0xFFU)       /*!< Bit mask for NV_FPROT1_PROT. */
#define BS_NV_FPROT1_PROT    (8U)          /*!< Bit field size in bits for NV_FPROT1_PROT. */

/*! @brief Read current value of the NV_FPROT1_PROT field. */
#define BR_NV_FPROT1_PROT(x) (HW_NV_FPROT1(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_FPROT0 - Non-volatile P-Flash Protection 0 - High Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FPROT0 - Non-volatile P-Flash Protection 0 - High Register (RO)
 *
 * Reset value: 0xFFU
 */
typedef union _hw_nv_fprot0
{
    uint8_t U;
    struct _hw_nv_fprot0_bitfields
    {
        uint8_t PROT : 8;              /*!< [7:0] P-Flash Region Protect */
    } B;
} hw_nv_fprot0_t;

/*!
 * @name Constants and macros for entire NV_FPROT0 register
 */
/*@{*/
#define HW_NV_FPROT0_ADDR(x)     ((x) + 0xBU)

#define HW_NV_FPROT0(x)          (*(__I hw_nv_fprot0_t *) HW_NV_FPROT0_ADDR(x))
#define HW_NV_FPROT0_RD(x)       (HW_NV_FPROT0(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FPROT0 bitfields
 */

/*!
 * @name Register NV_FPROT0, field PROT[7:0] (RO)
 */
/*@{*/
#define BP_NV_FPROT0_PROT    (0U)          /*!< Bit position for NV_FPROT0_PROT. */
#define BM_NV_FPROT0_PROT    (0xFFU)       /*!< Bit mask for NV_FPROT0_PROT. */
#define BS_NV_FPROT0_PROT    (8U)          /*!< Bit field size in bits for NV_FPROT0_PROT. */

/*! @brief Read current value of the NV_FPROT0_PROT field. */
#define BR_NV_FPROT0_PROT(x) (HW_NV_FPROT0(x).U)
/*@}*/

/*******************************************************************************
 * HW_NV_FSEC - Non-volatile Flash Security Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FSEC - Non-volatile Flash Security Register (RO)
 *
 * Reset value: 0xFFU
 *
 * Allows the user to customize the operation of the MCU at boot time
 */
typedef union _hw_nv_fsec
{
    uint8_t U;
    struct _hw_nv_fsec_bitfields
    {
        uint8_t SEC : 2;               /*!< [1:0] Flash Security */
        uint8_t FSLACC : 2;            /*!< [3:2] Freescale Failure Analysis Access Code
                                        * */
        uint8_t MEEN : 2;              /*!< [5:4]  */
        uint8_t KEYEN : 2;             /*!< [7:6] Backdoor Key Security Enable */
    } B;
} hw_nv_fsec_t;

/*!
 * @name Constants and macros for entire NV_FSEC register
 */
/*@{*/
#define HW_NV_FSEC_ADDR(x)       ((x) + 0xCU)

#define HW_NV_FSEC(x)            (*(__I hw_nv_fsec_t *) HW_NV_FSEC_ADDR(x))
#define HW_NV_FSEC_RD(x)         (HW_NV_FSEC(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FSEC bitfields
 */

/*!
 * @name Register NV_FSEC, field SEC[1:0] (RO)
 *
 * Values:
 * - 10 - MCU security status is unsecure
 * - 11 - MCU security status is secure
 */
/*@{*/
#define BP_NV_FSEC_SEC       (0U)          /*!< Bit position for NV_FSEC_SEC. */
#define BM_NV_FSEC_SEC       (0x03U)       /*!< Bit mask for NV_FSEC_SEC. */
#define BS_NV_FSEC_SEC       (2U)          /*!< Bit field size in bits for NV_FSEC_SEC. */

/*! @brief Read current value of the NV_FSEC_SEC field. */
#define BR_NV_FSEC_SEC(x)    (HW_NV_FSEC(x).B.SEC)
/*@}*/

/*!
 * @name Register NV_FSEC, field FSLACC[3:2] (RO)
 *
 * Values:
 * - 10 - Freescale factory access denied
 * - 11 - Freescale factory access granted
 */
/*@{*/
#define BP_NV_FSEC_FSLACC    (2U)          /*!< Bit position for NV_FSEC_FSLACC. */
#define BM_NV_FSEC_FSLACC    (0x0CU)       /*!< Bit mask for NV_FSEC_FSLACC. */
#define BS_NV_FSEC_FSLACC    (2U)          /*!< Bit field size in bits for NV_FSEC_FSLACC. */

/*! @brief Read current value of the NV_FSEC_FSLACC field. */
#define BR_NV_FSEC_FSLACC(x) (HW_NV_FSEC(x).B.FSLACC)
/*@}*/

/*!
 * @name Register NV_FSEC, field MEEN[5:4] (RO)
 *
 * Values:
 * - 10 - Mass erase is disabled
 * - 11 - Mass erase is enabled
 */
/*@{*/
#define BP_NV_FSEC_MEEN      (4U)          /*!< Bit position for NV_FSEC_MEEN. */
#define BM_NV_FSEC_MEEN      (0x30U)       /*!< Bit mask for NV_FSEC_MEEN. */
#define BS_NV_FSEC_MEEN      (2U)          /*!< Bit field size in bits for NV_FSEC_MEEN. */

/*! @brief Read current value of the NV_FSEC_MEEN field. */
#define BR_NV_FSEC_MEEN(x)   (HW_NV_FSEC(x).B.MEEN)
/*@}*/

/*!
 * @name Register NV_FSEC, field KEYEN[7:6] (RO)
 *
 * Values:
 * - 10 - Backdoor key access enabled
 * - 11 - Backdoor key access disabled
 */
/*@{*/
#define BP_NV_FSEC_KEYEN     (6U)          /*!< Bit position for NV_FSEC_KEYEN. */
#define BM_NV_FSEC_KEYEN     (0xC0U)       /*!< Bit mask for NV_FSEC_KEYEN. */
#define BS_NV_FSEC_KEYEN     (2U)          /*!< Bit field size in bits for NV_FSEC_KEYEN. */

/*! @brief Read current value of the NV_FSEC_KEYEN field. */
#define BR_NV_FSEC_KEYEN(x)  (HW_NV_FSEC(x).B.KEYEN)
/*@}*/

/*******************************************************************************
 * HW_NV_FOPT - Non-volatile Flash Option Register
 ******************************************************************************/

/*!
 * @brief HW_NV_FOPT - Non-volatile Flash Option Register (RO)
 *
 * Reset value: 0x3DU
 */
typedef union _hw_nv_fopt
{
    uint8_t U;
    struct _hw_nv_fopt_bitfields
    {
        uint8_t LPBOOT0 : 1;           /*!< [0]  */
        uint8_t BOOTPIN_OPT : 1;       /*!< [1]  */
        uint8_t NMI_DIS : 1;           /*!< [2]  */
        uint8_t RESET_PIN_CFG : 1;     /*!< [3]  */
        uint8_t LPBOOT1 : 1;           /*!< [4]  */
        uint8_t FAST_INIT : 1;         /*!< [5]  */
        uint8_t BOOTSRC_SEL : 2;       /*!< [7:6] Boot source selection */
    } B;
} hw_nv_fopt_t;

/*!
 * @name Constants and macros for entire NV_FOPT register
 */
/*@{*/
#define HW_NV_FOPT_ADDR(x)       ((x) + 0xDU)

#define HW_NV_FOPT(x)            (*(__I hw_nv_fopt_t *) HW_NV_FOPT_ADDR(x))
#define HW_NV_FOPT_RD(x)         (HW_NV_FOPT(x).U)
/*@}*/

/*
 * Constants & macros for individual NV_FOPT bitfields
 */

/*!
 * @name Register NV_FOPT, field LPBOOT0[0] (RO)
 *
 * Values:
 * - 00 - Core and system clock divider (OUTDIV1) is 0x7 (divide by 8) when
 *     LPBOOT1=0 or 0x1 (divide by 2) when LPBOOT1=1.
 * - 01 - Core and system clock divider (OUTDIV1) is 0x3 (divide by 4) when
 *     LPBOOT1=0 or 0x0 (divide by 1) when LPBOOT1=1.
 */
/*@{*/
#define BP_NV_FOPT_LPBOOT0   (0U)          /*!< Bit position for NV_FOPT_LPBOOT0. */
#define BM_NV_FOPT_LPBOOT0   (0x01U)       /*!< Bit mask for NV_FOPT_LPBOOT0. */
#define BS_NV_FOPT_LPBOOT0   (1U)          /*!< Bit field size in bits for NV_FOPT_LPBOOT0. */

/*! @brief Read current value of the NV_FOPT_LPBOOT0 field. */
#define BR_NV_FOPT_LPBOOT0(x) (HW_NV_FOPT(x).B.LPBOOT0)
/*@}*/

/*!
 * @name Register NV_FOPT, field BOOTPIN_OPT[1] (RO)
 *
 * Values:
 * - 00 - Force Boot from ROM if BOOTCFG0 asserted, where BOOTCFG0 is the boot
 *     config function which is muxed with NMI pin
 * - 01 - Boot source configured by FOPT (BOOTSRC_SEL) bits
 */
/*@{*/
#define BP_NV_FOPT_BOOTPIN_OPT (1U)        /*!< Bit position for NV_FOPT_BOOTPIN_OPT. */
#define BM_NV_FOPT_BOOTPIN_OPT (0x02U)     /*!< Bit mask for NV_FOPT_BOOTPIN_OPT. */
#define BS_NV_FOPT_BOOTPIN_OPT (1U)        /*!< Bit field size in bits for NV_FOPT_BOOTPIN_OPT. */

/*! @brief Read current value of the NV_FOPT_BOOTPIN_OPT field. */
#define BR_NV_FOPT_BOOTPIN_OPT(x) (HW_NV_FOPT(x).B.BOOTPIN_OPT)
/*@}*/

/*!
 * @name Register NV_FOPT, field NMI_DIS[2] (RO)
 *
 * Values:
 * - 00 - NMI interrupts are always blocked
 * - 01 - NMI_b pin/interrupts reset default to enabled
 */
/*@{*/
#define BP_NV_FOPT_NMI_DIS   (2U)          /*!< Bit position for NV_FOPT_NMI_DIS. */
#define BM_NV_FOPT_NMI_DIS   (0x04U)       /*!< Bit mask for NV_FOPT_NMI_DIS. */
#define BS_NV_FOPT_NMI_DIS   (1U)          /*!< Bit field size in bits for NV_FOPT_NMI_DIS. */

/*! @brief Read current value of the NV_FOPT_NMI_DIS field. */
#define BR_NV_FOPT_NMI_DIS(x) (HW_NV_FOPT(x).B.NMI_DIS)
/*@}*/

/*!
 * @name Register NV_FOPT, field RESET_PIN_CFG[3] (RO)
 *
 * Values:
 * - 00 - RESET pin is disabled following a POR and cannot be enabled as reset
 *     function
 * - 01 - RESET_b pin is dedicated
 */
/*@{*/
#define BP_NV_FOPT_RESET_PIN_CFG (3U)      /*!< Bit position for NV_FOPT_RESET_PIN_CFG. */
#define BM_NV_FOPT_RESET_PIN_CFG (0x08U)   /*!< Bit mask for NV_FOPT_RESET_PIN_CFG. */
#define BS_NV_FOPT_RESET_PIN_CFG (1U)      /*!< Bit field size in bits for NV_FOPT_RESET_PIN_CFG. */

/*! @brief Read current value of the NV_FOPT_RESET_PIN_CFG field. */
#define BR_NV_FOPT_RESET_PIN_CFG(x) (HW_NV_FOPT(x).B.RESET_PIN_CFG)
/*@}*/

/*!
 * @name Register NV_FOPT, field LPBOOT1[4] (RO)
 *
 * Values:
 * - 00 - Core and system clock divider (OUTDIV1) is 0x7 (divide by 8) when
 *     LPBOOT0=0 or 0x3 (divide by 4) when LPBOOT0=1.
 * - 01 - Core and system clock divider (OUTDIV1) is 0x1 (divide by 2) when
 *     LPBOOT0=0 or 0x0 (divide by 1) when LPBOOT0=1.
 */
/*@{*/
#define BP_NV_FOPT_LPBOOT1   (4U)          /*!< Bit position for NV_FOPT_LPBOOT1. */
#define BM_NV_FOPT_LPBOOT1   (0x10U)       /*!< Bit mask for NV_FOPT_LPBOOT1. */
#define BS_NV_FOPT_LPBOOT1   (1U)          /*!< Bit field size in bits for NV_FOPT_LPBOOT1. */

/*! @brief Read current value of the NV_FOPT_LPBOOT1 field. */
#define BR_NV_FOPT_LPBOOT1(x) (HW_NV_FOPT(x).B.LPBOOT1)
/*@}*/

/*!
 * @name Register NV_FOPT, field FAST_INIT[5] (RO)
 *
 * Values:
 * - 00 - Slower initialization
 * - 01 - Fast Initialization
 */
/*@{*/
#define BP_NV_FOPT_FAST_INIT (5U)          /*!< Bit position for NV_FOPT_FAST_INIT. */
#define BM_NV_FOPT_FAST_INIT (0x20U)       /*!< Bit mask for NV_FOPT_FAST_INIT. */
#define BS_NV_FOPT_FAST_INIT (1U)          /*!< Bit field size in bits for NV_FOPT_FAST_INIT. */

/*! @brief Read current value of the NV_FOPT_FAST_INIT field. */
#define BR_NV_FOPT_FAST_INIT(x) (HW_NV_FOPT(x).B.FAST_INIT)
/*@}*/

/*!
 * @name Register NV_FOPT, field BOOTSRC_SEL[7:6] (RO)
 *
 * Values:
 * - 00 - Boot from Flash
 * - 10 - Boot from ROM
 * - 11 - Boot from ROM
 */
/*@{*/
#define BP_NV_FOPT_BOOTSRC_SEL (6U)        /*!< Bit position for NV_FOPT_BOOTSRC_SEL. */
#define BM_NV_FOPT_BOOTSRC_SEL (0xC0U)     /*!< Bit mask for NV_FOPT_BOOTSRC_SEL. */
#define BS_NV_FOPT_BOOTSRC_SEL (2U)        /*!< Bit field size in bits for NV_FOPT_BOOTSRC_SEL. */

/*! @brief Read current value of the NV_FOPT_BOOTSRC_SEL field. */
#define BR_NV_FOPT_BOOTSRC_SEL(x) (HW_NV_FOPT(x).B.BOOTSRC_SEL)
/*@}*/

/*******************************************************************************
 * hw_nv_t - module struct
 ******************************************************************************/
/*!
 * @brief All NV module registers.
 */
#pragma pack(1)
typedef struct _hw_nv
{
    __I hw_nv_backkey3_t BACKKEY3;         /*!< [0x0] Backdoor Comparison Key 3. */
    __I hw_nv_backkey2_t BACKKEY2;         /*!< [0x1] Backdoor Comparison Key 2. */
    __I hw_nv_backkey1_t BACKKEY1;         /*!< [0x2] Backdoor Comparison Key 1. */
    __I hw_nv_backkey0_t BACKKEY0;         /*!< [0x3] Backdoor Comparison Key 0. */
    __I hw_nv_backkey7_t BACKKEY7;         /*!< [0x4] Backdoor Comparison Key 7. */
    __I hw_nv_backkey6_t BACKKEY6;         /*!< [0x5] Backdoor Comparison Key 6. */
    __I hw_nv_backkey5_t BACKKEY5;         /*!< [0x6] Backdoor Comparison Key 5. */
    __I hw_nv_backkey4_t BACKKEY4;         /*!< [0x7] Backdoor Comparison Key 4. */
    __I hw_nv_fprot3_t FPROT3;             /*!< [0x8] Non-volatile P-Flash Protection 1 - Low Register */
    __I hw_nv_fprot2_t FPROT2;             /*!< [0x9] Non-volatile P-Flash Protection 1 - High Register */
    __I hw_nv_fprot1_t FPROT1;             /*!< [0xA] Non-volatile P-Flash Protection 0 - Low Register */
    __I hw_nv_fprot0_t FPROT0;             /*!< [0xB] Non-volatile P-Flash Protection 0 - High Register */
    __I hw_nv_fsec_t FSEC;                 /*!< [0xC] Non-volatile Flash Security Register */
    __I hw_nv_fopt_t FOPT;                 /*!< [0xD] Non-volatile Flash Option Register */
} hw_nv_t;
#pragma pack()

/*! @brief Macro to access all NV registers. */
/*! @param x NV module instance base address. */
/*! @return Reference (not a pointer) to the registers struct. To get a pointer to the struct,
 *     use the '&' operator, like <code>&HW_NV(FTFA_FlashConfig_BASE)</code>. */
#define HW_NV(x)       (*(hw_nv_t *)(x))

#endif /* __HW_NV_REGISTERS_H__ */
/* EOF */
