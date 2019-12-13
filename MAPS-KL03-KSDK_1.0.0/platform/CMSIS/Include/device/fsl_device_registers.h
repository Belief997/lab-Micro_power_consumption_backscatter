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
#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

/*
 * Include the cpu specific register header files.
 *
 * The CPU macro should be declared in the project or makefile.
 */

    #define KL03Z4_SERIES

    /* CMSIS-style register definitions */
    #include "device/MKL03Z4/MKL03Z4.h"

    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.) */
    #include "device/MKL03Z4/MKL03Z4_adc.h"
    #include "device/MKL03Z4/MKL03Z4_cmp.h"
    #include "device/MKL03Z4/MKL03Z4_fgpio.h"
    #include "device/MKL03Z4/MKL03Z4_ftfa.h"
    #include "device/MKL03Z4/MKL03Z4_gpio.h"
    #include "device/MKL03Z4/MKL03Z4_i2c.h"
    #include "device/MKL03Z4/MKL03Z4_llwu.h"
    #include "device/MKL03Z4/MKL03Z4_lptmr.h"
    #include "device/MKL03Z4/MKL03Z4_lpuart.h"
    #include "device/MKL03Z4/MKL03Z4_mcg.h"
    #include "device/MKL03Z4/MKL03Z4_mcm.h"
    #include "device/MKL03Z4/MKL03Z4_mtb.h"
    #include "device/MKL03Z4/MKL03Z4_mtbdwt.h"
    #include "device/MKL03Z4/MKL03Z4_nv.h"
    #include "device/MKL03Z4/MKL03Z4_osc.h"
    #include "device/MKL03Z4/MKL03Z4_pmc.h"
    #include "device/MKL03Z4/MKL03Z4_port.h"
    #include "device/MKL03Z4/MKL03Z4_rcm.h"
    #include "device/MKL03Z4/MKL03Z4_rfsys.h"
    #include "device/MKL03Z4/MKL03Z4_rom.h"
    #include "device/MKL03Z4/MKL03Z4_rtc.h"
    #include "device/MKL03Z4/MKL03Z4_sim.h"
    #include "device/MKL03Z4/MKL03Z4_smc.h"
    #include "device/MKL03Z4/MKL03Z4_spi.h"
    #include "device/MKL03Z4/MKL03Z4_tpm.h"
    #include "device/MKL03Z4/MKL03Z4_vref.h"


#define FSL_FEATURE_ADC_CONVERSION_CONTROL_COUNT (2)

#define FSL_FEATURE_TPM_CHANNEL_COUNT (2)

#define FSL_FEATURE_TPM_CHANNEL_COUNTn(x) \
        ((x) == 0 ? (2) : \
        ((x) == 1 ? (2) : (-1)))

#define FSL_FEATURE_LLWU_HAS_EXTERNAL_PIN (8)

#define FSL_FEATURE_LLWU_HAS_INTERNAL_MODULE (1)

#define FSL_FEATURE_LLWU_HAS_PIN_FILTER (1)

#define FSL_FEATURE_INTERRUPT_IRQ_MAX (31)

#define FSL_FEATURE_INTERRUPT_IRQ_MIN (-14)

#endif /* __FSL_DEVICE_REGISTERS_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
