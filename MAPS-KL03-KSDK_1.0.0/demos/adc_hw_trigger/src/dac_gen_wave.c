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

#include "fsl_dac_driver.h"
#include "fsl_clock_manager.h"
#include "adc_hw_trigger.h"
#include "fsl_misc_utilities.h"
#include "fsl_hwtimer.h"

extern const uint32_t g_gpioBaseAddr[];

/* DAC buffer output data to create sine wave */
static uint16_t dacBuf[] = 
{
       0U,   39U,  156U,  345U,
     600U,  910U, 1264U, 1648U,
    2048U, 2448U, 2832U, 3186U,
    3496U, 3751U, 3940U, 4056U
};

static dac_state_t dacStateStruct;
extern const hwtimer_devif_t kSystickDevif;
static hwtimer_t hwtimer;

static void hwtimer_callback(void* data)
{
    DAC_DRV_SoftTriggerBuff(0);
}
/*!
 * @brief Use DAC fifo to generate sinewave on DACx_OUT
 */
int32_t dac_gen_wave(void)
{
    dac_user_config_t dacUserConfig;
    dac_buff_config_t dacBuffConfig;
    uint8_t buffLen;
    uint32_t period;

    buffLen = ARRAY_SIZE(dacBuf);

    /* Fill the structure with configuration of software trigger. */
    DAC_DRV_StructInitUserConfigNormal(&dacUserConfig);

    /* Initialize the DAC Converter. */
    DAC_DRV_Init(0, &dacUserConfig);

    /* Enable the feature of DAC internal buffer. */
#if FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
    dacBuffConfig.buffIndexWatermarkIntEnable = false;
    dacBuffConfig.watermarkMode = kDacBuffWatermarkFromUpperAs2Word;
#endif
    dacBuffConfig.buffIndexStartIntEnable = false;
    dacBuffConfig.buffIndexUpperIntEnable = false;
    dacBuffConfig.dmaEnable = false;
    dacBuffConfig.buffWorkMode = kDacBuffWorkAsSwingMode;
    dacBuffConfig.buffUpperIndex = buffLen - 1;
    DAC_DRV_EnableBuff(0, &dacBuffConfig, &dacStateStruct);

    /* Fill the buffer with setting data. */
    DAC_DRV_SetBuffValue(0, 0U, buffLen, dacBuf); 
 
    /*
     * Use HW timer of systick to do SW trigger of DAC,
     * the interrupt priority of systick should be set to low, e.g 5,
     * as the high priority should be reserved for ADC/PDB ISR.
     */
    if (kHwtimerSuccess != HWTIMER_SYS_Init(&hwtimer, &kSystickDevif, 0, 5, NULL))
    {
        return -1;
    }
    /*
     * Get the peroid the systick triggered.
     * There's 30 times of systick interrupt for one peroid of sine wave,
     * as we only have 16 data depth buffer for DAC, so we need to trigger
     * 30 times of interrupt to do software trigger.
     */
    period = INPUT_SIGNAL_FREQ * (2 * buffLen - 2);
    if (kHwtimerSuccess != HWTIMER_SYS_SetPeriod(&hwtimer, kCoreClock, 1000000/period))
    {
        return -1;
    }
    /* install call back for SW trigger for DAC */
    if (kHwtimerSuccess != HWTIMER_SYS_RegisterCallback(&hwtimer, hwtimer_callback, 0))
    {
        return -1;
    }
    /* start systick timer */
    if (kHwtimerSuccess != HWTIMER_SYS_Start(&hwtimer))
    {
        return -1;
    }

    return 0;
}

void dac_stop_wave(void)
{
    /* Disable the systick timer */
    HWTIMER_SYS_Stop(&hwtimer);

    /* Disable the feature of DAC internal buffer. */
    DAC_DRV_DisableBuff(0);
    
    /* De-initialize the DAC converter. */
    DAC_DRV_Deinit(0);
}
