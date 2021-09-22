/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
#ifndef _FSL_POWER_H_
#define _FSL_POWER_H_

#include <stdint.h>

/*! @addtogroup power */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAKE_PD_BITS(reg, slot) ((reg << 8) | slot)
#define PDRCFG0 0x0U
#define PDRCFG1 0x1U

typedef enum pd_bits
{
    kPDRUNCFG_PD_FRO_EN = MAKE_PD_BITS(PDRCFG0, 4U),
    kPDRUNCFG_PD_FLASH = MAKE_PD_BITS(PDRCFG0, 5U),
    kPDRUNCFG_PD_TEMPS = MAKE_PD_BITS(PDRCFG0, 6U),
    kPDRUNCFG_PD_BOD_RESET = MAKE_PD_BITS(PDRCFG0, 7U),
    kPDRUNCFG_PD_BOD_INTR = MAKE_PD_BITS(PDRCFG0, 8U),
    kPDRUNCFG_PD_ADC0 = MAKE_PD_BITS(PDRCFG0, 10U),
    kPDRUNCFG_PD_VDDFLASH = MAKE_PD_BITS(PDRCFG0, 11U),
    kPDRUNCFG_LP_VDDFLASH = MAKE_PD_BITS(PDRCFG0, 12U),
    kPDRUNCFG_PD_RAM0 = MAKE_PD_BITS(PDRCFG0, 13U),
    kPDRUNCFG_PD_RAM1 = MAKE_PD_BITS(PDRCFG0, 14U),
    kPDRUNCFG_PD_RAM2 = MAKE_PD_BITS(PDRCFG0, 15U),
    kPDRUNCFG_PD_RAMX = MAKE_PD_BITS(PDRCFG0, 16U),
    kPDRUNCFG_PD_ROM = MAKE_PD_BITS(PDRCFG0, 17U),
    kPDRUNCFG_PD_VDDHV_ENA = MAKE_PD_BITS(PDRCFG0, 18U),
    kPDRUNCFG_PD_VD7_ENA = MAKE_PD_BITS(PDRCFG0, 19U),
    kPDRUNCFG_PD_WDT_OSC = MAKE_PD_BITS(PDRCFG0, 20U),
    kPDRUNCFG_PD_USB0_PHY = MAKE_PD_BITS(PDRCFG0, 21U),
    kPDRUNCFG_PD_SYS_PLL0 = MAKE_PD_BITS(PDRCFG0, 22U),
    kPDRUNCFG_PD_VREFP_SW = MAKE_PD_BITS(PDRCFG0, 23U),
    kPDRUNCFG_PD_FLASH_BG = MAKE_PD_BITS(PDRCFG0, 25U),

    kPDRUNCFG_PD_ALT_FLASH_IBG = MAKE_PD_BITS(PDRCFG1, 28U),
    kPDRUNCFG_SEL_ALT_FLASH_IBG = MAKE_PD_BITS(PDRCFG1, 29U),
    
    kPDRUNCFG_ForceUnsigned = 0x80000000U
} pd_bit_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
* @name Power Configuration
* @{
*/


/*!
 * @brief API to enable deep sleep bit in the ARM Core.
 *
 * @return none
 */
static inline void POWER_EnableDeepSleep(void)
{
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}




/*!
 * @brief Power Library API to enter deep sleep mode.
 *
 * @param exclude_from_pd  Bit mask of the PDRUNCFG bits that needs to be powered on during deep sleep
 * @return none
 */
void POWER_EnterDeepSleep(uint32_t exclude_from_pd);

/*!
 * @brief Power Library API to enter deep power down mode.
 *
 * @param exclude_from_pd  Bit mask of the PDRUNCFG bits that needs to be powered on during deep power down mode, 
 *                         but this is has no effect as the voltages are cut off.
 * @return none
 */
void POWER_EnterDeepPowerDown(uint32_t exclude_from_pd);

/*!
 * @brief Power Library API to choose normal regulation and set the voltage for the desired operating frequency.
 *
 * @param freq  - The desired frequency at which the part would like to operate, 
 *                note that the voltage and flash wait states should be set before changing frequency
 * @return none
 */
void POWER_SetVoltageForFreq(uint32_t freq);

/*!
 * @brief Power Library API to choose low power regulation and set the voltage for the desired operating frequency.
 *
 * @param freq  - The desired frequency at which the part would like to operate, 
 *                note only 12MHz and 48Mhz are supported
 * @return none
 */
void POWER_SetLowPowerVoltageForFreq(uint32_t freq);

/*!
 * @brief Power Library API to return the library version.
 *
 * @return version number of the power library
 */
uint32_t POWER_GetLibVersion(void);

/* @} */

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* _FSL_POWER_H_ */
