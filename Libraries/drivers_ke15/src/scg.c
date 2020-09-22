/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "scg.h"


#if defined(SCG)


uint32_t SCG_GetMode(void)
{
    return (SCG_Mode_t)((SCG_CSR_SCS_MASK & SCG->CSR) >> SCG_CSR_SCS_SHIFT);
}

/* this function will enable FIRC, and SPLL, SPLL source is fiexed from FIRC */
uint32_t ClockSetup(uint32_t opt)
{
    static bool isInitialized = false;
    
    if(isInitialized == true)
    {
        return CH_ERR;
    }
    
    uint32_t tmp;
    
    /* set all clk div to lowest */
    tmp = SCG->RCCR;
    tmp |= SCG_RCCR_DIVCORE_MASK | SCG_RCCR_DIVSLOW_MASK;
    SCG->RCCR = tmp;
    
    /* enable FIRC */
    SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN_MASK | SCG_FIRCCSR_FIRCLPEN_MASK | SCG_FIRCCSR_FIRCSTEN_MASK;
    while(0 == (SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)) {};  
    SCG->RCCR = SCG_RCCR_SCS(kSCG_FIRC) | SCG_RCCR_DIVCORE(0) | SCG_RCCR_DIVSLOW(1);
    while(kSCG_FIRC != SCG_GetMode());

    switch(opt)
    {
        case IRC_48M:
                /* disable amd cponfig PLL */
            //    SCG->LPFLLCSR = 0;
             //   SCG->LPFLLCFG = SCG_LPLLCFG_SOURCE(1) | SCG_SPLLCFG_MULT(0) | SCG_SPLLCFG_PREDIV(7);
             //   SCG->SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK | SCG_SPLLCSR_SPLLSTEN_MASK;
           //     while(0 == (SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)) {};
                    
                /* switch mode to SPLL */
//                SCG->RCCR = SCG_RCCR_SCS(kSCG_PLL) | SCG_RCCR_DIVCORE(0) | SCG_RCCR_DIVSLOW(1);
//                while(kSCG_PLL != SCG_GetMode());
                SystemCoreClock = 48*1000*1000;
            break;
        case IRC_96M:
                /* disable amd cponfig PLL */
//                SCG->SPLLCSR = 0;
//                SCG->SPLLCFG = SCG_SPLLCFG_SOURCE(1) | SCG_SPLLCFG_MULT(32-16) | SCG_SPLLCFG_PREDIV(7);
//                SCG->SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK | SCG_SPLLCSR_SPLLSTEN_MASK;
//                while(0 == (SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)) {};
//                    
//                /* switch mode to SPLL */
//                SCG->RCCR = SCG_RCCR_SCS(kSCG_PLL) | SCG_RCCR_DIVCORE(0) | SCG_RCCR_DIVSLOW(3); /* divslow clock also is flash clock, so must <25M */
//                while(kSCG_PLL != SCG_GetMode());
                SystemCoreClock = 96*1000*1000;
            break;      
        default:
            break;
    }
    return CH_OK;
}


#endif


