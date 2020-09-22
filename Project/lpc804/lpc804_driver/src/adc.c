/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "adc.h"

void ADC_Init(uint32_t sample_rate)
{
    /* enable uart clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<24);
    LPC_SYSCON->PDRUNCFG &= ~(1<<4);
    LPC_SYSCON->ADCCLKDIV = 1;  // Enable clock, and divide-by-1 at this clock divider
    LPC_SYSCON->ADCCLKSEL = 0; // Use fro_clk as source for ADC async clock
    
    /* clock div */
    LPC_ADC->CTRL = 1;
}

void ADC_SetIntMode(ADC_Int_t mode, bool val)
{
    switch(mode)
    {
        case kADC_SEQA:
            (val)?(LPC_ADC->INTEN |= (1<<0)):(LPC_ADC->INTEN &= ~(1<<0));
            break;
        case kADC_SEQB:
            (val)?(LPC_ADC->INTEN |= (1<<1)):(LPC_ADC->INTEN &= ~(1<<1));
            break;
        case kADC_OVR:
            (val)?(LPC_ADC->INTEN |= (1<<2)):(LPC_ADC->INTEN &= ~(1<<2));
            break;
        default:
            break;
    }
    NVIC_EnableIRQ(ADC_SEQA_IRQn);
    NVIC_EnableIRQ(ADC_SEQB_IRQn);
}

void ADC_ConfigSeq(uint32_t seq, uint32_t chl_bm)
{
    /* set mode to END OF SEQUENCE */
    uint32_t seq_ctrl = (chl_bm & 0xFFF) | (1<<30);
    
    LPC_ADC->SEQA_CTRL = seq_ctrl | 0x80000000;
}

void ADC_SoftSingleTrigger(uint32_t seq)
{
    LPC_ADC->SEQA_CTRL |= (1<<26);
    while((LPC_ADC->SEQA_GDAT & 0x80000000) == 0) {};
}

void ADC_SetContinueTrigger(uint32_t seq, bool val)
{
    (val)?(LPC_ADC->SEQA_CTRL |= (1<<27)):(LPC_ADC->SEQA_CTRL &= ~(1<<27));
}

uint16_t ADC_GetResult(uint32_t ch)
{
    return (LPC_ADC->DAT[ch] >> 4) & 0x0FFF;
}

uint32_t ADC_GetResultEx(uint32_t ch)
{
    return LPC_ADC->DAT[ch];
}



//! @}
