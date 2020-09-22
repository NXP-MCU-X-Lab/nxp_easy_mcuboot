/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "mrt.h"
#include "common.h"

static uint32_t fac_us;


void MRT_Init(uint32_t ch, uint32_t us)
{
    /* enable uart clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);

    /* get clock */
    fac_us = US_TO_COUNT(1, GetClock(kCoreClock));
    
    LPC_MRT->Channel[ch].CTRL = 0x00000000;
    LPC_MRT->Channel[ch].INTVAL = 0x80000000 | us*fac_us;

}

void MRT_SetValue(uint8_t ch, uint32_t val)
{
    LPC_MRT->Channel[ch].INTVAL = 0x80000000 | val;
}

void MRT_SetTime(uint32_t ch, uint32_t us)
{
    fac_us = US_TO_COUNT(1, GetClock(kCoreClock));
    MRT_SetValue(ch, fac_us * us);
}

uint32_t MRT_GetValue(uint32_t ch)
{
    uint32_t val;
    val = LPC_MRT->Channel[ch].TIMER & 0x7FFFFFFF;
    return val;
}



uint32_t MRT_GetTime(uint32_t ch)
{
    return MRT_GetValue(ch) / fac_us;
}

uint32_t MRT_SetIntMode(uint32_t ch, bool val)
{
    (val)?(LPC_MRT->Channel[ch].CTRL |= 0x01):(LPC_MRT->Channel[ch].CTRL &= ~0x01);
    NVIC_EnableIRQ(MRT_IRQn);
    return CH_OK;
}

