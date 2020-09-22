/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "ctimer.h"
#include "common.h"

volatile static uint32_t fac_us;


void CTIMER_PWM_Init(uint32_t instance, uint32_t pwm_chl, uint32_t freq)
{
    uint32_t period;
    /* enable CTIMER clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<25);

    /* get clock */
    fac_us = US_TO_COUNT(1, GetClock(kCoreClock));

    /* Setup the cimer mode and count select */
    LPC_CTIMER0->CTCR = 0;
    LPC_CTIMER0->PR = 0;
    
    LPC_CTIMER0->PWMC |= (1U << pwm_chl);
    LPC_CTIMER0->MCR = (1<<10);
    
    //LIB_TRACE("PWM input clock after prescaler:%d\r\n", (GetClock(kCoreClock)/(LPC_CTIMER0->PR + 1)));
    period = ((GetClock(kCoreClock)/(LPC_CTIMER0->PR + 1)) / freq) - 1;
    LPC_CTIMER0->MR[3] = period;
    
    LIB_TRACE("PWM MR[3](perdiod):%d\r\n", LPC_CTIMER0->MR[3]);
    
    /* initial duty */
    LPC_CTIMER0->MR[pwm_chl] = period/2;
    
    /* clear IT flags */
    LPC_CTIMER0->IR = 0xFF;
    
    LPC_CTIMER0->TCR |= 0x01;
}


void CTIMER_PWM_SetDuty(uint32_t instance, uint32_t pwm_chl, uint32_t duty)
{
    LPC_CTIMER0->MR[pwm_chl] = (LPC_CTIMER0->MR[3] * (10000 - duty)) / 10000;
}
