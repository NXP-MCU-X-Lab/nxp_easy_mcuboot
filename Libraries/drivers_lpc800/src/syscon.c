/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
  
#include "syscon.h"
#include "common.h"

#if defined(LPC82X)
/* 0:IRC 1:OSC 3:CLN_IN */
uint32_t SetupSystemPLL(uint8_t input_src, uint32_t in_clk, uint32_t out_clk)
{
    uint32_t m, p;
    
    /* enable PLL */
    LPC_SYSCON->PDRUNCFG &= ~((1<<7));
    
    if(input_src == 0x01)
    {
        LPC_SYSCON->PDRUNCFG &= ~((1<<5));
        LPC_IOCON->PIO0_8         &= ~(3 <<  3);          /* no pull-down/pull-up       */
        LPC_IOCON->PIO0_9         &= ~(3 <<  3);          /* no pull-down/pull-up       */
        LPC_SWM->PINENABLE0       &= ~(3 <<  6);          /* enable XTALIN/XTALOUT func.*/
    }
    
    LPC_SYSCON->SYSPLLCLKSEL = input_src;
    LPC_SYSCON->SYSPLLCLKUEN = 0x00;
    LPC_SYSCON->SYSPLLCLKUEN = 0x01;
    
    /* m: 1-32, p = 1,2,4,8 */
    for(m=1; m<32; m++)
    {
        for(p=1; p<=8; p*=2)
        {
            if(out_clk == m*in_clk)
            {
                if((2*p*out_clk) > 156000000 && (2*p*out_clk) < 320000000)
                {
                    LIB_TRACE("m:%d, p:%d\r\n", m, p);
                    LPC_SYSCON->SYSPLLCTRL = ((m-1)<<0) | ((p/2)<<5);
                    while((LPC_SYSCON->SYSPLLSTAT & 0x01) == 0);
                    return CH_OK;
                }
            }
        }
    }
    LIB_TRACE("system pll config failed\r\n");
    return CH_ERR;
}
#endif
