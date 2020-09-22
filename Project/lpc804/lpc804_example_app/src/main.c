/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "flash.h"


int main(void)
{
    uint8_t c;
    SystemCoreClockUpdate();
    
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1<<7) | (1<<14) | (1<<18);
    
    DelayInit();
    DelayMs(10);
 
    /* UART: RX--P0_0, TX--P0_1 */
    SWM_Config(0, 1, 0);
    SWM_Config(0, 0, 8);
   
    UART_Init(HW_UART0, 115200);
    
    printf("example app: main address:0x%X\r\n", (uint32_t)main);
    printf("CoreClock:%dHz\r\n", GetClock(kCoreClock));

    while(1)
    {
        if(UART_GetChar(HW_UART0, &c) == CH_OK)
        {
            UART_PutChar(HW_UART0, c);
        }
    }
}

