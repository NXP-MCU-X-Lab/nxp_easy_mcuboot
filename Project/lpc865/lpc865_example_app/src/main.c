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

#define LED_RED_PORT        (0)
#define LED_RED_PIN         (12)

int main(void)
{
    uint8_t c;
    SystemCoreClockUpdate();
    
    SYSCON->SYSAHBCLKCTRL0 |= (1<<7) | (1<<14) | (1<<18);
    
    CLOCK_SetFroOscFreq(36000U);
    SystemCoreClock = 24*1000*1000;
    
    DelayInit();
    DelayMs(10);
 
    GPIO_Init(LED_RED_PORT, LED_RED_PIN, kGPIO_OPPL);
    
    /* UART: P1_16 P1_17 */
    SWM_Config(0, 32+17, 0);
    SWM_Config(0, 32+16, 8);
   
    UART_Init(HW_UART0, 115200);
    
    printf("example app: main address:0x%X\r\n", (uint32_t)main);
    printf("CoreClock:%dHz\r\n", GetClock(kCoreClock));

    SysTick_SetTime(100*1000);
    SysTick_SetIntMode(true);
    
    while(1)
    {
        if(UART_GetChar(HW_UART0, &c) == CH_OK)
        {
            UART_PutChar(HW_UART0, c);
        }
    }
}


void SysTick_Handler(void)
{
    GPIO_PinToggle(LED_RED_PORT, LED_RED_PIN);
}


