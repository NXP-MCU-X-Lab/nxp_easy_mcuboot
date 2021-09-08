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
#include "syscon.h"


int main(void)
{
    uint8_t c;
    SetFROClock(48*1000*1000, true);
    DelayInit();
    
    SetPinMux(HW_GPIO0, 0, 1);
    SetPinMux(HW_GPIO0, 1, 1);
    UART_Init(HW_UART0, 115200);

    printf("I am application, main1 address:0x%X\r\n", main);
    GPIO_Init(HW_GPIO0, 29, kGPIO_OPPH);
    while(1)
    {
        GPIO_PinToggle(HW_GPIO0, 29);
        DelayMs(1000);
    }
    
}



void HardFault_Handler(void)
{
    NVIC_SystemReset();
    
    while(1)
    {
    }
}


