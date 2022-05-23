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
#include "lpuart.h"
#include "flash.h"
#include "scg.h"


int main(void)
{
    uint8_t c;
    DelayInit();
    
    SCG->FIRCDIV =   SCG_FIRCDIV_FIRCDIV2(1); 
    SetPinMux(HW_GPIOB, 0, 2);
    SetPinMux(HW_GPIOB, 1, 2);
    LPUART_Init(HW_LPUART0, 115200);

    GPIO_Init(HW_GPIOD, 10, kGPIO_OPPH);
    while(1)
    {
        GPIO_PinToggle(HW_GPIOD, 10);
        DelayMs(500);
    }
    
}



void HardFault_Handler(void)
{
    NVIC_SystemReset();
    
    while(1)
    {
    }
}


