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
    
    SCG->FIRCDIV =   SCG_FIRCDIV_FIRCDIV2(1) | SCG_FIRCDIV_FIRCDIV1(1);  
    SetPinMux(HW_GPIOB, 0, 2);
    SetPinMux(HW_GPIOB, 1, 2);
    LPUART_Init(HW_LPUART0, 115200);

    printf("I am application, main address:0x%X\r\n", main);
    GPIO_Init(HW_GPIOB, 5, kGPIO_OPPH);
    while(1)
    {
        GPIO_PinToggle(HW_GPIOB, 5);
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


