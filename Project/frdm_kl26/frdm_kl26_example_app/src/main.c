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
#include "mcuboot.h"
#include "bl_cfg.h"


int main(void)
{
    uint8_t c;
    DelayInit();
    
    UART_Init(UART0_RX_PA01_TX_PA02, 115200);    
    GPIO_Init(HW_GPIOE, 31, kGPIO_Mode_OPP);
    
    printf("frdm_kl26 example app\r\n");
    printf("main address:0x%X\r\n", (uint32_t)main);

    
    while(1)
    {
        DelayMs(500);
        GPIO_PinToggle(HW_GPIOE, 31);
    }
    
}

