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

int main(void)
{
    DelayInit();
    
    UART_Init(UART0_RX_PB16_TX_PB17, 115200);
    printf("frdm_k64_example app\r\n");
    printf("main entry address:0x%08X\r\n", (uint32_t)main);
    while(1)
    {
        
    }
}


