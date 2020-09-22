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
    DelayInit();
    DelayMs(10);
    
    UART_Init(HW_UART1, 115200);
    
    printf("frdm_ke02 example app\r\n");
    printf("main address:0x%X\r\n", (uint32_t)main);
    

    while(1)
    {
        printf("frdm_ke02 example app\r\n");
        DelayMs(500);
    }
    
}
