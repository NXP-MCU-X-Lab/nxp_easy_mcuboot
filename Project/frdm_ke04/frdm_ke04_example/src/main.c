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


static void ICS_FEE_20M(void)
{
    SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(1); /* Update system prescalers */
    /* Switch to FEE Mode */
    /* ICS->C2: BDIV=0,LP=0 */
    ICS->C2 &= (uint8_t)~(uint8_t)((ICS_C2_BDIV(0x07) | ICS_C2_LP_MASK));
    /* OSC->CR: OSCEN=1,??=0,OSCSTEN=0,OSCOS=1,??=0,RANGE=1,HGO=0,OSCINIT=0 */
    OSC->CR = (OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_RANGE_MASK);
    /* ICS->C1: CLKS=0,RDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    ICS->C1 = (ICS_C1_CLKS(0x00) | ICS_C1_RDIV(0x03) | ICS_C1_IRCLKEN_MASK);
    while((ICS->S & ICS_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((ICS->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
    }
    
    SystemCoreClock = 20*1000*1000;
}

int main(void)
{
    ICS_FEE_20M();
    DelayInit();
    DelayMs(10);
    
    GPIO_Init(HW_GPIOC, 5, kGPIO_OPPH);
    UART_Init(HW_UART0, 115200);
    
    printf("frdm-ke04 example app\r\n");
    printf("main address:0x%X\r\n", (uint32_t)main);
    

    while(1)
    {
        printf("frdm_ke04 example app\r\n");
        DelayMs(500);
        GPIO_PinToggle(HW_GPIOC, 5);
    }
    
}
