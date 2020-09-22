/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "gpio.h"


void SetPinPull(uint32_t instance, uint32_t pin, uint32_t val)
{
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1<<18);
    uint32_t addr = NULL;

    switch(pin)
    {
        case 0:
            addr = (uint32_t)&LPC_IOCON->PIO0_0;
            break;
        case 1:
            addr = (uint32_t)&LPC_IOCON->PIO0_1;
            break;
        case 2:
            addr = (uint32_t)&LPC_IOCON->PIO0_2;
            break;
        case 3:
            addr = (uint32_t)&LPC_IOCON->PIO0_3;
            break;
        case 4:
            addr = (uint32_t)&LPC_IOCON->PIO0_4;
            break;
        case 5:
            addr = (uint32_t)&LPC_IOCON->PIO0_5;
            break;
//        case 6:
//            addr = (uint32_t)&LPC_IOCON->PIO0_6;
//            break;
        case 7:
            addr = (uint32_t)&LPC_IOCON->PIO0_7;
            break;
        case 8:
            addr = (uint32_t)&LPC_IOCON->PIO0_8;
            break;
        case 9:
            addr = (uint32_t)&LPC_IOCON->PIO0_9;
            break;
        case 10:
            addr = (uint32_t)&LPC_IOCON->PIO0_10;
            break;
        case 11:
            addr = (uint32_t)&LPC_IOCON->PIO0_11;
            break;
        case 12:
            addr = (uint32_t)&LPC_IOCON->PIO0_12;
            break;
        case 13:
            addr = (uint32_t)&LPC_IOCON->PIO0_13;
            break;
        case 14:
            addr = (uint32_t)&LPC_IOCON->PIO0_14;
            break;
        case 15:
            addr = (uint32_t)&LPC_IOCON->PIO0_15;
            break;
        case 17:
            addr = (uint32_t)&LPC_IOCON->PIO0_17;
            break;
        case 18:
            addr = (uint32_t)&LPC_IOCON->PIO0_18;
            break;
        case 19:
            addr = (uint32_t)&LPC_IOCON->PIO0_19;
            break;
        case 20:
            addr = (uint32_t)&LPC_IOCON->PIO0_20;
            break;
        case 21:
            addr = (uint32_t)&LPC_IOCON->PIO0_21;
            break;
        case 22:
            addr = (uint32_t)&LPC_IOCON->PIO0_22;
            break;
        case 23:
            addr = (uint32_t)&LPC_IOCON->PIO0_23;
            break;
    }
    
    *((volatile uint32_t*)addr) &= ~(0x03<<3);
    switch(val)
    {
        case 0:
            *((volatile uint32_t*)addr) |= 0x01<<3;
            break;
        case 1:
            *((volatile uint32_t*)addr) |= 0x02<<3;
            break;
        default:
            break;
    }
}

void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir)
{
    if(dir)
    {
        LPC_GPIO_PORT->DIRSET0 = (1<<pin);
    }
    else
    {
        LPC_GPIO_PORT->DIRCLR0 = (1<<pin);
    }
}



void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    if(data)
    {
        LPC_GPIO_PORT->SET0 = (1<<pin);
    }
    else
    {
        LPC_GPIO_PORT->CLR0 = (1<<pin);
    }
}

uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return ((LPC_GPIO_PORT->PIN0 >> pin) & 0x01);
}

void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    LPC_GPIO_PORT->NOT0 = (1<<pin);
}

int GPIO_SetIntMode(uint32_t socket, GPIO_Int_t mode, bool val)
{
    /* all socket use edge sensitive */
    
    LPC_PIN_INT->ISEL &= ~(1<<socket);
    
    switch(mode)
    {
        case kGPIO_Int_RE:
            (val)?(LPC_PIN_INT->IENR |= (1<<socket)):(LPC_PIN_INT->IENR &= ~(1<<socket));
            break;
        case kGPIO_Int_FE:
            (val)?(LPC_PIN_INT->IENF |= (1<<socket)):(LPC_PIN_INT->IENF &= ~(1<<socket));
            break;
        case kGPIO_Int_EE:
            (val)?(LPC_PIN_INT->IENR |= (1<<socket)):(LPC_PIN_INT->IENR &= ~(1<<socket));
            (val)?(LPC_PIN_INT->IENF |= (1<<socket)):(LPC_PIN_INT->IENF &= ~(1<<socket));
            break;
    }
    LPC_PIN_INT->IST = 0xFF;
    NVIC_EnableIRQ((IRQn_Type)(PININT0_IRQn + socket));
    return CH_OK;
}

uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode)
{
    /* enable GPIO clock */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1<<6);
    
    switch(mode)
    {
        case kGPIO_IFT:
            GPIO_SetPinDir(instance, pin, 0);
            SetPinPull(0, pin, 2);
            break;
        case kGPIO_OPPH:
            GPIO_PinWrite(instance, pin, 1);
            GPIO_SetPinDir(instance, pin, 1);
            break;
        case kGPIO_OPPL:
            GPIO_PinWrite(instance, pin, 0);
            GPIO_SetPinDir(instance, pin, 1);
            break;
        case kGPIO_IPD:
            GPIO_SetPinDir(instance, pin, 0);
            SetPinPull(0, pin, 0);
            break;
        case kGPIO_IPU:
            GPIO_SetPinDir(instance, pin, 0);
            SetPinPull(0, pin, 1);
            break;
    }
    
    return CH_OK;
}
    

//! @}
