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
    SYSCON->SYSAHBCLKCTRL0 |= (1<<18);
    uint32_t addr = NULL;

    switch(pin)
    {
        case 0:
            addr = (uint32_t)&IOCON->PIO[0];
            break;
        case 1:
            addr = (uint32_t)&IOCON->PIO[1];
            break;
        case 2:
            addr = (uint32_t)&IOCON->PIO[2];
            break;
        case 3:
            addr = (uint32_t)&IOCON->PIO[3];
            break;
        case 4:
            addr = (uint32_t)&IOCON->PIO[4];
            break;
        case 5:
            addr = (uint32_t)&IOCON->PIO[5];
            break;
//        case 6:
//            addr = (uint32_t)&IOCON->PIO[6];
//            break;
        case 7:
            addr = (uint32_t)&IOCON->PIO[7];
            break;
        case 8:
            addr = (uint32_t)&IOCON->PIO[8];
            break;
        case 9:
            addr = (uint32_t)&IOCON->PIO[9];
            break;
        case 10:
            addr = (uint32_t)&IOCON->PIO[10];
            break;
        case 11:
            addr = (uint32_t)&IOCON->PIO[11];
            break;
        case 12:
            addr = (uint32_t)&IOCON->PIO[12];
            break;
        case 13:
            addr = (uint32_t)&IOCON->PIO[13];
            break;
        case 14:
            addr = (uint32_t)&IOCON->PIO[14];
            break;
        case 15:
            addr = (uint32_t)&IOCON->PIO[15];
            break;
        case 17:
            addr = (uint32_t)&IOCON->PIO[17];
            break;
        case 18:
            addr = (uint32_t)&IOCON->PIO[18];
            break;
        case 19:
            addr = (uint32_t)&IOCON->PIO[19];
            break;
        case 20:
            addr = (uint32_t)&IOCON->PIO[20];
            break;
        case 21:
            addr = (uint32_t)&IOCON->PIO[21];
            break;
        case 22:
            addr = (uint32_t)&IOCON->PIO[22];
            break;
        case 23:
            addr = (uint32_t)&IOCON->PIO[23];
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
        GPIO->DIRSET[0] = (1<<pin);
    }
    else
    {
        GPIO->DIRCLR[0] = (1<<pin);
    }
}



void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    if(data)
    {
        GPIO->SET[0] = (1<<pin);
    }
    else
    {
        GPIO->CLR[0] = (1<<pin);
    }
}

uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return ((GPIO->PIN[0] >> pin) & 0x01);
}

void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    GPIO->NOT[0] = (1<<pin);
}

int GPIO_SetIntMode(uint32_t socket, GPIO_Int_t mode, bool val)
{
    /* all socket use edge sensitive */
    
    PINT->ISEL &= ~(1<<socket);
    
    switch(mode)
    {
        case kGPIO_Int_RE:
            (val)?(PINT->IENR |= (1<<socket)):(PINT->IENR &= ~(1<<socket));
            break;
        case kGPIO_Int_FE:
            (val)?(PINT->IENF |= (1<<socket)):(PINT->IENF &= ~(1<<socket));
            break;
        case kGPIO_Int_EE:
            (val)?(PINT->IENR |= (1<<socket)):(PINT->IENR &= ~(1<<socket));
            (val)?(PINT->IENF |= (1<<socket)):(PINT->IENF &= ~(1<<socket));
            break;
    }
    PINT->IST = 0xFF;
    NVIC_EnableIRQ((IRQn_Type)(PIN_INT0_IRQn + socket));
    return CH_OK;
}

uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode)
{
    /* enable GPIO clock */
    SYSCON->SYSAHBCLKCTRL0 |= (1<<6);
    
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
