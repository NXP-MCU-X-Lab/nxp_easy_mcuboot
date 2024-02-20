/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "uart.h"
#include "common.h"

#if defined(USART2)
static USART_Type* const UARTBases[] = {USART0, USART1, USART2};
#else
static USART_TypeDef* const UARTBases[] = {USART0, USART1};
#endif


int Putc(uint8_t data)
{
    UART_PutChar(HW_UART0, data);
    return 0;
}

int Getc(void)
{
    uint8_t ch;
    while(UART_GetChar(HW_UART0, &ch) != CH_OK);
    return ch;
}

uint32_t UART_GetChar(uint32_t instance, uint8_t *ch)
{
    if(UARTBases[instance]->STAT & (1<<0))
    {
        *ch = (UARTBases[instance]->RXDAT & 0xFF);
        return CH_OK;
    }
    return CH_ERR;
}



void UART_PutChar(uint32_t instance, uint8_t ch)
{
    while (!((UARTBases[instance]->STAT) & (1<<2))); // Wait for TX Ready
    UARTBases[instance]->TXDAT  = ch;             // Write the character to the TX buffer
}

uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val)
{
    (val)?(UARTBases[instance]->INTENSET = (1<<0)):(UARTBases[instance] ->INTENCLR = (1<<0));
    
    if(val)
    {
        switch(instance)
        {
            case 0:
                NVIC_EnableIRQ(USART0_IRQn);
                break;
            case 1:
                NVIC_EnableIRQ(USART1_IRQn);
                break;
            case 2:
                #if defined(USART2_IRQn)
                NVIC_EnableIRQ(UART2_IRQn);
                #endif
                break;
        }
    }
    return CH_OK;
}

#if defined(LPC82X)

void UART_SetBaudRate(uint32_t instance, uint32_t baud)
{
    int m, brg;
    
    UARTBases[instance]->CFG &= ~(1<<0);
    
    brg = GetClock(kCoreClock) / (16 * baud) - 1;
    m =   GetClock(kCoreClock) / ((16 * baud) / 256 * (brg + 1)) - 256;
    SYSCON->UARTFRGMULT = m; 
    UARTBases[instance]->BRG = brg;
    
    UARTBases[instance]->CFG |= (1<<0);
}

uint32_t UART_Init(uint32_t instance, uint32_t baudrate)
{
    /* enable uart clock */
    SYSCON->SYSAHBCLKCTRL |= (1<<14) | (1<<15) | (1<<16);
    
    /* uart clock div = 1(bypass) */
    SYSCON->UARTCLKDIV = 1;
    SYSCON->UARTFRGMULT = 0; 
    SYSCON->UARTFRGDIV = 0xFF;
    
    UART_SetBaudRate(instance, baudrate);
    
    UARTBases[instance]->CTL = (1<<8);
    UARTBases[instance]->STAT = 0xFFFF;
    UARTBases[instance]->CFG = (1<<2) | (1<<0);
    
    SetConsole(Putc, Getc);
    return CH_OK;
}

#else


uint32_t UART_Init(uint32_t MAP, uint32_t baudrate)
{
    int m, brg;
    
    /* enable uart clock */
    SYSCON->SYSAHBCLKCTRL0 |= (1<<14) | (1<<15);
    
    /* uart frg clock */
    SYSCON->FRG[0].FRGCLKSEL = 0x00;
    SYSCON->FRG[0].FRGMULT = 0; 
    SYSCON->FRG[0].FRGDIV = 255;
    SYSCON->FCLKSEL[0] = 0x01;      /* 0: FRO, 1:MAIN 2: FRG0, 4:FRO_DIV*/
    
    brg = GetClock(kCoreClock) / (16 * baudrate) - 1;
    brg = GetClock(kCoreClock) / (16 * baudrate) - 1;
    m =  256 * (GetClock(kCoreClock) / (16 * baudrate * (brg + 1))) - 256;
    
    SYSCON->FRG[0].FRGMULT = m; 
    USART0->BRG = brg;
    
    USART0->CTL = (1<<8);
    USART0->STAT = 0xFFFF;
    USART0->CFG = (1<<2) | (1<<0);
    USART0->OSR = 0x0F;
    

    SetConsole(Putc, Getc);
    return CH_OK;
}
#endif
