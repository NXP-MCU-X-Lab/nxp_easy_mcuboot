/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "uart.h"
#include "common.h"

#if defined(UART0)

extern bool _is_fitst_init;
uint8_t UART_DebugInstance;

const void* UARTBases[] = UART_BASES;

static const Reg_t UARTClkGate[] =
{
    {(void*)&(SIM->SCGC), SIM_SCGC_UART0_MASK, SIM_SCGC_UART0_SHIFT},
#ifdef SIM_SCGC_UART1_MASK
    {(void*)&(SIM->SCGC), SIM_SCGC_UART1_MASK, SIM_SCGC_UART1_SHIFT},
#endif
#ifdef SIM_SCGC_UART2_MASK
    {(void*)&(SIM->SCGC), SIM_SCGC_UART2_MASK, SIM_SCGC_UART2_SHIFT},
#endif
};

static const IRQn_Type UART_IRQTbl[] =
{
    (IRQn_Type)(UART0_IRQn),
#ifdef SIM_SCGC_UART1_MASK
    (IRQn_Type)(UART1_IRQn),
#endif
#ifdef SIM_SCGC_UART2_MASK
    (IRQn_Type)(UART2_IRQn),
#endif
};

static inline int Getc(void)
{
    uint8_t ch;
    while(UART_GetChar(UART_DebugInstance, &ch) != CH_OK);
    return ch;
}

static inline int Putc(uint8_t ch)
{
    UART_PutChar(UART_DebugInstance, ch);
    return ch;
}
/**
 * @brief  设置串口时钟源
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  opt : 无意义
 * @retval 时钟速度
 */
uint32_t UART_SetClock(uint32_t instance)
{
    uint32_t clk;
    clk = GetClock(kBusClock);

    return clk;
}

void UART_SetDebugInstance(uint32_t instance)
{
    UART_DebugInstance = instance;
}

uint32_t UART_GetDebugInstance(void)
{
    return UART_DebugInstance;
}


/**
 * @brief  设置串口波特率
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  baud : 通信波特率
 * @retval None
 */
void UART_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t clk;
    UART_Type *UARTx = (UART_Type*)UARTBases[instance];
    clk = UART_SetClock(instance);
    /* disable module */
    UARTx->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
     
    /* Calculate baud settings */
    uint16_t u16Sbr = (((clk)>>4) + (baud>>1))/baud;
    
    /* Save off the current value of the UARTx_BDH except for the SBR field */
    uint8_t u8Temp = UARTx->BDH & ~(UART_BDH_SBR_MASK);
    
    UARTx->BDH = u8Temp |  UART_BDH_SBR(u16Sbr >> 8);
    UARTx->BDL = (uint8_t)(u16Sbr & UART_BDL_SBR_MASK);
    
    /* enable moudle */
    UARTx->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
}

uint32_t UART_Init(uint32_t MAP, uint32_t baudrate)
{
    map_t * pq = (map_t*)&(MAP);
    
    uint32_t instance = pq->ip;
    REG_SET(UARTClkGate, instance);
    
    UART_SetBaudRate(pq->ip, baudrate);

    if(_is_fitst_init)
    {
        UART_DebugInstance = instance;
        SetConsole(Putc, Getc);
        _is_fitst_init = false;
    }
    return pq->ip;
}


uint32_t UART_DeInit(uint32_t instance)
{
    UART_Type *UARTx = (UART_Type*)UARTBases[instance];
    UARTx->BDH = 0;
    UARTx->BDL = 0;
    UARTx->C1 = 0;
    UARTx->C2 = 0;
    UARTx->C3 = 0;
    return CH_OK;
}

uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val)
{
    REG_SET(UARTClkGate, instance);
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    NVIC_EnableIRQ(UART_IRQTbl[instance]);
    switch(mode)
    {
        case kUART_IntTx:
            (val)?(UARTx->C2 |= UART_C2_TIE_MASK):(UARTx->C2 &= ~UART_C2_TIE_MASK);
            break;
        case kUART_IntRx:
            (val)?(UARTx->C2 |= UART_C2_RIE_MASK):(UARTx->C2 &= ~UART_C2_RIE_MASK);
            break;
        case kUART_IntIdleLine:
            (val)?(UARTx->C2 |= UART_C2_ILIE_MASK):(UARTx->C2 &= ~UART_C2_ILIE_MASK);
            break;
        default:
            break;
    }
    return CH_OK;
}


void UART_PutChar(uint32_t instance, uint8_t ch)
{
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    while(!(UARTx->S1 & UART_S1_TDRE_MASK));
    UARTx->D = (uint8_t)ch;
}

uint32_t UART_GetChar(uint32_t instance, uint8_t *ch)
{
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    
    if(UARTx->S1 & UART_S1_RDRF_MASK)
    {
        *ch = (uint8_t)(UARTx->D);	
        return CH_OK; 		  
    }
    if(UARTx->S1 & UART_S1_OR_MASK)
    {
        uint8_t *dummy = (uint8_t*)&UARTx->S1;
        (*dummy) |= UART_S1_OR_MASK;
    }
    return CH_ERR;
}


#endif
