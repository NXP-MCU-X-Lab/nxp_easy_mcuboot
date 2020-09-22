/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "spi.h"


LPC_SPI_TypeDef* const SPIBases[] = {LPC_SPI0, LPC_SPI1};

#define SPI_STAT_BITMASK            (0x1FF)					/** SPI STAT Register BitMask */
#define SPI_STAT_RXRDY              (1 << 0)				/** Receiver Ready Flag */
#define SPI_STAT_TXRDY              (1 << 1)				/** Transmitter Ready Flag */
#define SPI_STAT_RXOV               (1 << 2)				/** Receiver Overrun interrupt flag */
#define SPI_STAT_TXUR               (1 << 3)				/** Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_STAT_SSA                (1 << 4)				/** Slave Select Assert */
#define SPI_STAT_SSD                (1 << 5)				/** Slave Select Deassert */
#define SPI_STAT_STALLED            (1 << 6)				/** Stalled status flag */
#define SPI_STAT_EOT                (1 << 7)				/** End Transfer flag */
#define SPI_STAT_MSTIDLE            (1 << 8)				/** Idle status flag */

#define SPI_ASSERTNUM_SSEL(n) ((~(1U << ((n) + 16))) & 0xF0000)
#define SPI_DEASSERTNUM_SSEL(n) (1U << ((n) + 16))
#define SPI_DEASSERT_ALL (0xF0000)

uint32_t SPI_Init(uint32_t instance, uint32_t baudrate)
{
    int div;
    
    /* enable uart clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11) | (1<<12);
    
    SPIBases[instance]->CFG = 0x00;
    div = (GetClock(kCoreClock) / baudrate) - 1;
    if(div < 0)
    {
        div = 0;
    }
    SPIBases[instance]->DIV = div;
    
    /* 8 bit frame */
    SPIBases[instance]->TXCTL = (0x07<<24);
    
    /* enable SPI and set to master mode */
    SPIBases[instance]->CFG = (1<<0) | (1<<2);
    
    return CH_OK;
}


uint32_t SPI_ReadWriteFIFO(uint32_t instance, uint8_t *in_buf, uint8_t *out_buf, uint32_t len)
{
    uint32_t stat;
    int tx_cnt, rx_cnt;
    volatile uint8_t dummy;
    tx_cnt = rx_cnt = len;
    
    while(1)
    {
        stat = SPIBases[instance]->STAT;
        if(stat &  SPI_STAT_TXRDY && tx_cnt)
        {
            SPIBases[instance]->TXDAT = (out_buf != NULL)?(*out_buf++):(0xFF);
            tx_cnt --;
        }
        
        if(stat & SPI_STAT_RXRDY && rx_cnt)
        {
            (in_buf != NULL)?(*in_buf++ = SPIBases[instance]->RXDAT):(dummy = SPIBases[instance]->RXDAT);
            rx_cnt--;
        }
        
        if(tx_cnt == 0 && rx_cnt == 0)
        {
            return CH_OK;
        }
    }
}

uint32_t SPI_ReadWriteEx(uint32_t instance, uint32_t data, uint16_t cs, uint32_t cs_states)
{
    uint8_t rx;
    SPI_ReadWriteFIFO(instance, &rx, (uint8_t*)&data, 1);
    return rx;
}


uint32_t SPI_ReadWrite(uint32_t instance, uint32_t data)
{
    return SPI_ReadWriteEx(instance, data, 0, 1);
}



