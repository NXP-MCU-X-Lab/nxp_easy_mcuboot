/**
  ******************************************************************************
  * @file    spi.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2015.6.21
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */

#include "common.h"
#include "spi.h"

static FLEXCOMM_Type* const FLEXCOMMBases[] = FLEXCOMM_BASE_PTRS;
SPI_Type* const SPIBases[] = SPI_BASE_PTRS;

#define SPI_ASSERTNUM_SSEL(n) ((~(1U << ((n) + 16))) & 0xF0000)
#define SPI_DEASSERTNUM_SSEL(n) (1U << ((n) + 16))
#define SPI_DEASSERT_ALL (0xF0000)

uint32_t SPI_SetClock(uint32_t instance)
{
    SetFlexCommClk(instance, 1);
    return GetClock(kFROHfClock);
}

void SPI_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t tmp, clk;

    clk = SPI_SetClock(instance);

    /* calculate baudrate */
    tmp = (clk / baud);
    
    LIB_TRACE("SPI input clock:%dHz div:%d\r\n", clk, tmp);
    
    SPIBases[instance]->DIV &= ~SPI_DIV_DIVVAL_MASK;
    SPIBases[instance]->DIV |= SPI_DIV_DIVVAL(tmp - 1);
}


uint32_t SPI_Init(uint32_t MAP, uint32_t baudrate)
{
    map_t * pq = (map_t*)&(MAP);
    
    uint32_t instance = pq->ip;
    
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK 
                            | SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK;

    /* select flexcomm to SPI */
    FLEXCOMMBases[instance]->PSELID &= ~FLEXCOMM_PSELID_PERSEL_MASK;
    FLEXCOMMBases[instance]->PSELID |= FLEXCOMM_PSELID_PERSEL(2);
    
    SPIBases[instance]->CFG &= ~SPI_CFG_ENABLE_MASK;
    
    SPI_SetBaudRate(instance, baudrate);
    
    /* add some delay */
    SPIBases[instance]->DLY = SPI_DLY_PRE_DELAY(0) | SPI_DLY_POST_DELAY(0) | SPI_DLY_TRANSFER_DELAY(0) | SPI_DLY_FRAME_DELAY(0);
    
    /* enable and clear FIFO */
    SPIBases[instance]->FIFOCFG |= SPI_FIFOCFG_ENABLETX_MASK | SPI_FIFOCFG_ENABLERX_MASK | SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    
    /* clear FIFO error */
    SPIBases[instance]->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    SPIBases[instance]->CFG = SPI_CFG_ENABLE_MASK | SPI_CFG_MASTER_MASK;
    
    return 0;
}


uint32_t SPI_ReadWriteEx(uint32_t instance, uint32_t data, uint16_t cs, uint32_t cs_states)
{
    SPIBases[instance]->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    
    while((SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK) == 0);
    SPIBases[instance]->FIFOWR = (SPI_DEASSERT_ALL & SPI_ASSERTNUM_SSEL(cs)) | SPI_FIFOWR_TXDATA(data) |  SPI_FIFOWR_LEN(7) | SPI_FIFOWR_EOT(cs_states);
    
    while((SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0);
    return (SPIBases[instance]->FIFORD & SPI_FIFORD_RXDATA_MASK);
}


uint32_t SPI_ReadFIFO(uint32_t instance, uint8_t *buf, uint32_t len)
{
    uint32_t rd_count = 0;
    uint32_t wr_count = len;
    //uint32_t dummy;
    
    //SPIBases[instance]->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    //SPIBases[instance]->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    while(1)
    {
        if(rd_count == len)
        {
            break;
        }
        
        if(wr_count)
        {
            if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK)
            {
                SPIBases[instance]->FIFOWR = (SPI_DEASSERT_ALL & SPI_ASSERTNUM_SSEL(0)) | SPI_FIFOWR_TXDATA(0xFF) |  SPI_FIFOWR_LEN(7) | SPI_FIFOWR_EOT(0);
                wr_count--;
            }
        }

        if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
        {
            *buf++ = SPIBases[instance]->FIFORD;
            rd_count++;
        }
    }
    return CH_OK;
}


uint32_t SPI_WriteFIFO(uint32_t instance, uint8_t *buf, uint32_t len)
{
    volatile uint32_t dummy;
    uint32_t rd_count = 0;
    uint32_t wr_count = len;
    
    while(1)
    {
        if(rd_count == len)
        {
            break;
        }

        if(wr_count)
        {
            if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK)
            {
                SPIBases[instance]->FIFOWR = (SPI_DEASSERT_ALL & SPI_ASSERTNUM_SSEL(0)) | SPI_FIFOWR_TXDATA(*buf++) |  SPI_FIFOWR_LEN(7) | SPI_FIFOWR_EOT(0);
                wr_count--;
            }
        }

        if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
        {
            dummy = SPIBases[instance]->FIFORD;
            rd_count++;
        }
    }
    return CH_OK;
}

uint32_t SPI_ReadWriteFIFO(uint32_t instance, uint8_t *in_buf, uint8_t *out_buf, uint32_t len)
{
    volatile uint32_t dummy;
    volatile uint32_t rd_count = 0;
    volatile uint32_t wr_count = len;
    
    while(1)
    {
        if(rd_count == len)
        {
            break;
        }

        if(wr_count)
        {
            if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK)
            {
                SPIBases[instance]->FIFOWR = (SPI_DEASSERT_ALL & SPI_ASSERTNUM_SSEL(0)) | SPI_FIFOWR_TXDATA(*out_buf++) |  SPI_FIFOWR_LEN(7) | SPI_FIFOWR_EOT(0);
                wr_count--;
            }
        }

        if(SPIBases[instance]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
        {
            if(in_buf != NULL)
            {
                *in_buf++ = SPIBases[instance]->FIFORD;
            }
            else
            {
                dummy = SPIBases[instance]->FIFORD;
            }
            rd_count++;
        }
    }
    return CH_OK;
}


uint32_t SPI_ReadWrite(uint32_t instance, uint32_t data)
{
    return SPI_ReadWriteEx(instance, data, 0, 1);
}



