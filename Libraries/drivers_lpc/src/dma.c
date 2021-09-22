/**
  ******************************************************************************
  * @file    dma.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */

#include "dma.h"
#include "common.h"




/* DMA SRAM table - this can be optionally used with the Chip_DMA_SetSRAMBase()
   function if a DMA SRAM table is needed. */
ALIGN(512) DMA_Desc_t DMA_DescTbl[MAX_DMA_CHANNEL];

void DMA_Init(void)
{
    /* enable dma clock gate */
    #if defined(SYSCON_AHBCLKCTRL_DMA0_MASK)
    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_DMA0_MASK;
    #else
    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_DMA_MASK;
    #endif
    
    /* set descriptor table */
    DMA0->SRAMBASE = (uint32_t)DMA_DescTbl;
    
    /* enable dma peripheral */
    DMA0->CTRL |= DMA_CTRL_ENABLE_MASK;
}

void DMA_SetChlIntMode(uint32_t chl, bool val)
{
    (val)?(DMA0->COMMON[0].INTENSET = (1 << chl)):(DMA0->COMMON[0].INTENCLR = (1 << chl));
    NVIC_EnableIRQ(DMA0_IRQn);
}

void DMA_SWTrigger(uint32_t chl)
{
    DMA0->CHANNEL[chl].XFERCFG |= DMA_CHANNEL_XFERCFG_SWTRIG_MASK;
}

void DMA_SetChlLink(uint8_t chl, DMA_Desc_t *list)
{
    DMA_Desc_t *p = (DMA_Desc_t *) DMA0->SRAMBASE;
    
    p[chl].next = (uint32_t)list;
    DMA0->CHANNEL[chl].XFERCFG |= DMA_CHANNEL_XFERCFG_RELOAD_MASK;
}

uint32_t DMA_GetTransferCnt(uint8_t chl)
{
    return ((DMA0->CHANNEL[chl].XFERCFG & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >> DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT) + 1;
}

void DMA_CopyDesc(uint8_t ch, DMA_Desc_t *desc)
{
    DMA_Desc_t *p = (DMA_Desc_t *) DMA0->SRAMBASE;
    *desc = p[ch];
}

DMA_Desc_t *DMA_GetDesc(uint8_t ch)
{
    DMA_Desc_t *p = (DMA_Desc_t *) DMA0->SRAMBASE;
    return &p[ch];
}

void DMA_SetupChl(DMA_ChlSetup_t *setup)
{
    uint8_t sInc, dInc;
    DMA_Desc_t *p = (DMA_Desc_t *) DMA0->SRAMBASE;
    DMA_Desc_t desc;
    
    desc.next = NULL;
    
    if(setup->transferCnt == 0)
    {
        return;
    }
    
    /* the actuall sAddr and dAddr is the end address */
    desc.src = setup->sAddr +  setup->dataWidth * setup->sAddrInc * (setup->transferCnt - 1);
    desc.dest = setup->dAddr +  setup->dataWidth * setup->dAddrInc * (setup->transferCnt - 1);
    p[setup->chl] = desc;
    
    sInc = setup->sAddrInc == 4 ? 3 : setup->sAddrInc;
    dInc = setup->dAddrInc == 4 ? 3 : setup->dAddrInc;
    
    /* DMA_CHANNEL_XFERCFG6_SRCINC : fixme when 4xwidth, it will be error */
    DMA0->CHANNEL[setup->chl].XFERCFG = DMA_CHANNEL_XFERCFG_SETINTA_MASK | 
                                        DMA_CHANNEL_XFERCFG_XFERCOUNT(setup->transferCnt - 1) | 
                                        DMA_CHANNEL_XFERCFG_WIDTH(setup->dataWidth/2) | 
                                        DMA_CHANNEL_XFERCFG_SRCINC(sInc) |  
                                        DMA_CHANNEL_XFERCFG_DSTINC(dInc);
    
    (setup->isPeriph)?(DMA0->CHANNEL[setup->chl].CFG = DMA_CHANNEL_CFG_PERIPHREQEN_MASK):(DMA0->CHANNEL[setup->chl].CFG = 0);
    
    /* enable channel */
    DMA0->COMMON[0].ENABLESET = (1 << setup->chl);
    DMA0->COMMON[0].SETVALID  = (1 << setup->chl);
}


