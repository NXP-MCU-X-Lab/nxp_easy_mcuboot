/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "dma.h"
#include "common.h"

/*! @name CHANNEL_XFERCFG - Transfer configuration register for DMA channel . */
#define DMA_CHANNEL_XFERCFG_SETINTA_MASK         (0x10U)
#define DMA_CHANNEL_XFERCFG_SETINTA_SHIFT        (4U)
#define DMA_CHANNEL_XFERCFG_SETINTA(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SETINTA_SHIFT)) & DMA_CHANNEL_XFERCFG_SETINTA_MASK)
#define DMA_CHANNEL_XFERCFG_SETINTB_MASK         (0x20U)
#define DMA_CHANNEL_XFERCFG_SETINTB_SHIFT        (5U)
#define DMA_CHANNEL_XFERCFG_SETINTB(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SETINTB_SHIFT)) & DMA_CHANNEL_XFERCFG_SETINTB_MASK)
#define DMA_CHANNEL_XFERCFG_WIDTH_MASK           (0x300U)
#define DMA_CHANNEL_XFERCFG_WIDTH_SHIFT          (8U)
#define DMA_CHANNEL_XFERCFG_WIDTH(x)             (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_WIDTH_SHIFT)) & DMA_CHANNEL_XFERCFG_WIDTH_MASK)
#define DMA_CHANNEL_XFERCFG_SRCINC_MASK          (0x3000U)
#define DMA_CHANNEL_XFERCFG_SRCINC_SHIFT         (12U)
#define DMA_CHANNEL_XFERCFG_SRCINC(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SRCINC_SHIFT)) & DMA_CHANNEL_XFERCFG_SRCINC_MASK)
#define DMA_CHANNEL_XFERCFG_DSTINC_MASK          (0xC000U)
#define DMA_CHANNEL_XFERCFG_DSTINC_SHIFT         (14U)
#define DMA_CHANNEL_XFERCFG_DSTINC(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_DSTINC_SHIFT)) & DMA_CHANNEL_XFERCFG_DSTINC_MASK)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK       (0x3FF0000U)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT      (16U)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT(x)         (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT)) & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK)

/* DMA SRAM table - this can be optionally used with the Chip_DMA_SetSRAMBase()
   function if a DMA SRAM table is needed. */
ALIGN(512) DMA_Desc_t DMA_DescTbl[MAX_DMA_CHANNEL];

void DMA_Init(void)
{
    /* enable uart clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<29);

    /* set descriptor table */
    LPC_DMA->SRAMBASE = (uint32_t)DMA_DescTbl;
    
    /* enable dma peripheral */
    LPC_DMA->CTRL |= (1<<0);
}

void DMA_SetChlIntMode(uint32_t ch, bool val)
{
    (val)?(LPC_DMA->INTENSET0 = (1 << ch)):(LPC_DMA->INTENCLR0 = (1 << ch));
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_SWTrigger(uint32_t ch)
{
    LPC_DMA->CHANNEL[ch].XFERCFG |= (1<<2);
}

bool DMA_IsChComplete(uint32_t ch)
{
    if(LPC_DMA->INTA0 & (1<<ch))
    {
        LPC_DMA->INTA0 = LPC_DMA->INTA0 & (1<<ch);
        return true;
    }
    else
    {
        return false;
    }
}

void DMA_SetChlLink(uint8_t ch, DMA_Desc_t *list)
{
    DMA_Desc_t *pDesc = (DMA_Desc_t *) LPC_DMA->SRAMBASE;
    
    pDesc[ch].next = (uint32_t)list;
    LPC_DMA->CHANNEL[ch].XFERCFG |= (1<<1);
}

void DMA_SetupChl(DMA_ChlSetup_t *setup)
{
    uint8_t sInc, dInc;
    DMA_Desc_t *pDesc = (DMA_Desc_t *) LPC_DMA->SRAMBASE;
    DMA_Desc_t desc;
    
    desc.next = NULL;
    
    if(setup->transferCnt == 0)
    {
        return;
    }
    
    /* the actuall sAddr and dAddr is the end address */
    desc.src = setup->sAddr +  setup->dataWidth * setup->sAddrInc * (setup->transferCnt - 1);
    desc.dest = setup->dAddr +  setup->dataWidth * setup->dAddrInc * (setup->transferCnt - 1);
    pDesc[setup->chl] = desc;
    
    sInc = setup->sAddrInc == 4 ? 3 : setup->sAddrInc;
    dInc = setup->dAddrInc == 4 ? 3 : setup->dAddrInc;
    
    /* DMA_CHANNEL_XFERCFG6_SRCINC : fixme when 4xwidth, it will be error */
    LPC_DMA->CHANNEL[setup->chl].XFERCFG = DMA_CHANNEL_XFERCFG_SETINTA_MASK | 
                                        DMA_CHANNEL_XFERCFG_XFERCOUNT(setup->transferCnt - 1) | 
                                        DMA_CHANNEL_XFERCFG_WIDTH(setup->dataWidth/2) | 
                                        DMA_CHANNEL_XFERCFG_SRCINC(sInc) |  
                                        DMA_CHANNEL_XFERCFG_DSTINC(dInc);
    
    (setup->isPeriph)?(LPC_DMA->CHANNEL[setup->chl].CFG = (1<<0)):(LPC_DMA->CHANNEL[setup->chl].CFG = 0);
    
    /* enable channel */
    LPC_DMA->ENABLESET0 = (1 << setup->chl);
    LPC_DMA->SETVALID0 = (1 << setup->chl);
}


