/**
  ******************************************************************************
  * @file    mrt.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
  
  
#include "mrt.h"
#include "common.h"

static uint32_t fac_us;

#if defined(LPC8XX)
typedef struct {
    struct {
        __IO uint32_t  INTVAL;
        __I  uint32_t  TIMER;
        __IO uint32_t  CTRL;
        __IO uint32_t  STAT;
    }CHANNEL[4];
}MRT_Type;
#endif

#if !defined(MRT_BASE)
    #if defined(LPC_MRT_BASE)
    #define MRT_BASE            LPC_MRT_BASE
    #elif defined(MRT0)
    #define MRT_BASE            MRT0_BASE
    #endif
#endif

#define MMRT                                      ((MRT_Type *)MRT_BASE)

void MRT_Init(uint32_t chl, uint32_t us)
{
    #if defined(SYSCON_AHBCLKCTRL_MRT_MASK)
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_MRT_MASK;
    #else
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_MRT0_MASK;
    #endif
    


    /* repect interrupt, disable timer */
    MMRT->CHANNEL[chl].CTRL = 0x00000000;

    /* get clock */
    fac_us = US_TO_COUNT(1, GetClock(kCoreClock));
    
    if(us*fac_us > 0x00FFFFFF)
    {
        MMRT->CHANNEL[chl].INTVAL = 0x80000000 | 0x00FFFFFF;
        LIB_TRACE("MRT overflow:%d%%\r\n", (us*fac_us*100)/0x00FFFFFF);
    }
    else
    {
        MMRT->CHANNEL[chl].INTVAL = 0x80000000 | us*fac_us;
    }
}

void MRT_SetValue(uint8_t chl, uint32_t val)
{
    MMRT->CHANNEL[chl].INTVAL = 0x80000000 | val;
}

uint32_t MRT_GetValue(uint32_t chl)
{
    return MMRT->CHANNEL[chl].TIMER & 0x7FFFFFFF;
}

void MRT_SetTime(uint32_t chl, uint32_t us)
{
    fac_us = US_TO_COUNT(1, GetClock(kCoreClock));
    MMRT->CHANNEL[chl].INTVAL = 0x80000000 | us*fac_us;
}

uint32_t MRT_GetTime(uint32_t chl)
{
    return MMRT->CHANNEL[chl].INTVAL/fac_us;
}

uint32_t MRT_SetIntMode(uint32_t chl, bool val)
{
    (val)?(MMRT->CHANNEL[chl].CTRL |= MRT_CHANNEL_CTRL_INTEN_MASK):(MMRT->CHANNEL[chl].CTRL &= ~MRT_CHANNEL_CTRL_INTEN_MASK);
    NVIC_EnableIRQ(MRT0_IRQn);
    return CH_OK;
}

