/**
  ******************************************************************************
  * @file    scg.h
  * @author  YANDLD
  * @version V3.0
  * @date    2016.05.27
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_SCG_H__
#define __CH_LIB_SCG_H__

#include <stdint.h>

#define IRC_48M 0
#define IRC_96M 1

typedef enum
{
    kSCG_SLOW = 0,
    kSCG_SOSC = 1,
    kSCG_SIRC = 2,
    kSCG_FIRC = 3,
    kSCG_PLL = 6,
}SCG_Mode_t;

uint32_t ClockSetup(uint32_t opt);

#endif


