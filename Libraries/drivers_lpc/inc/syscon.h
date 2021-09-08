/**
  ******************************************************************************
  * @file    syscon.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.05.31
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_SYSCON_H__
#define __CH_LIB_SYSCON_H__

#include <stdint.h>
#include <stdbool.h>

enum
{
    kPLLSrcFRO12M = 0,
    kPLLSrcCLKIN = 1,
};

uint32_t SetupSystemPLL(uint32_t in_clk, uint32_t out_clk);
uint32_t SetupUSBPLL(void);
void SetFlashClock(uint32_t clk);
void SetFROClock(uint32_t freq, bool val);
void SetClockOut(uint32_t opt);
void SetExtOSC(uint32_t freq, bool val);


#endif


