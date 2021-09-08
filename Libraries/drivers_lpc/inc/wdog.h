/**
  ******************************************************************************
  * @file    wdog.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.2.16
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_WDOG_H__
#define __CH_LIB_WDOG_H__

#include <stdint.h>
#include <stdbool.h>


/* API */
void WDOG_Init(uint32_t ms);
void WDOG_Refresh(void);
uint32_t WDOG_GetValue(void);
uint32_t WDOG_GetResetCnt(void);
void WDOG_Disable(void);

#endif

