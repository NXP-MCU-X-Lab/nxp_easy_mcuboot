/**
  ******************************************************************************
  * @file    flash.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.6
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_IFLASH_H__
#define __CH_LIB_LPC_IFLASH_H__

#include <stdint.h>


//!< API 功能接口
void FLASH_Init(void);
uint32_t FLASH_GetSectorSize(void);
uint32_t FLASH_GetPageSize(void);
uint8_t FLASH_ErasePage(uint32_t addr);
uint8_t FLASH_WritePage(uint32_t addr, const uint8_t *buf);
uint8_t FLASH_EraseSector(uint32_t addr);
uint32_t FLASH_Test(uint32_t startAddr, uint32_t len);
uint32_t ISP_GetUID(void);
uint32_t ISP_Reinvoke(uint8_t mode);

#endif

