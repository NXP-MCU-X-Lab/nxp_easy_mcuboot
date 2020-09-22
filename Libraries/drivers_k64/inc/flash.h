/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CH_LIB_IFLASH_H__
#define __CH_LIB_IFLASH_H__

#include <stdint.h>


//!< API 功能接口
void FLASH_Init(void);
uint32_t FLASH_GetSectorSize(void);
uint8_t FLASH_WriteSector(uint32_t addr, const uint8_t *buf, uint32_t len);
uint8_t FLASH_EraseSector(uint32_t addr);
uint32_t FLASH_Test(uint32_t startAddr, uint32_t len);
uint32_t FLASH_GetProgramCmd(void);

#endif

