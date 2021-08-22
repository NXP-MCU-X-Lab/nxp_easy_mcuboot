/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdint.h>
#include "fsl_romapi.h"


//!< API 功能接口
void FLASH_Init(void);
void FLASH_DeInit(void);
uint32_t FLASH_GetSectorSize(void);
status_t FLASH_WritePage(uint32_t addr, const uint8_t *buf, uint32_t len);
status_t FLASH_EraseSector(uint32_t addr);
status_t FLASH_Read(uint32_t addr, const uint8_t *buf, uint32_t len);
uint32_t FLASH_Test(uint32_t startAddr, uint32_t len);
uint32_t FLASH_GetProgramCmd(void);

#endif

