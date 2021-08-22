/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef __BL_CFG_H__
#define __BL_CFG_H__

#include "common.h"

/* Base address of user application */
#define APPLICATION_BASE            (0x60040000UL)//Application code offset address
#define MAX_PACKET_LEN              (512)//Strongly recommend to use NorFlash page size
#define BL_TIMEOUT_MS               (1000)//Bootloader timeout before jump app
#define TARGET_FLASH_SIZE           (64*1024UL*1024UL)
#define UART_INSTANCE               LPUART1

#endif
