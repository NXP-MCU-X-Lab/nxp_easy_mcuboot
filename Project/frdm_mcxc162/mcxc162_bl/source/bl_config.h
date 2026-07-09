/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BL_CONFIG_H_
#define BL_CONFIG_H_

#define BL_BOOT_BASE          (0x00000000u)
#define BL_BOOT_SIZE          (0x00004000u)
#define BL_APP_BASE           (0x00004000u)
#define BL_BACKUP_BASE        (0x0000A000u)
#define BL_SLOT_SIZE          (0x00006000u)
#define BL_FLASH_SECTOR_SIZE  (0x00002000u)
#define BL_FLASH_PHRASE_SIZE  (16u)
#define BL_FLASH_PAGE_SIZE    (128u)

#define BL_SRAM_BASE          (0x20000000u)
#define BL_SRAM_SIZE          (0x00003000u)

#define BL_TIMEOUT_MS         (300u)
#define BL_SYSTICK_MS         (10u)
#define BL_UART_BAUDRATE      (115200u)

#ifndef BL_ENABLE_BOOT_LOG
#define BL_ENABLE_BOOT_LOG    (1u)
#endif

#define BL_STATUS_SUCCESS     (0u)
#define BL_STATUS_FAIL        (1u)

#endif /* BL_CONFIG_H_ */
