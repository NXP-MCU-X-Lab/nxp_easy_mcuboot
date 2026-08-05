/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BL_IMAGE_H_
#define BL_IMAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "bl_trial.h"

#define BL_IMAGE_MAGIC         (0x4D435831u)
#define BL_IMAGE_HEADER_OFFSET (0x200u)
#define BL_IMAGE_HEADER_VER    (1u)

typedef struct
{
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t image_version;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t flags;
    uint32_t reserved[10];
} bl_image_header_t;

typedef struct
{
    uint32_t storage_base;
    uint32_t execution_base;
    uint32_t version;
    uint32_t image_size;
    uint32_t image_crc32;
    bool valid;
} bl_image_info_t;

typedef enum
{
    kBL_ImageActionStayInBootloader = 0,
    kBL_ImageActionBootApp,
    kBL_ImageActionRestoreBackup,
} bl_image_action_t;

uint32_t BL_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t length);
uint32_t BL_Crc32Finish(uint32_t crc);
uint32_t BL_ImageCalcCrc32(uint32_t storage_base, uint32_t image_size);
bool BL_ImageValidate(uint32_t storage_base,
                      uint32_t execution_base,
                      uint32_t slot_size,
                      uint32_t sram_base,
                      uint32_t sram_size,
                      bl_image_info_t *info);
bl_image_action_t BL_ImageSelectAction(bool app_valid,
                                      const bl_image_info_t *app,
                                      bool backup_valid,
                                      const bl_image_info_t *backup);
bool BL_ImageEraseRequestIsValid(uint32_t start_addr, uint32_t byte_cnt);

bool BL_TrialShouldRestore(bool trial_active,
                           bool watchdog_reset,
                           uint32_t image_crc32,
                           volatile bl_trial_state_t *state);

#endif /* BL_IMAGE_H_ */
