/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BL_IMAGE_H_
#define BL_IMAGE_H_

#include <stdbool.h>
#include <stdint.h>

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

uint32_t BL_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t length);
uint32_t BL_Crc32Finish(uint32_t crc);
uint32_t BL_ImageCalcCrc32(uint32_t storage_base, uint32_t image_size);
bool BL_ImageValidate(uint32_t storage_base,
                      uint32_t execution_base,
                      uint32_t slot_size,
                      uint32_t sram_base,
                      uint32_t sram_size,
                      bl_image_info_t *info);

#endif /* BL_IMAGE_H_ */
