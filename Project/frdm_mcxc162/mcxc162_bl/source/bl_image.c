/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bl_image.h"

#include <stddef.h>
#include <string.h>

#include "bl_config.h"

#define BL_CRC32_INIT  (0xFFFFFFFFu)
#define BL_CRC32_POLY  (0xEDB88320u)
#define BL_CRC32_XOROUT (0xFFFFFFFFu)

uint32_t BL_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;

    while (length > 0u)
    {
        crc ^= *data;
        for (i = 0u; i < 8u; i++)
        {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (BL_CRC32_POLY & mask);
        }
        data++;
        length--;
    }

    return crc;
}

uint32_t BL_Crc32Finish(uint32_t crc)
{
    return crc ^ BL_CRC32_XOROUT;
}

uint32_t BL_ImageCalcCrc32(uint32_t storage_base, uint32_t image_size)
{
    uint32_t crc = BL_CRC32_INIT;
    uint32_t offset;
    const uint32_t crc_offset = BL_IMAGE_HEADER_OFFSET + (uint32_t)offsetof(bl_image_header_t, image_crc32);

    for (offset = 0u; offset < image_size; offset++)
    {
        uint8_t byte;

        if ((offset >= crc_offset) && (offset < (crc_offset + sizeof(uint32_t))))
        {
            byte = 0u;
        }
        else
        {
            byte = *(const volatile uint8_t *)(storage_base + offset);
        }

        crc = BL_Crc32Update(crc, &byte, 1u);
    }

    return BL_Crc32Finish(crc);
}

static bool range_contains(uint32_t base, uint32_t size, uint32_t addr)
{
    return (addr >= base) && (addr < (base + size));
}

static bool vector_is_valid(uint32_t storage_base, uint32_t execution_base, uint32_t slot_size, uint32_t sram_base, uint32_t sram_size)
{
    const uint32_t *vectors = (const uint32_t *)storage_base;
    uint32_t sp = vectors[0];
    uint32_t pc = vectors[1] & ~1u;

    if ((vectors[1] & 1u) == 0u)
    {
        return false;
    }

    if ((sp < sram_base) || (sp > (sram_base + sram_size)) || ((sp & 0x3u) != 0u))
    {
        return false;
    }

    return range_contains(execution_base, slot_size, pc);
}

bool BL_ImageValidate(uint32_t storage_base,
                      uint32_t execution_base,
                      uint32_t slot_size,
                      uint32_t sram_base,
                      uint32_t sram_size,
                      bl_image_info_t *info)
{
    const bl_image_header_t *header = (const bl_image_header_t *)(storage_base + BL_IMAGE_HEADER_OFFSET);
    uint32_t crc;

    if (info != NULL)
    {
        (void)memset(info, 0, sizeof(*info));
        info->storage_base = storage_base;
        info->execution_base = execution_base;
    }

    if (!vector_is_valid(storage_base, execution_base, slot_size, sram_base, sram_size))
    {
        return false;
    }

    if ((header->magic != BL_IMAGE_MAGIC) ||
        (header->header_version != BL_IMAGE_HEADER_VER) ||
        (header->header_size != sizeof(bl_image_header_t)))
    {
        return false;
    }

    if ((header->image_size == 0u) ||
        (header->image_size > slot_size) ||
        ((header->image_size & (BL_FLASH_PHRASE_SIZE - 1u)) != 0u) ||
        (header->image_size < (BL_IMAGE_HEADER_OFFSET + sizeof(bl_image_header_t))))
    {
        return false;
    }

    crc = BL_ImageCalcCrc32(storage_base, header->image_size);
    if (crc != header->image_crc32)
    {
        return false;
    }

    if (info != NULL)
    {
        info->version = header->image_version;
        info->image_size = header->image_size;
        info->image_crc32 = header->image_crc32;
        info->valid = true;
    }

    return true;
}

bl_image_action_t BL_ImageSelectAction(bool app_valid,
                                      const bl_image_info_t *app,
                                      bool backup_valid,
                                      const bl_image_info_t *backup)
{
    if (!app_valid)
    {
        return backup_valid ? kBL_ImageActionRestoreBackup : kBL_ImageActionStayInBootloader;
    }

    if (!backup_valid)
    {
        return kBL_ImageActionBootApp;
    }

    if (app->version > backup->version)
    {
        return kBL_ImageActionBootApp;
    }

    if (app->version < backup->version)
    {
        return kBL_ImageActionRestoreBackup;
    }

    if ((app->image_size == backup->image_size) && (app->image_crc32 == backup->image_crc32))
    {
        return kBL_ImageActionBootApp;
    }

    return kBL_ImageActionRestoreBackup;
}

bool BL_ImageEraseRequestIsValid(uint32_t start_addr, uint32_t byte_cnt)
{
    return (start_addr == BL_APP_BASE) && (byte_cnt > 0u) && (byte_cnt <= BL_SLOT_SIZE);
}

static void trial_clear(volatile bl_trial_state_t *state)
{
    state->magic = 0u;
    state->image_crc32 = 0u;
    state->watchdog_reset_count = 0u;
    state->demo_fault = 0u;
}

bool BL_TrialShouldRestore(bool trial_active,
                           bool watchdog_reset,
                           uint32_t image_crc32,
                           volatile bl_trial_state_t *state)
{
    if (!trial_active)
    {
        trial_clear(state);
        return false;
    }

    if (!BL_TrialStateMatches(state, image_crc32))
    {
        trial_clear(state);
        state->magic = BL_TRIAL_MAGIC;
        state->image_crc32 = image_crc32;
    }

    if (!watchdog_reset)
    {
        state->watchdog_reset_count = 0u;
        state->demo_fault = 0u;
        return false;
    }

    state->watchdog_reset_count++;
    if (state->watchdog_reset_count < BL_TRIAL_RESET_LIMIT)
    {
        return false;
    }

    trial_clear(state);
    return true;
}
