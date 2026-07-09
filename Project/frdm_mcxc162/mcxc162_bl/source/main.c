/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_flash.h"
#include "fsl_k4_flash.h"
#include "fsl_lpuart.h"
#include "fsl_reset.h"

#include "bl_config.h"
#include "bl_image.h"
#include "clock_config.h"
#include "mcuboot.h"
#include "pin_mux.h"


static mcuboot_t s_mcuboot;
static flash_config_t s_flash;
static volatile uint32_t s_ticks;
static volatile bool s_timeout_jump;
static bool s_app_valid;
static bool s_flash_ready;

typedef struct
{
    bool dirty;
    uint32_t addr;
    uint8_t data[BL_FLASH_PHRASE_SIZE];
} phrase_cache_t;

static phrase_cache_t s_phrase_cache;
static int s_flash_status;

#if BL_ENABLE_BOOT_LOG
static void bl_log_raw(const char *text)
{
    const char *p = text;

    while (*p != '\0')
    {
        (void)LPUART_WriteBlocking(LPUART0, (uint8_t *)p, 1u);
        p++;
    }
}

static void bl_log_hex32(uint32_t value)
{
    char text[11];
    uint32_t i;

    text[0] = '0';
    text[1] = 'x';
    for (i = 0u; i < 8u; i++)
    {
        uint32_t digit = (value >> ((7u - i) * 4u)) & 0xFu;
        text[2u + i] = (char)((digit < 10u) ? ('0' + digit) : ('A' + digit - 10u));
    }
    text[10] = '\0';

    bl_log_raw(text);
}

static void bl_log_image(const char *name, bool valid, const bl_image_info_t *info)
{
    bl_log_raw(name);
    bl_log_raw(valid ? " valid" : " invalid");
    if (valid && (info != NULL))
    {
        bl_log_raw(" ver=");
        bl_log_hex32(info->version);
        bl_log_raw(" size=");
        bl_log_hex32(info->image_size);
        bl_log_raw(" crc=");
        bl_log_hex32(info->image_crc32);
    }
    bl_log_raw("\r\n");
}

static void bl_log_region(const char *name, uint32_t base, uint32_t size)
{
    bl_log_raw("layout ");
    bl_log_raw(name);
    bl_log_raw(" base=");
    bl_log_hex32(base);
    bl_log_raw(" size=");
    bl_log_hex32(size);
    bl_log_raw("\r\n");
}

static void bl_log_layout(void)
{
    bl_log_region("boot", BL_BOOT_BASE, BL_BOOT_SIZE);
    bl_log_region("app", BL_APP_BASE, BL_SLOT_SIZE);
    bl_log_region("backup", BL_BACKUP_BASE, BL_SLOT_SIZE);
    bl_log_raw("layout header offset=");
    bl_log_hex32(BL_IMAGE_HEADER_OFFSET);
    bl_log_raw(" addr=");
    bl_log_hex32(BL_APP_BASE + BL_IMAGE_HEADER_OFFSET);
    bl_log_raw("\r\n");
}

#define BL_LOG_RAW(text)       bl_log_raw(text)
#define BL_LOG_HEX32(value)    bl_log_hex32(value)
#define BL_LOG_IMAGE(n, v, i)  bl_log_image((n), (v), (i))
#define BL_LOG_LAYOUT()        bl_log_layout()
#else
#define BL_LOG_RAW(text)       do { } while (0)
#define BL_LOG_HEX32(value)    do { } while (0)
#define BL_LOG_IMAGE(n, v, i)  do { } while (0)
#define BL_LOG_LAYOUT()        do { } while (0)
#endif

static bool range_is_inside(uint32_t base, uint32_t size, uint32_t addr, uint32_t len)
{
    if ((len == 0u) || (addr < base))
    {
        return false;
    }

    return len <= ((base + size) - addr);
}

static uint32_t align_down(uint32_t value, uint32_t align)
{
    return value & ~(align - 1u);
}

static uint32_t align_up(uint32_t value, uint32_t align)
{
    return (value + align - 1u) & ~(align - 1u);
}

static bool logical_to_backup(uint32_t addr, uint32_t len, uint32_t *mapped)
{
    if (!range_is_inside(BL_APP_BASE, BL_SLOT_SIZE, addr, len))
    {
        return false;
    }

    *mapped = BL_BACKUP_BASE + (addr - BL_APP_BASE);
    return true;
}

static int flash_verify_readback(uint32_t addr, const uint8_t *data, uint32_t len)
{
    return (memcmp((const void *)addr, data, len) == 0) ? 0 : -1;
}

static int flash_erase_aligned(uint32_t addr, uint32_t len)
{
    uint32_t sector;
    status_t status;

    if (!s_flash_ready)
    {
        return -1;
    }

    status = FLASH_Erase(&s_flash, FLASH, addr, len, kFLASH_ApiEraseKey);
    if (status != kStatus_FLASH_Success)
    {
        return -1;
    }

    for (sector = addr; sector < (addr + len); sector += BL_FLASH_SECTOR_SIZE)
    {
        status = FLASH_VerifyEraseSector(&s_flash, FLASH, sector, BL_FLASH_SECTOR_SIZE);
        if (status != kStatus_FLASH_Success)
        {
            return -1;
        }
    }

    return 0;
}

static int flash_program_aligned(uint32_t addr, const uint8_t *data, uint32_t len)
{
    status_t status;

    if (!s_flash_ready)
    {
        return -1;
    }

    status = FLASH_Program(&s_flash, FLASH, addr, (uint8_t *)data, len);
    if (status != kStatus_FLASH_Success)
    {
        return -1;
    }

    return flash_verify_readback(addr, data, len);
}

static int phrase_cache_flush(void)
{
    int status = 0;

    if (s_phrase_cache.dirty)
    {
        status = flash_program_aligned(s_phrase_cache.addr, s_phrase_cache.data, BL_FLASH_PHRASE_SIZE);
        s_phrase_cache.dirty = false;
    }

    if (status != 0)
    {
        s_flash_status = status;
    }

    return status;
}

static void phrase_cache_start(uint32_t phrase_addr)
{
    s_phrase_cache.addr = phrase_addr;
    memset(s_phrase_cache.data, 0xFF, sizeof(s_phrase_cache.data));
    s_phrase_cache.dirty = false;
}

static int phrase_cache_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    while (len > 0u)
    {
        uint32_t phrase_addr = align_down(addr, BL_FLASH_PHRASE_SIZE);
        uint32_t phrase_offset = addr - phrase_addr;
        uint32_t write_len = BL_FLASH_PHRASE_SIZE - phrase_offset;

        if (write_len > len)
        {
            write_len = len;
        }

        if (!s_phrase_cache.dirty || (s_phrase_cache.addr != phrase_addr))
        {
            if (phrase_cache_flush() != 0)
            {
                return -1;
            }
            phrase_cache_start(phrase_addr);
        }

        memcpy(&s_phrase_cache.data[phrase_offset], data, write_len);
        s_phrase_cache.dirty = true;

        if ((phrase_offset + write_len) == BL_FLASH_PHRASE_SIZE)
        {
            if (phrase_cache_flush() != 0)
            {
                return -1;
            }
        }

        addr += write_len;
        data += write_len;
        len -= write_len;
    }

    return 0;
}

static int memory_erase(uint32_t start_addr, uint32_t byte_cnt)
{
    uint32_t mapped_start;
    uint32_t mapped_end;
    uint32_t erase_start;
    uint32_t erase_end;

    if (phrase_cache_flush() != 0)
    {
        return -1;
    }

    if (!logical_to_backup(start_addr, byte_cnt, &mapped_start))
    {
        return -1;
    }

    mapped_end = mapped_start + byte_cnt;
    erase_start = align_down(mapped_start, BL_FLASH_SECTOR_SIZE);
    erase_end = align_up(mapped_end, BL_FLASH_SECTOR_SIZE);

    if (!range_is_inside(BL_BACKUP_BASE, BL_SLOT_SIZE, erase_start, erase_end - erase_start))
    {
        return -1;
    }

    s_flash_status = flash_erase_aligned(erase_start, erase_end - erase_start);
    phrase_cache_start(0u);

    return s_flash_status;
}

static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    uint32_t mapped_addr;

    if (s_flash_status != 0)
    {
        return -1;
    }

    if (!logical_to_backup(start_addr, byte_cnt, &mapped_addr))
    {
        s_flash_status = -1;
        return -1;
    }

    s_flash_status = phrase_cache_write(mapped_addr, buf, byte_cnt);
    return s_flash_status;
}

static int memory_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t mapped_addr;

    if (!logical_to_backup(addr, len, &mapped_addr))
    {
        return -1;
    }

    memcpy(buf, (const void *)mapped_addr, len);
    return 0;
}

static int mcuboot_send(uint8_t *buf, uint32_t len)
{
    return (LPUART_WriteBlocking(LPUART0, buf, len) == kStatus_Success) ? 0 : -1;
}

static void mcuboot_reset(void)
{
    SDK_DelayAtLeastUs(10000u, CLOCK_GetCoreSysClkFreq());
    NVIC_SystemReset();
}

static void mcuboot_complete(void)
{
    (void)phrase_cache_flush();
}

static void jump_to_image(uint32_t storage_base)
{
    const uint32_t *vectors = (const uint32_t *)storage_base;
    uint32_t sp = vectors[0];
    uint32_t pc = vectors[1];

    __disable_irq();
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL = 0u;
    (void)LPUART_Deinit(LPUART0);

    SCB->VTOR = storage_base;
    __set_MSP(sp);
    __set_PSP(sp);
    __DSB();
    __ISB();

    ((void (*)(void))pc)();
}

static void mcuboot_jump(uint32_t addr, uint32_t arg, uint32_t sp)
{
    (void)addr;
    (void)arg;
    (void)sp;

    if (BL_ImageValidate(BL_APP_BASE, BL_APP_BASE, BL_SLOT_SIZE, BL_SRAM_BASE, BL_SRAM_SIZE, NULL))
    {
        BL_LOG_RAW("Jump A\r\n");
        jump_to_image(BL_APP_BASE);
    }

    BL_LOG_RAW("Jump A blocked\r\n");
}

static bool copy_backup_to_app(const bl_image_info_t *backup)
{
    uint8_t page[BL_FLASH_PAGE_SIZE];
    uint32_t offset;

    if (flash_erase_aligned(BL_APP_BASE, BL_SLOT_SIZE) != 0)
    {
        BL_LOG_RAW("Promote erase failed\r\n");
        return false;
    }

    for (offset = 0u; offset < backup->image_size; offset += sizeof(page))
    {
        uint32_t len = backup->image_size - offset;
        if (len > sizeof(page))
        {
            len = sizeof(page);
        }

        memcpy(page, (const void *)(BL_BACKUP_BASE + offset), len);
        if (flash_program_aligned(BL_APP_BASE + offset, page, len) != 0)
        {
            BL_LOG_RAW("Promote program failed\r\n");
            return false;
        }
    }

    if (memcmp((const void *)BL_APP_BASE, (const void *)BL_BACKUP_BASE, backup->image_size) != 0)
    {
        BL_LOG_RAW("Promote verify failed\r\n");
        return false;
    }

    return BL_ImageValidate(BL_APP_BASE, BL_APP_BASE, BL_SLOT_SIZE, BL_SRAM_BASE, BL_SRAM_SIZE, NULL);
}

static void promote_backup_if_needed(void)
{
    bl_image_info_t app;
    bl_image_info_t backup;
    bool app_valid = BL_ImageValidate(BL_APP_BASE, BL_APP_BASE, BL_SLOT_SIZE, BL_SRAM_BASE, BL_SRAM_SIZE, &app);
    bool backup_valid = BL_ImageValidate(BL_BACKUP_BASE, BL_APP_BASE, BL_SLOT_SIZE, BL_SRAM_BASE, BL_SRAM_SIZE, &backup);

    BL_LOG_IMAGE("A", app_valid, &app);
    BL_LOG_IMAGE("B", backup_valid, &backup);

    if (backup_valid && ((!app_valid) || (backup.version > app.version)))
    {
        BL_LOG_RAW("Promote B to A\r\n");
        app_valid = copy_backup_to_app(&backup);
        BL_LOG_RAW(app_valid ? "Promote ok\r\n" : "Promote failed\r\n");
    }
    else
    {
        BL_LOG_RAW("Keep A\r\n");
    }

    s_app_valid = app_valid;
}

static bool boot_init(void)
{
    BOARD_InitBootClocks();

    MBC->MBC_INDEX[0].MBC_MEMN_GLBAC[0] = 0x00007777u;
    MBC->MBC_INDEX[0].MBC_DOM0_MEM0_BLK_CFG_W[0] = 0x00000000u;

    s_flash_ready = (FLASH_Init(&s_flash) == kStatus_FLASH_Success);
    return s_flash_ready;
}

static void uart_init(void)
{
    lpuart_config_t config;

    BOARD_InitDEBUG_UARTPins();
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(kFRO_LF_DIV_to_LPUART0);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BL_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    (void)LPUART_Init(LPUART0, &config, CLOCK_GetLpuartClkFreq(0u));
}

static void mcuboot_config(void)
{
    memset(&s_mcuboot, 0, sizeof(s_mcuboot));

    s_mcuboot.op_send = mcuboot_send;
    s_mcuboot.op_reset = mcuboot_reset;
    s_mcuboot.op_jump = mcuboot_jump;
    s_mcuboot.op_complete = mcuboot_complete;
    s_mcuboot.op_mem_erase = memory_erase;
    s_mcuboot.op_mem_write = memory_write;
    s_mcuboot.op_mem_read = memory_read;
    s_mcuboot.cfg_flash_start = BL_APP_BASE;
    s_mcuboot.cfg_flash_size = BL_SLOT_SIZE;
    s_mcuboot.cfg_flash_sector_size = BL_FLASH_SECTOR_SIZE;
    s_mcuboot.cfg_ram_start = BL_SRAM_BASE;
    s_mcuboot.cfg_ram_size = BL_SRAM_SIZE;
    s_mcuboot.cfg_device_id = 0x000C162u;
    s_mcuboot.cfg_uuid = 0x4D435831u;

    mcuboot_init(&s_mcuboot);
}

int main(void)
{
    uint8_t c;
    bool flash_ready;

    flash_ready = boot_init();
    uart_init();
    BL_LOG_RAW("\r\nMCXC162 BL\r\n");
    if (!flash_ready)
    {
        BL_LOG_RAW("Flash init failed\r\n");
    }
    BL_LOG_LAYOUT();
    promote_backup_if_needed();
    mcuboot_config();
    phrase_cache_start(0u);

    (void)SysTick_Config(CLOCK_GetCoreSysClkFreq() / 1000u * BL_SYSTICK_MS);

    while (1)
    {
        if ((LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag) != 0u)
        {
            c = LPUART_ReadByte(LPUART0);
            mcuboot_recv(&s_mcuboot, &c, 1u);
        }

        if (s_timeout_jump)
        {
            s_timeout_jump = false;
            mcuboot_jump(BL_APP_BASE, 0u, 0u);
        }

        mcuboot_proc(&s_mcuboot);
    }
}

void SysTick_Handler(void)
{
    s_ticks++;

    if ((s_ticks >= (BL_TIMEOUT_MS / BL_SYSTICK_MS)) &&
        s_app_valid &&
        (mcuboot_is_connected(&s_mcuboot) == 0u))
    {
        s_timeout_jump = true;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }
}

void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1)
    {
    }
}

void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
    (void)failedExpr;
    (void)file;
    (void)line;
}
