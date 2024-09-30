/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_lpuart.h"
#include "fsl_romapi.h"

#include "mcuboot.h"
#include "bl_cfg.h"

#define SYSTICK_INTERVAL_MS 100

static uint8_t force_enter_bl = 0;
static uint8_t timeout_jump = 0;
static mcuboot_t mcuboot;
static flash_config_t s_flashDriver;

void JumpToImage(uint32_t addr);

// Function to erase flash memory
static int memory_erase(uint32_t start_addr, uint32_t byte_cnt)
{
    status_t status;
    
    uint32_t end_addr = start_addr + byte_cnt;
    uint32_t aligned_start = start_addr & ~(s_flashDriver.PFlashSectorSize - 1);
    uint32_t aligned_end = (end_addr + s_flashDriver.PFlashSectorSize - 1) & ~(s_flashDriver.PFlashSectorSize - 1);

    // Erase flash sectors
    for (uint32_t addr = aligned_start; addr < aligned_end; addr += s_flashDriver.PFlashSectorSize)
    {
        status = FLASH_API->flash_erase_sector(&s_flashDriver, addr, s_flashDriver.PFlashSectorSize, kFLASH_ApiEraseKey);
        if (status != kStatus_Success)
        {
            return -1;
        }
    }

    return 0;
}

// Function to write to flash memory
static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    status_t status;
    
    uint32_t end_addr = start_addr + byte_cnt;
    uint32_t current_addr = start_addr;
    uint32_t remaining = byte_cnt;

    // Write data to flash
    while (current_addr < end_addr)
    {
        uint32_t write_size = (remaining < s_flashDriver.PFlashPageSize) ? remaining : s_flashDriver.PFlashPageSize;
        status = FLASH_API->flash_program_page(&s_flashDriver, current_addr, buf, write_size);
        if (status != kStatus_Success)
        {
            return -1;
        }

        current_addr += write_size;
        buf += write_size;
        remaining -= write_size;
    }

    return 0;
}

// Function to read from memory
int memory_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, (void*)addr, len);
    return 0;
}

// Function to send data through UART
static int mcuboot_send(uint8_t *buf, uint32_t len)
{
    return LPUART_WriteBlocking(LPUART0, buf, len);
}

// Function to reset the MCU
static void mcuboot_reset(void)
{
    // Delay for a while to wait for mcuboot to send respond packet
    SDK_DelayAtLeastUs(10*1000, CLOCK_GetCoreSysClkFreq());
    NVIC_SystemReset();
}

static void mcuboot_complete(void) {}

// Function to validate application address
bool is_app_addr_validate(void)
{
    uint32_t *vectorTable = (uint32_t*)APPLICATION_BASE;
    uint32_t pc = vectorTable[1];
    
    return (pc >= APPLICATION_BASE && pc <= (APPLICATION_BASE + TARGET_FLASH_SIZE));
}

// Function to jump to the application
static void mcuboot_jump(uint32_t addr, uint32_t arg, uint32_t sp)
{
    if(is_app_addr_validate())
    {
        // Clean up resources and jump to the image
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        LPUART_Deinit(LPUART0);

        JumpToImage(addr);
    }
}

// Function to initialize clocks
static void app_clk_init(void)
{
    // Enable clocks for ports and GPIOs
    CLOCK_EnableClock(kCLOCK_GatePORT0);
    CLOCK_EnableClock(kCLOCK_GatePORT1);
    CLOCK_EnableClock(kCLOCK_GatePORT2);
    CLOCK_EnableClock(kCLOCK_GatePORT3);
    CLOCK_EnableClock(kCLOCK_GateGPIO0);
    CLOCK_EnableClock(kCLOCK_GateGPIO1);
    CLOCK_EnableClock(kCLOCK_GateGPIO2);
    CLOCK_EnableClock(kCLOCK_GateGPIO3);
    
    // Release peripheral resets
    RESET_ReleasePeripheralReset(kPORT0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT2_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT3_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO2_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO3_RST_SHIFT_RSTn);
    
    RESET_ReleasePeripheralReset(kLPUART0_RST_SHIFT_RSTn);

    // Configure UART pins
    PORT_SetPinMux(PORT0, 2, kPORT_MuxAlt2);
    PORT_SetPinMux(PORT0, 3, kPORT_MuxAlt2);
    PORT0->PCR[2] |= PORT_PCR_IBE_MASK;
    PORT0->PCR[3] |= PORT_PCR_IBE_MASK;
    
    // Configure UART clock
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPUART0);
}

int main(void)
{
    uint8_t c;

    app_clk_init();

    // Configure UART
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(LPUART0, &config, CLOCK_GetLpuartClkFreq(0));

    printf("FRDM-MCXA153 Bootloader\r\n");
    
    FLASH_API->flash_init(&s_flashDriver);
    
    // Print flash properties
    printf("PFlash Information:");
    printf("kFLASH_PropertyPflashSectorSize = %d\r\n", s_flashDriver.PFlashTotalSize);
    printf("kFLASH_PropertyPflashPageSize = 0x%X\r\n", s_flashDriver.PFlashPageSize);

    // Configure mcuboot
    mcuboot.op_send = mcuboot_send;
    mcuboot.op_reset = mcuboot_reset;
    mcuboot.op_jump = mcuboot_jump;
    mcuboot.op_complete = mcuboot_complete;
    
    mcuboot.op_mem_erase = memory_erase;
    mcuboot.op_mem_write = memory_write;
    mcuboot.op_mem_read = memory_read;
    
    mcuboot.cfg_flash_start = APPLICATION_BASE; 
    mcuboot.cfg_flash_size = TARGET_FLASH_SIZE;
    mcuboot.cfg_flash_sector_size = s_flashDriver.PFlashSectorSize;
    mcuboot.cfg_ram_start = 0x20000000;
    mcuboot.cfg_ram_size = 32*1024;
    mcuboot.cfg_device_id = 0x12345678;
    mcuboot.cfg_uuid = 0x12345678;
    
    mcuboot_init(&mcuboot);

    // Configure SysTick
    uint32_t ticks = CLOCK_GetCoreSysClkFreq() / 1000 * SYSTICK_INTERVAL_MS;
    SysTick_Config(ticks);
    
    while(1)
    {
        if (LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag)
        {
            c = LPUART_ReadByte(LPUART0);
            mcuboot_recv(&mcuboot, &c, 1);
        }
        
        if(timeout_jump == 1)
        {
            mcuboot_jump(APPLICATION_BASE, 0, 0);
            timeout_jump = 0;
        }
        mcuboot_proc(&mcuboot);
    }
}

// SysTick handler for timeout management
void SysTick_Handler(void)
{
    static int timeout;
    if(timeout > BL_TIMEOUT_MS/100)
    {
        if(force_enter_bl == 0 && mcuboot_is_connected(&mcuboot) == 0)
        {
            timeout_jump = 1;
        }
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }
    timeout++;
}

// HardFault handler
void HardFault_Handler(void)
{
    printf("HardFault_Handler\r\n");
    NVIC_SystemReset();
    while(1) {}
}

// Assertion handler
void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
    // Empty implementation
}

// Function to jump to the application image
void JumpToImage(uint32_t addr)
{
    static uint32_t sp, pc;
    uint32_t *vectorTable = (uint32_t*)addr;
    sp = vectorTable[0];
    pc = vectorTable[1];

    typedef void(*app_entry_t)(void);
    static app_entry_t s_application = 0;

    s_application = (app_entry_t)pc;

    // Change MSP and PSP
    __set_MSP(sp);
    __set_PSP(sp);
    
    SCB->VTOR = addr;
    
    // Jump to application
    s_application();

    // Should never reach here
    __NOP();
}

// Weak symbol definitions for standard I/O
#pragma weak __stdout
#pragma weak __stdin
FILE __stdout;
FILE __stdin;

// Weak symbol definition for fputc
#pragma weak fputc
int fputc(int ch, FILE *f)
{
    LPUART_WriteByte(LPUART0, (uint8_t)ch);
    
    while (!(LPUART0->STAT & LPUART_STAT_TDRE_MASK))
    {
    }
    
    return ch;
}
