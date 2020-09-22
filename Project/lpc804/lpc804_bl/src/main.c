/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "flash.h"

#include "mcuboot.h"
#include "bl_cfg.h"

static uint8_t force_enter_bl = 0;
static uint8_t timeout_jump = 0;
static mcuboot_t mcuboot;

static int memory_erase(uint32_t start_addr, uint32_t byte_cnt)
{
    int addr;
    addr = start_addr;
    
    while(addr < (byte_cnt + start_addr))
    {
        FLASH_ErasePage(addr);
        addr += FLASH_GetPageSize();
    }
    return 0;
}

static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    uint32_t page_size;
    uint32_t page_addr;
    static ALIGN(64) uint8_t align_buf[64];
    
    page_size = FLASH_GetPageSize();
    
    page_addr = ALIGN_DOWN(start_addr, page_size);
    
    memcpy(align_buf, (uint8_t*)page_addr, page_size);
    memcpy(align_buf + start_addr - page_addr, buf, byte_cnt);
    
    FLASH_WritePage(page_addr, align_buf);

    
    return 0;
}

int memory_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, (void*)addr, len);
    return 0;
}

static int mcuboot_send(uint8_t *buf, uint32_t len)
{
    while(len--)
    {
        UART_PutChar(HW_UART0, *buf++);
    }
    return 0;
}

static void mcuboot_reset(void)
{
    /* delay for a while to wait mcuboot send respond packet */
    DelayMs(100);
    NVIC_SystemReset();
}

static void mcuboot_complete(void) {}

bool is_app_addr_validate(void)
{
    uint32_t *vectorTable = (uint32_t*)APPLICATION_BASE;
    uint32_t pc = vectorTable[1];
    
    if (pc < APPLICATION_BASE || pc > TARGET_FLASH_SIZE)
    {
        return false;
    }
    else
    {
        return true;
    } 
}

static void mcuboot_jump(uint32_t addr, uint32_t arg, uint32_t sp)
{
    if(is_app_addr_validate() == true)
    {
        /* clean up resouces */
        JumpToImage(addr);
    }
}


int main(void)
{
    uint8_t c;
    SystemCoreClockUpdate();
    
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1<<7) | (1<<14) | (1<<18);
    
    DelayInit();
    DelayMs(10);
 
    /* UART: RX--P0_0, TX--P0_1 */
    SWM_Config(0, 1, 0);
    SWM_Config(0, 0, 8);
   
    UART_Init(HW_UART0, 115200);
    
    LIB_TRACE("CoreClock:%dHz\r\n", GetClock(kCoreClock));

    FLASH_Init();
    
    /* config the mcuboot */
    mcuboot.op_send = mcuboot_send;
    mcuboot.op_reset = mcuboot_reset;
    mcuboot.op_jump = mcuboot_jump;
    mcuboot.op_complete = mcuboot_complete;
    
    mcuboot.op_mem_erase = memory_erase;
    mcuboot.op_mem_write = memory_write;
    mcuboot.op_mem_read = memory_read;
    
    mcuboot.cfg_flash_start = APPLICATION_BASE;
    mcuboot.cfg_flash_size = TARGET_FLASH_SIZE;
    mcuboot.cfg_flash_sector_size = 64;
    mcuboot.cfg_ram_start = 0x10000000;
    mcuboot.cfg_ram_size = 2*1024;
    mcuboot.cfg_device_id = 0x12345678;
    mcuboot.cfg_uuid = GetUID();
    
    mcuboot_init(&mcuboot);
        
    SysTick_SetTime(100*1000);
    SysTick_SetIntMode(true);
    
    while(1)
    {
        if(UART_GetChar(HW_UART0, &c) == CH_OK)
        {
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


void SysTick_Handler(void)
{
    static int timeout;
    if(timeout > BL_TIMEOUT_MS/100)
    {
        if(force_enter_bl == 0 && mcuboot_is_connected(&mcuboot) == 0)
        {
            timeout_jump = 1;
        }
        SysTick_SetIntMode(false);
    }
    timeout++;
}


void HardFault_Handler(void)
{
    LIB_TRACE("HardFault_Handler\r\n");
    #if LIB_DEBUG
    while(1);
    #endif
}
