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
#include "syscon.h"
#include "hid_mcuboot.h"
#include "bl_cfg.h"

static uint8_t force_enter_bl = 0;
static uint8_t timeout_jump = 0;
static hid_mcuboot_t hid_mcuboot;

static struct usbd_t usbd;
void ch_usb_init(struct usbd_t *h);


void USBD_Connect (bool con);
void USBD_Init (void);




static int memory_erase(uint32_t start_addr, uint32_t byte_cnt)
{
    int addr;
    addr = start_addr;

    while(addr < (byte_cnt + start_addr))
    {
        FLASH_EraseSector(addr);
        addr += FLASH_GetSectorSize();
    }
    return 0;
}

static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    uint32_t page_size;
    uint32_t ali_addr;
    static ALIGN(256) uint8_t align_buf[256];
    
    page_size = FLASH_GetPageSize();
    
    ali_addr = ALIGN_DOWN(start_addr, page_size);
    
    memcpy(align_buf, (uint8_t*)ali_addr, page_size);
    memcpy(align_buf + start_addr - ali_addr, buf, byte_cnt);

    //FLASH_ErasePage(ali_addr);
    FLASH_WritePage(ali_addr, align_buf);

    return 0;
}

int memory_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, (void*)addr, len);
    return 0;
}


static void mcuboot_reset(void)
{
    /* delay for a while to wait mcuboot send respond packet */
    DelayMs(100);
    NVIC_SystemReset();
}

static void mcuboot_complete(void) 
{
    
}

bool is_app_addr_validate(void)
{
    uint32_t *vectorTable = (uint32_t*)APPLICATION_BASE;
    uint32_t pc = vectorTable[1];

    if (pc < APPLICATION_BASE || pc > (APPLICATION_BASE + TARGET_FLASH_SIZE))
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
        GPIO_Init(HW_GPIO0, 24, kGPIO_IFT);
        USART0->CFG &= ~(USART_CFG_ENABLE_MASK);
        USBD_Connect(false);
        
        /* jump to app */
        JumpToImage(addr);
    }
}


int main(void)
{
    SetFROClock(48*1000*1000, true);
    DelayInit();
    
    SetPinMux(HW_GPIO0, 0, 1);
    SetPinMux(HW_GPIO0, 1, 1);
    UART_Init(HW_UART0, 115200);

    /* if press boot pin */
    GPIO_Init(HW_GPIO0, 24, kGPIO_IPU);
    if(GPIO_PinRead(HW_GPIO0, 24) == 0)
    {
        printf("enter boot loader mode\r\n");
        force_enter_bl = 1;
    }
    
    ch_usb_init(&usbd);
    
    usbd_hid_init(&usbd, 1);
    extern struct usbd_hid_callback_t hid_cb;
    usbd_hid_set_cb(&hid_cb);
    
    /* VBUS LPC51U68 */
    IOCON->PIO[1][6] = IOCON_PIO_FUNC(7) | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_MODE(0);
    USBD_Init();
    USBD_Connect(true);
    

    hid_mcuboot.op_reset = mcuboot_reset;
    hid_mcuboot.op_jump = mcuboot_jump;
    hid_mcuboot.op_complete = mcuboot_complete;
    
    hid_mcuboot.op_mem_erase = memory_erase;
    hid_mcuboot.op_mem_write = memory_write;
    hid_mcuboot.op_mem_read = memory_read;
    
    hid_mcuboot.cfg_flash_start = APPLICATION_BASE; 
    hid_mcuboot.cfg_flash_size = TARGET_FLASH_SIZE;
    hid_mcuboot.cfg_flash_sector_size = FLASH_GetSectorSize();
    hid_mcuboot.cfg_ram_start = 0x20000000;
    hid_mcuboot.cfg_ram_size = 128*1024;
    hid_mcuboot.cfg_device_id = 0x12345678;
    hid_mcuboot.cfg_uuid = GetUID();
    hid_mcuboot_init(&hid_mcuboot);

    
    FLASH_Init();
    SysTick_SetTime(100*1000);
    SysTick_SetIntMode(true);
    
    while(1)
    {
        if(timeout_jump == 1)
        {
            mcuboot_jump(APPLICATION_BASE, 0, 0);
            timeout_jump = 0;
        }
    }
}


void SysTick_Handler(void)
{
    static int timeout;
    if(timeout > BL_TIMEOUT_MS/100)
    {
        if(force_enter_bl == 0 && hid_mcuboot_is_connected(&hid_mcuboot) == 0)
        {
            timeout_jump = 1;
        }
        SysTick_SetIntMode(false);
    }
    timeout++;
}


void HardFault_Handler(void)
{
    NVIC_SystemReset();
    
    while(1)
    {
    }
}



