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
        FLASH_EraseSector(addr);
        addr += FLASH_GetSectorSize();
    }
    return 0;
}

static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    FLASH_WriteSector(start_addr, buf, byte_cnt);
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
    return CH_OK;
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
        JumpToImage(addr);
    }
}

static void ICS_FEE_20M(void)
{
    SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(1); /* Update system prescalers */
    /* Switch to FEE Mode */
    /* ICS->C2: BDIV=0,LP=0 */
    ICS->C2 &= (uint8_t)~(uint8_t)((ICS_C2_BDIV(0x07) | ICS_C2_LP_MASK));
    /* OSC->CR: OSCEN=1,??=0,OSCSTEN=0,OSCOS=1,??=0,RANGE=1,HGO=0,OSCINIT=0 */
    OSC->CR = (OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_RANGE_MASK);
    /* ICS->C1: CLKS=0,RDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    ICS->C1 = (ICS_C1_CLKS(0x00) | ICS_C1_RDIV(0x03) | ICS_C1_IRCLKEN_MASK);
    while((ICS->S & ICS_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
    }
    while((ICS->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
    }
    
    SystemCoreClock = 20*1000*1000;
}

int main(void)
{
    uint8_t c;

    ICS_FEE_20M();
    DelayInit();
    
   // GPIO_Init(HW_GPIOC, 5, kGPIO_OPPL);
    UART_Init(HW_UART0, 115200);

        
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
    mcuboot.cfg_flash_sector_size = FLASH_GetSectorSize();
    mcuboot.cfg_ram_start = 0x1FFFFF00;
    mcuboot.cfg_ram_size = 4*1024;
    mcuboot.cfg_device_id = 0x12345678;
    mcuboot.cfg_uuid = GetUID();
    
    mcuboot_init(&mcuboot);
   
    FLASH_Init();
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
    NVIC_SystemReset();
    
    while(1)
    {
    }
}


