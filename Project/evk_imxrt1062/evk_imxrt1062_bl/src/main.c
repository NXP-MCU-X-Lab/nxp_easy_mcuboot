/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_lpuart.h"
#include "mcuboot.h"
#include "bl_cfg.h"
#include "flash.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t force_enter_bl = 0;
static uint8_t timeout_jump = 0;
/* mcuboot should be aligned 4bytes to make frame_packet 
   buffer 4bytes aligment for safe programming
*/
static __attribute__((aligned(4))) mcuboot_t mcuboot;
/*******************************************************************************
 * Code
 ******************************************************************************/
int PutChar(uint8_t ch)
{
	  LPUART_WriteBlocking(UART_INSTANCE, &ch, 1);
	
	  return ch;
}

int GetChar(void)
{
	  uint8_t c;
	  /* Blocking Mode to receive */
	  LPUART_ReadBlocking(UART_INSTANCE, &c, 1);
	  return c;
}

__attribute__((section("RamFunction")))
status_t UART_GetChar(LPUART_Type *base, uint8_t *ch)
{
    if(base->STAT & LPUART_STAT_RDRF_MASK)
    {
        *ch = (uint8_t)(base->DATA);	
        return kStatus_Success; 		  
    }
		/* if OverRun flag is set it must be cleared for normal receive */
    if(base->STAT & LPUART_STAT_OR_MASK)
    {
        base->STAT |= LPUART_STAT_OR_MASK;
    }
    return kStatus_Fail;
}

__attribute__((section("RamFunction")))
static int memory_erase(uint32_t start_addr, uint32_t byte_cnt)
{
	  static bool first_erase = true;
    int addr;
    addr = start_addr;

	  /* For the 1st Erase operation, FlashInit should be called */
	  if(first_erase==true)
		{
	      FLASH_Init();
			  first_erase = false;
		}
	  
    while(addr < (byte_cnt + start_addr))
    {
        FLASH_EraseSector(addr);
        addr += FLASH_GetSectorSize();
    }
    return 0;
}

__attribute__((section("RamFunction")))
static int memory_write(uint32_t start_addr, uint8_t *buf, uint32_t byte_cnt)
{
    FLASH_WritePage(start_addr, buf, byte_cnt);
    return 0;
}

__attribute__((section("RamFunction")))
static int memory_read(uint32_t addr, uint8_t *buf, uint32_t len)
{   
	  /* For FlexSPI Memory ReadBack, use IP Command instead of AXI command for security */
	  if((addr >= 0x60000000) && (addr < 0x7F800000))
			FLASH_Read(addr, buf, len);
		else
      memcpy(buf, (void*)addr, len);
    return 0;
}

__attribute__((section("RamFunction")))
static int mcuboot_send(uint8_t *buf, uint32_t len)
{
    LPUART_WriteBlocking(UART_INSTANCE, buf, len);
    return CH_OK;
}

static void mcuboot_reset(void)
{
    /* delay for a while to wait mcuboot send respond packet */
    DelayMs(100);
    NVIC_SystemReset();
}

static void mcuboot_complete(void)
{
		FLASH_DeInit();
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
        JumpToImage(addr);
    }
}

int main(void)
{
    uint8_t c;
	  BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
	
    DelayInit();
	
	  /* UART Init */
	  lpuart_config_t config;
	  LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;
    LPUART_Init(UART_INSTANCE, &config, CLOCK_GetClockRootFreq(kCLOCK_UartClkRoot));
	
	  /* Set Console for Stdio */
	  SetConsole(PutChar, GetChar);
	
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
    mcuboot.cfg_ram_start = 0x20200000;//OCRAM Start Address
    mcuboot.cfg_ram_size = (256+512)*1024;//Default OCRAM Size
    mcuboot.cfg_device_id = 0x12345678;
    mcuboot.cfg_uuid = GetUID();
    
    mcuboot_init(&mcuboot);
		
    SysTick_SetTime(100*1000);
    SysTick_SetIntMode(true);
		
    while(1)
    {
			  if(UART_GetChar(UART_INSTANCE, &c) == kStatus_Success)
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


