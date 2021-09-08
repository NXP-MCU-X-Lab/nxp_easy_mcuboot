/**
  ******************************************************************************
  * @file    spifi.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
  
  
#include "spifi.h"
#include <string.h>
#include "common.h"

#if defined(SPIFI0) && defined(LPC54608)

void SPIFI_Reset(void)
{
    /* reset */
    SPIFI0->STAT = SPIFI_STAT_RESET_MASK;
    
    /* Wait for the RESET flag cleared by HW */
    while (SPIFI0->STAT & SPIFI_STAT_RESET_MASK) {};
}

void SPIFI_Init(uint32_t baud)
{
    SYSCON->SPIFICLKSEL = SYSCON_SPIFICLKSEL_SEL(3);    /* select SPIFI clock to kFROHfClock */
    
    (GetClock(kFROHfClock) == 96*1000*1000)?(SYSCON->SPIFICLKDIV = SYSCON_SPIFICLKDIV_DIV(1)):(SYSCON->SPIFICLKDIV = SYSCON_SPIFICLKDIV_DIV(0));
    
    /* enable clock */
    #if defined(SYSCON_AHBCLKCTRL_SPIFI_MASK)
    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_SPIFI_MASK;
    #endif
    
    SPIFI_Reset();
        
    /* Set time delay parameter */
    SPIFI0->CTRL = SPIFI_CTRL_TIMEOUT(0xFFFF) | SPIFI_CTRL_CSHIGH(0x0F) | SPIFI_CTRL_D_PRFTCH_DIS(0) | SPIFI_CTRL_MODE3(0) | SPIFI_CTRL_PRFTCH_DIS(0) | SPIFI_CTRL_DUAL(0) | SPIFI_CTRL_RFCLK(1) | SPIFI_CTRL_FBCLK(1);          
}



void SPIFI_SetCmd(spifi_command_t *cmd)
{
    /* If SPIFI in memory mode, call reset function to abort memory mode */
    if (SPIFI0->STAT& SPIFI_STAT_MCINIT_MASK)
    {
        SPIFI_Reset();
    }

    /* Wait for other command finished */
    while (SPIFI0->STAT & SPIFI_STAT_CMD_MASK)
    {
    }

    SPIFI0->CMD = SPIFI_CMD_DATALEN(cmd->dataLen) | SPIFI_CMD_POLL(0) | SPIFI_CMD_DOUT(cmd->direction) |
                SPIFI_CMD_INTLEN(cmd->intermediateBytes) | SPIFI_CMD_FIELDFORM(cmd->format) |
                SPIFI_CMD_FRAMEFORM(cmd->type) | SPIFI_CMD_OPCODE(cmd->opcode);    
}

void SPIFI_SetMemoryCmd(spifi_command_t *cmd)
{
    /* Wait for the CMD and MCINT flag all be 0 */
    while (SPIFI0->STAT & (SPIFI_STAT_MCINIT_MASK | SPIFI_STAT_CMD_MASK)) {};
    SPIFI0->MCMD = SPIFI_MCMD_POLL(0U) | SPIFI_MCMD_DOUT(0U) | SPIFI_MCMD_INTLEN(cmd->intermediateBytes) | SPIFI_MCMD_FIELDFORM(cmd->format) | SPIFI_MCMD_FRAMEFORM(cmd->type) | SPIFI_MCMD_OPCODE(cmd->opcode);
}








void QSPI_Init(qspi_flash_t * ctx, uint32_t freq)
{   
    SPIFI_Init(freq);
    SPIFI_Reset();
    SPIFI_SetMemoryCmd(&ctx->cmd[QSPI_CMD_READ_IDX]);
}

uint32_t QSPI_GetDeviceID(qspi_flash_t *ctx)
{
    SPIFI_Reset();
    SPIFI_SetCmd(&ctx->cmd[QSPI_READ_ID_IDX]);
    while ((SPIFI0->STAT & SPIFI_STAT_INTRQ_MASK) == 0U) {};
    
    return SPIFI0->DAT32;
}

uint32_t QSPI_WaitComplete(qspi_flash_t *ctx)
{
    uint8_t val = 0;
    /* Check WIP bit */
    do
    {
        SPIFI_SetCmd(&ctx->cmd[QSPI_GET_STATUS_IDX]);
        while ((SPIFI0->STAT & SPIFI_STAT_INTRQ_MASK) == 0U) {};
        val =  *((volatile uint8_t *)(&(SPIFI0->DAT8)));
    } while (val & 0x1);
    return CH_OK;
}

uint32_t QSPI_WritePage(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf)
{
    int i;
    uint32_t *p = (uint32_t*)buf;
    
    SPIFI_SetCmd(&ctx->cmd[QSPI_WRITE_ENABLE_IDX]);
    SPIFI0->ADDR = (addr & 0x0FFFFFFF);
    SPIFI_SetCmd(&ctx->cmd[QSPI_PROGRAM_PAGE_IDX]);
    
    for (i=0; i<ctx->page_size/sizeof(uint32_t); i++)
    {
        SPIFI0->DAT32 = p[i];
    }

    QSPI_WaitComplete(ctx);
    return CH_OK;
}


uint32_t QSPI_WriteSector(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf)
{
    uint32_t i;
    for(i=0; i< (ctx->sector_size / ctx->page_size); i++)
    {
        QSPI_WritePage(ctx, addr + i*ctx->page_size, buf + i*ctx->page_size);
    }
    
    return CH_OK;
}

uint32_t QSPI_EraseSector(qspi_flash_t *ctx, uint32_t addr)
{
    SPIFI_SetCmd(&ctx->cmd[QSPI_WRITE_ENABLE_IDX]);
    SPIFI0->ADDR = (addr & 0x0FFFFFFF);
    SPIFI_SetCmd(&ctx->cmd[QSPI_ERASE_SECTOR_IDX]);
    return QSPI_WaitComplete(ctx);
}

void QSPI_SetReadMode(qspi_flash_t *ctx)
{
    SPIFI_Reset();
    SPIFI_SetMemoryCmd(&ctx->cmd[QSPI_CMD_READ_IDX]);
}

void QSPI_Read(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf, uint32_t len)
{
    QSPI_SetReadMode(ctx);
    memcpy(buf, (uint8_t*)addr, len);
}

uint32_t QSPI_SectorTest(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf)
{
    int i;
    uint8_t *p;
    
    p = (uint8_t *)(addr);
    
    SPIFI_Reset();
    QSPI_EraseSector(ctx, addr);
    
    QSPI_SetReadMode(ctx);
    
    for(i=0; i<ctx->sector_size; i++)
    {
        if(p[i] != 0xFF)
        {
            printf("erase err %d\r\n", i);
            return CH_ERR;
        }
    }
    
    for(i=0; i<ctx->sector_size; i++)
    {
        buf[i] = i & 0xFF;
    }
    
    SPIFI_Reset();
    QSPI_WriteSector(ctx, addr, buf);
    
    /* memory read */
    QSPI_SetReadMode(ctx);
    
    for(i=0; i<ctx->sector_size; i++)
    {
        if(p[i] != (i & 0xFF))
        {
            printf("err read:%X shoud:%X\r\n", p[i], (i % 0xFF));
            return CH_ERR;
        }
    }
    
    return CH_OK;
}

#endif

