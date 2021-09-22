/**
  ******************************************************************************
  * @file    flash.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.6
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include <string.h>

#include "flash.h"
#include "common.h"

unsigned long CCLK = 12000;            // CCLK in kHz

struct sIAP
{                  // IAP Structure
    unsigned long cmd;           // Command
    unsigned long par[4];        // Parameters
    unsigned long stat;          // Status
    unsigned long res[2];        // Result
} IAP;

/* IAP Call */
typedef void (*IAP_Entry) (unsigned long *cmd, unsigned long *stat);

#if defined(LPC54114) || defined(LPC51U68)
#define IAP_Call ((IAP_Entry) 0x03000205)
#define PHY_PAGE_SIZE       (256)       /* actual flash page size */
#define PAGE_COUNT      (LOG_PAGE_SIZE/PHY_PAGE_SIZE)
#define SECTOR_SIZE         (32*1024)
#elif defined(LPC54608)
#define IAP_Call ((IAP_Entry) 0x03000205)
#define PHY_PAGE_SIZE       (256)       /* actual flash page size */
#define PAGE_COUNT      (LOG_PAGE_SIZE/PHY_PAGE_SIZE)
#define SECTOR_SIZE         (32*1024)
#else
#error "Unsupported flash drivers!"
#endif

unsigned long GetSecNum (unsigned long adr)
{
    unsigned long n;

    n = adr / SECTOR_SIZE;
    return (n);
}

 /**
 * @brief  获得扇区大小
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
uint32_t FLASH_GetSectorSize(void)
{
    return SECTOR_SIZE;
}

uint32_t FLASH_GetPageSize(void)
{
    return PHY_PAGE_SIZE;
}

 /**
 * @brief  初始化Flash
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
void FLASH_Init(void)
{
    CCLK = GetClock(kCoreClock)/1000;
}


/* 1=USART, 2=USB, 5=I2C, 6=SPI */
uint32_t ISP_Reinvoke(uint8_t mode)
{
    IAP.cmd    = 57;
    IAP.par[0] = mode;  
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed
    return 0;
}

 /**
 * @brief  擦除Flash扇区
 * @note   该功能将删除一个Flash扇区的内容
 * @param  addr: 擦除区域起始地址
 * @retval 返回操作结果
 */
uint8_t FLASH_EraseSector(uint32_t addr)
{
    unsigned long n;

    n = GetSecNum(addr);                          // Get Sector Number

    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP.par[2] = CCLK;                           // CCLK in kHz
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    return (0);                                  // Finished without Errors
}

uint32_t ISP_GetUID(void)
{
#if defined(LPC54114)
    uint32_t uid0, uid1, uid2;
    /* from 0x0100_0100 to 0x0100_010C */
    uid0 = *(uint32_t *)(0x01000100);
    uid1 = *(uint32_t *)(0x01000104);
    uid2 = *(uint32_t *)(0x01000108);
    return uid0 ^ uid1 ^ uid2;
#else
    IAP.cmd    = 58;                             // Prepare Sector for Erase
    IAP.par[0] = 0;                              // Start Sector
    IAP.par[1] = 0;                              // End Sector
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed

    return IAP.res[0] & IAP.res[1];
#endif
}

uint8_t FLASH_ErasePage(uint32_t addr)
{
    unsigned long n;
    unsigned long page;
    
    n = GetSecNum(addr);                          // Get Sector Number

    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed
    
    page = addr / PHY_PAGE_SIZE;
    
    IAP.cmd    = 59;
    IAP.par[0] = page;
    IAP.par[1] = page;
    IAP.par[2] = CCLK;
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);
    __enable_irq();
    if (IAP.stat) return (1);

    return (0);
}


 /**
 * @brief  write a flash page
 * @note   
 * @param  addr: start address, must be align with 256 bytes
 * @param  buf : buf pointer
 * @param  len : len of buffer
 * @retval CH_OK or CH_ERR
 */
uint8_t FLASH_WritePage(uint32_t addr, const uint8_t *buf)
{
    unsigned long n;

#if SET_VALID_CODE != 0                        // Set valid User Code Signature
  if (adr == 0) {                              // Check for Vector Table
    n = *((unsigned long *)(buf + 0x00)) +
        *((unsigned long *)(buf + 0x04)) +
        *((unsigned long *)(buf + 0x08)) +
        *((unsigned long *)(buf + 0x0C)) +
        *((unsigned long *)(buf + 0x10)) +
        *((unsigned long *)(buf + 0x14)) +
        *((unsigned long *)(buf + 0x18));
    *((unsigned long *)(buf + 0x1C)) = 0 - n;  // Signature at Reserved Vector
  }
#endif

    n = GetSecNum(addr);                          // Get Sector Number
  
    IAP.cmd    = 50;                             // Prepare Sector for Write
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 51;                             // Copy RAM to Flash
    IAP.par[0] = addr;                            // Destination Flash Address
    IAP.par[1] = (unsigned long)buf;             // Source RAM Address
    IAP.par[2] = PHY_PAGE_SIZE;                 // Fixed Page Size
    IAP.par[3] = CCLK;                           // CCLK in kHz
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    return CH_OK;
}

static uint32_t FLASH_PageTest(uint32_t addr)
{
    int i;
    uint8_t *p;
    ALIGN(8) uint8_t buf[PHY_PAGE_SIZE];
    
    FLASH_ErasePage(addr);
    memset(buf, 0, sizeof(buf));
    FLASH_WritePage(addr, buf);
    
    for(i=0; i<sizeof(buf); i++)
    {
        buf[i] = i % 0xFF;
    }
    
    uint32_t ret;
    ret = FLASH_ErasePage(addr);
    if(ret)
    {
        LIB_TRACE("FLASH_ErasePage failed\r\n");
    }
    
    ret = FLASH_WritePage(addr, buf);
    if(ret)
    {
        LIB_TRACE("FLASH_WritePage failed\r\n");
    }
    
    p = (uint8_t*)(addr);
    for(i=0; i<sizeof(buf); i++)
    {
        if(p[i] != (i%0xFF))
        {
            ret++;
            LIB_TRACE("ERR:[%d]:0x%02X ", i, *p); 
            return CH_ERR;
        }
    }
    LIB_TRACE("page test: %dKB ok\r\n", addr/1024);
    return CH_OK;
}

uint32_t FLASH_Test(uint32_t addr, uint32_t len)
{
    int  ret;
    FLASH_Init();
    uint32_t start = addr;
    
    while(addr <= start + len)
    {
        ret = FLASH_PageTest(addr);
        if(ret)
        {
            return ret;
        }
        addr += FLASH_GetPageSize();
    }
    return ret;
}
