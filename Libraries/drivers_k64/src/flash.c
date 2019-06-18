/**
  ******************************************************************************
  * @file    flash.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.6
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "flash.h"
#include "common.h"

/* flash commands */
#define RD1BLK    0x00  /* read 1 block */
#define RD1SEC    0x01  /* read 1 section */
#define PGMCHK    0x02  /* program check */
#define RDRSRC    0x03  /* read resource */
#define PGM4      0x06  /* program phase program 4 byte */
#define PGM8      0x07  /* program phase program 8 byte */
#define ERSBLK    0x08  /* erase flash block */
#define ERSSCR    0x09  /* erase flash sector */
#define PGMSEC    0x0B  /* program section */
#define RD1ALL    0x40  /* read 1s all block */
#define RDONCE    0x41  /* read once */
#define PGMONCE   0x43  /* program once */
#define ERSALL    0x44  /* erase all blocks */
#define VFYKEY    0x45  /* verift backdoor key */
#define PGMPART   0x80  /* program paritition */
#define SETRAM    0x81  /* set flexram function */
#define NORMAL_LEVEL 0x0


/* disable interrupt before lunch command */
#define CCIF    (1<<7)
#define ACCERR  (1<<5)
#define FPVIOL  (1<<4)
#define MGSTAT0 (1<<0)


#if defined(FTFL)
#define FTF    FTFL
#define SECTOR_SIZE     (2048)
#define PROGRAM_CMD      PGM4
#elif defined(FTFE)
#define FTF    FTFE
#define SECTOR_SIZE     (4096)
#define PROGRAM_CMD      PGM8
#elif defined(FTFA)
    #if (__CORTEX_M == 0)
        #if defined(MKL28Z7)
        #define SECTOR_SIZE     (2048)
        #else
        #define SECTOR_SIZE     (1024)
        #endif
    #else
    #define SECTOR_SIZE     (2048)
    #endif
#define PROGRAM_CMD      PGM4
#define FTF    FTFA
#endif


volatile uint8_t s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8_t *reg);
flash_run_entry_t s_flash_run_entry;
    
static inline uint8_t FlashCmdStart(void)
{
    /* clear command result flags */
    FTF->FSTAT = ACCERR | FPVIOL;
    s_flash_run_entry = (flash_run_entry_t)((uint32_t)s_flash_command_run + 1);
    s_flash_run_entry(&FTF->FSTAT);
    
    if(FTF->FSTAT & (ACCERR | FPVIOL | MGSTAT0)) return CH_ERR;
    return CH_OK;
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

 /**
 * @brief  获得最小编程长度
 * @note   None
 * @param  None
 * @retval 4或8
 */
uint32_t FLASH_GetProgramCmd(void)
{
    uint32_t len;
    (PROGRAM_CMD == PGM4)?(len = 4):(len = 8);
    return len;
}

 /**
 * @brief  初始化Flash
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
void FLASH_Init(void)
{
    /* clear status */
    FTF->FSTAT = ACCERR | FPVIOL;
}

 /**
 * @brief  擦除Flash扇区
 * @note   该功能将删除一个Flash扇区的内容
 * @param  addr: 擦除区域起始地址
 * @retval 返回操作结果
 */
uint8_t FLASH_EraseSector(uint32_t addr)
{
    int ret;
    
    /* data flash */
    if(addr >= 0x10000000)
    {
        addr |= (1<<23);
    }
    
	union
	{
		uint32_t  word;
		uint8_t   byte[4];
	} dest;
	dest.word = (uint32_t)addr;

    /* set cmd */
	FTF->FCCOB0 = ERSSCR; 
	FTF->FCCOB1 = dest.byte[2];
	FTF->FCCOB2 = dest.byte[1];
	FTF->FCCOB3 = dest.byte[0];
    __disable_irq();
    ret = FlashCmdStart();
    __enable_irq();
    
    return ret;
}

 /**
 * @brief  写Flash一个扇区
 * @note   字节数必须等于扇区尺寸
 * @param  addr: 开始地址
 * @param  buf : 写入数据起始指针
 * @param  len : 字节数
 * @retval CH_OK：完成；CH_ERR：失败
 */
uint8_t FLASH_WriteSector(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint16_t step, ret, i;
    
    /* data flash */
    if(addr >= 0x10000000)
    {
        addr |= (1<<23);
    }
    
	union
	{
		uint32_t  word;
		uint8_t   byte[4];
	} dest;
	dest.word = (uint32_t)addr;

	FTF->FCCOB0 = PROGRAM_CMD;
    
    switch(PROGRAM_CMD)
    {
        case PGM4:
            step = 4;
            break;
        case PGM8:
            step = 8;
            break;
        default:
            LIB_TRACE("FLASH: no program cmd found!\r\n");
            step = 4;
            break;
    }
    
	for(i=0; i<len; i+=step)
	{
        /* set address */
		FTF->FCCOB1 = dest.byte[2];
		FTF->FCCOB2 = dest.byte[1];
		FTF->FCCOB3 = dest.byte[0];

		FTF->FCCOB4 = buf[3];
		FTF->FCCOB5 = buf[2];
		FTF->FCCOB6 = buf[1];
		FTF->FCCOB7 = buf[0];
        
        if(step == 8)
        {
            FTF->FCCOB8 = buf[7];
            FTF->FCCOB9 = buf[6];
            FTF->FCCOBA = buf[5];
            FTF->FCCOBB = buf[4];
        }

		dest.word += step; buf += step;

        __disable_irq();
        ret = FlashCmdStart();
        __enable_irq();
        
		if(CH_OK != ret) 
        {
            return CH_ERR;
        }
    }
    return CH_OK;
}

 /**
 * @brief  设置扇区测试
 * @note   None
 * @param  addr: 开始地址
 * @retval CH_OK：完成；CH_ERR：失败
 */
static uint32_t FLASH_SetcorTest(uint32_t addr)
{
    uint32_t ret, i, j;
    uint8_t *p;
    ALIGN(8) uint8_t buf[32];
    
    LIB_TRACE("program addr:0x%X(%dKB) ...", addr, addr/1024);
    ret = FLASH_EraseSector(addr);
    
    for(i=0; i<sizeof(buf); i++)
    {
        buf[i] = i % 0xFF;
    }
    
    for(i=0; i<(SECTOR_SIZE/sizeof(buf)); i++)
    {
        ret += FLASH_WriteSector(addr + sizeof(buf)*i, buf, sizeof(buf));  
        if(ret)
        {
            LIB_TRACE("issue command failed\r\n");
            return CH_ERR;
        }
    }
    
    LIB_TRACE("varify addr:0x%X ...", addr);
    for(i=0; i<(SECTOR_SIZE/sizeof(buf)); i++)
    {
        p = (uint8_t*)(addr + sizeof(buf)*i);
        for(j=0; j<sizeof(buf); j++)
        {
            if(p[j] != (j%0xFF))
            {
                ret++;
                LIB_TRACE("ERR:[%d]:0x%02X ", i, *p); 
            }
        }
    }
    
    if(ret == 0)
    {
        LIB_TRACE("OK\r\n"); 
    }
    return ret;
}

 /**
 * @brief  设置扇区尺寸测试
 * @note   None
 * @param  addr: 开始地址
 * @retval CH_OK：完成；CH_ERR：失败
 */
static uint32_t FLASH_SetcorSizeTest(uint32_t addr)
{
    int i;
    uint8_t *p;
    FLASH_SetcorTest(addr);
    FLASH_SetcorTest(addr + SECTOR_SIZE);
    FLASH_EraseSector(addr);
    p = (uint8_t*)(addr);
    
    for(i=0; i<SECTOR_SIZE*2; i++)
    {
        /* if (all content = 0xFF), then there is a error */
        if(*p++ != 0xFF)
        {
            break;
        }
    }
    
    if(i == SECTOR_SIZE*2)
    {
        LIB_TRACE("Flash SectorTest Error in %d\r\n", i);
        return CH_ERR;
    }
    
    return CH_OK;
}

 /**
 * @brief  Flash自测
 * @note   确保有足够的栈空间
 * @param  addr: 开始地址
 * @retval CH_OK：完成；CH_ERR：失败
 */
uint32_t FLASH_Test(uint32_t addr, uint32_t len)
{
    int i, ret;
    FLASH_Init();
    FLASH_SetcorSizeTest(addr);
    for(i=0; i<(len/SECTOR_SIZE); i++)
    {
        ret = FLASH_SetcorTest(addr + i*SECTOR_SIZE);
        if(ret != CH_OK)
        {
            return ret;
        }
    }
    return ret;
}
