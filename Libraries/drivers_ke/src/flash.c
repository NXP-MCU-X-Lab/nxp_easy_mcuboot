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


#define SECTOR_SIZE     (512)

#define FTMRH_FCLKDIV_FDIVLD_MASK                0x80u
#define FTMRH_FSTAT_CCIF_MASK                    0x80u
#define FTMRH_FSTAT_ACCERR_MASK                  0x20u
#define FTMRH_FSTAT_FPVIOL_MASK                  0x10u
#define FTMRH_FSTAT_MGSTAT_MASK                  0x3u
#define FTMRH_ERROR                              (FTMRH_FSTAT_ACCERR_MASK | FTMRH_FSTAT_FPVIOL_MASK | FTMRH_FSTAT_MGSTAT_MASK)
#define FTFx_SSD_FSTAT_FPVIOL                   (8)


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
 * @brief  初始化Flash
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
void FLASH_Init(void)
{
    FTMRH->FCLKDIV = 0x1F;
    MCM->PLACR = 0x1BC00; /* Disable flash cache */
}

 /**
 * @brief  擦除Flash扇区
 * @note   该功能将删除一个Flash扇区的内容
 * @param  addr: 擦除区域起始地址
 * @retval 返回操作结果
 */
uint8_t FLASH_EraseSector(uint32_t addr)
{
    // Clear error flags
    FTMRH->FSTAT = 0x30;
    
    // Write index to specify the command code to be loaded
    FTMRH->FCCOBIX = 0;
    // Write command code and memory address bits[23:16]	
    FTMRH->FCCOBHI = 0x0A;                            // Flash Sector Erase command

    FTMRH->FCCOBLO = (uint8_t)((addr >> 16)&0x007f);   // memory address bits[17:16]
    // Write index to specify the lower byte memory address bits[15:0] to be loaded
    FTMRH->FCCOBIX = 0x1;
    // Write the lower byte memory address bits[15:0]
    //FTMRH_FCCOB = (uint16_t)adr;
    FTMRH->FCCOBHI = (uint8_t)(addr >>  8);
    FTMRH->FCCOBLO = (uint8_t)(addr);

    // Launch the command
    FTMRH->FSTAT = 0x80;
    
    // Wait till command is completed
    while (!(FTMRH->FSTAT & FTMRH_FSTAT_CCIF_MASK));

    if (FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)  return(FTFx_SSD_FSTAT_FPVIOL);
    if (FTMRH->FSTAT & FTMRH_ERROR)              return(1);
    return (0);
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
    int  i;

    for (i = 0; i < ((len+7)/8); i++)
    {
        // Clear error flags
        FTMRH->FSTAT = 0x30;
    
        // Write index to specify the command code to be loaded
        FTMRH->FCCOBIX = 0;
        // Write command code and memory address bits[17:16]	
        FTMRH->FCCOBHI = 0x06;// program P-FLASH command
        FTMRH->FCCOBLO = (uint8_t) ((addr >> 16));// Addr[22:16] always 0
        // Write index to specify the lower byte memory address bits[15:0] to be loaded
        FTMRH->FCCOBIX = 0x1;
        // Write the lower byte memory address bits[15:0]
        FTMRH->FCCOBHI = (uint8_t)(addr >>  8);
        FTMRH->FCCOBLO = (uint8_t)(addr);

        // Write index to specify the word0 (MSB word) to be programmed
        FTMRH->FCCOBIX = 0x2;
        // Write the word0
        FTMRH->FCCOBHI =  buf[1];
        FTMRH->FCCOBLO =  buf[0];
        // Write index to specify the word1 (LSB word) to be programmed
        FTMRH->FCCOBIX = 0x3;
        // Write the word1
        FTMRH->FCCOBHI =  buf[3];
        FTMRH->FCCOBLO =  buf[2];
    
        // Write the word2
		FTMRH->FCCOBIX = 0x4;
        FTMRH->FCCOBHI =  buf[5];
        FTMRH->FCCOBLO =  buf[4];

        // Write the word3
        FTMRH->FCCOBIX = 0x5;
        FTMRH->FCCOBHI =  buf[7];
        FTMRH->FCCOBLO =  buf[6];
    
        // Launch the command
        FTMRH->FSTAT = 0x80;
        // Wait till command is completed
        while (!(FTMRH->FSTAT & FTMRH_FSTAT_CCIF_MASK));
        // Check error status
        if (FTMRH->FSTAT & FTMRH_ERROR) return(1);
        buf += 8;
        addr += 8;

    }
    return (0);                                  // Finished without Errors
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
