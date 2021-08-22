/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "flash.h"
#include "common.h"
#include "stdio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief FLEXSPI NOR flash driver Structure in XIP config */
extern flexspi_nor_config_t norflash_config;
/*! @brief FLEXSPI NOR flash driver Structure in flash driver */
static flexspi_nor_config_t norConfig;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Get FLEXSPI NOR Configuration Block */
void FLEXSPI_NorFlash_GetConfig(flexspi_nor_config_t *config)
{
	  /* Copy norflash config block from xip config */
    memcpy(config, &norflash_config, sizeof(flexspi_nor_config_t));
	
    /* Override some default config */
	  config->memConfig.deviceType           = kFLEXSPIDeviceType_SerialNOR;
    config->memConfig.deviceModeType       = kDeviceConfigCmdType_Generic;
	  config->memConfig.serialClkFreq        = kFLEXSPISerialClk_30MHz; //Safe Serial Flash Frequencey
	  config->ipcmdSerialClkFreq             = kFLEXSPISerialClk_30MHz; //Safe Clock frequency for IP command
    config->memConfig.controllerMiscOption = FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_SafeConfigFreqEnable);//Always enable Safe configuration Frequency
     
    /* Read Status */
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05U, READ_SDR, FLEXSPI_1PAD, 0x1U);

    /* Write Enable */
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06U, STOP, FLEXSPI_1PAD, 0x0U);

    /* Page Program - quad mode */
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32U, RADDR_SDR, FLEXSPI_1PAD, 0x18U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x04U, STOP, FLEXSPI_1PAD, 0x0U);

    /* Sector Erase */
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD7U, RADDR_SDR, FLEXSPI_1PAD, 0x18U);
}

 /**
 * @brief  获得扇区大小
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
uint32_t FLASH_GetSectorSize(void)
{
    return 4096;//QSPI Flash Sector Size
}

 /**
 * @brief  获得最小编程长度
 * @note   None
 * @param  None
 * @retval 256 or 512 for QSPI Flash
 */
uint32_t FLASH_GetProgramCmd(void)
{
    return 256;//QSPI Flash Page Program
}
 /**
 * @brief  初始化Flash
 * @note   None
 * @param  None
 * @retval None
 */
void FLASH_Init(void)
{
	  /* Clean up FLEXSPI NOR flash driver Structure */
    memset(&norConfig, 0U, sizeof(flexspi_nor_config_t));
    /* Setup FLEXSPI NOR Configuration Block */
    FLEXSPI_NorFlash_GetConfig(&norConfig);
	  /* Initializes the FLEXSPI module for the other FLEXSPI APIs */
    ROM_FLEXSPI_NorFlash_Init(0, &norConfig);
	  /* Reset the Flexspi's Cache */
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
}
 /**
 * @brief  反初始化Flash
 * @note   None
 * @param  None
 * @retval None
 */
void FLASH_DeInit(void)
{
	  /* Clear the FlexSPI LUT to avoid unexpected erase or program operion trigger */
	  memset(&norConfig, 0U, sizeof(flexspi_nor_config_t));
	  ROM_FLEXSPI_NorFlash_UpdateLut(0, NOR_CMD_LUT_SEQ_IDX_READSTATUS, norConfig.memConfig.lookupTable, sizeof(norConfig.memConfig.lookupTable)-(4*NOR_CMD_LUT_SEQ_IDX_READSTATUS));
    /* Reset the Flexspi's Cache */
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
}

 /**
 * @brief  擦除Flash扇区
 * @note   该功能将删除一个Flash扇区的内容
 * @param  addr: 擦除区域起始地址
 * @retval 返回操作结果
 */
status_t FLASH_EraseSector(uint32_t addr)
{
	  status_t status;
	  addr &= 0x0FFFFFFF;
	
    __disable_irq();
    status = ROM_FLEXSPI_NorFlash_Erase(0, &norConfig, addr, norConfig.sectorSize);
    __enable_irq();
    
    return status;
}

 /**
 * @brief  写Flash一个页
 * @note   字节数小于等于一页
 * @param  addr: 开始地址
 * @param  buf : 写入数据起始指针
 * @param  len : 字节数
 * @retval kStatus_Success：完成
 */
status_t FLASH_WritePage(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    status_t status;
    addr &= 0x0FFFFFFF;
	
    __disable_irq();
	  norConfig.pageSize = len;
    status = ROM_FLEXSPI_NorFlash_ProgramPage(0, &norConfig, addr, (const uint32_t *)buf);
    __enable_irq();

    return status;
}
 /**
 * @brief  读Flash内容
 * @param  addr: 开始地址
 * @param  buf : 读缓存指针
 * @param  len : 字节数
 * @retval kStatus_Success：完成
 */
status_t FLASH_Read(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    status_t status;
	  flexspi_xfer_t flashXfer;
    
	  addr &= 0x0FFFFFFF;

	  flashXfer.operation = kFLEXSPIOperation_Read;
    flashXfer.seqNum = 1;
	  flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_READ;
    flashXfer.baseAddress = addr;
    flashXfer.isParallelModeEnable = false;	
	  flashXfer.rxBuffer = (uint32_t *)buf;
    flashXfer.rxSize = len;
	
    __disable_irq();
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
    status = ROM_FLEXSPI_NorFlash_CommandXfer(0, &flashXfer);
    __enable_irq();

    return status;
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
	  
	  uint32_t Sector_Size = FLASH_GetSectorSize();
    
    LIB_TRACE("program addr:0x%X(%dKB) ...", addr, addr/1024);
    ret = FLASH_EraseSector(addr);
    
    for(i=0; i<sizeof(buf); i++)
    {
        buf[i] = i % 0xFF;
    }
    
    for(i=0; i<(Sector_Size/sizeof(buf)); i++)
    {
        ret += FLASH_WritePage(addr + sizeof(buf)*i, buf, sizeof(buf));  
        if(ret)
        {
            LIB_TRACE("issue command failed\r\n");
            return CH_ERR;
        }
    }
    
    LIB_TRACE("varify addr:0x%X ...", addr);
    for(i=0; i<(Sector_Size/sizeof(buf)); i++)
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
	  uint32_t Sector_Size = FLASH_GetSectorSize();
	
    FLASH_SetcorTest(addr);
    FLASH_SetcorTest(addr + Sector_Size);
    FLASH_EraseSector(addr);
    p = (uint8_t*)(addr);
    
    for(i=0; i<Sector_Size*2; i++)
    {
        /* if (all content = 0xFF), then there is a error */
        if(*p++ != 0xFF)
        {
            break;
        }
    }
    
    if(i == Sector_Size*2)
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
	  uint32_t Sector_Size = FLASH_GetSectorSize();
	
    FLASH_Init();

    for(i=0; i<(len/Sector_Size); i++)
    {
        ret = FLASH_SetcorTest(addr + i*Sector_Size);
        if(ret != CH_OK)
        {
            return ret;
        }
    }
    return ret;
}
