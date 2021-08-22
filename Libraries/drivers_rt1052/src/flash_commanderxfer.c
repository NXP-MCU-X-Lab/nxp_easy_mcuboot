/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "flash.h"
#include "fsl_romapi.h"
#include "common.h"
#include "stdio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define HYPER_FLASH

#define FLEXSPI_LUT_KEY_VAL (0x5AF05AF0ul)    //!< FlesSPI Unlock/Lock Key
#define FLEXSPI_WAIT_TIMEOUT_NS (500000000UL) //!< FlexSPI timeout value, 500ms
#define FLEXSPI_FREQ_1GHz (1000000000UL)

#define FREQ_1MHz (1000000UL)
#define FLEXSPI_DLLCR_DEFAULT (0x100UL)
enum
{
    kFlexSpiDelayCellUnit_Min = 75,  // 75ps
    kFlexSpiDelayCellUnit_Max = 225, // 225ps
};

//!@brief FlexSPI Clock Type
typedef enum
{
    kFlexSpiClock_CoreClock,       //!< ARM Core Clock
    kFlexSpiClock_AhbClock,        //!< AHB clock
    kFlexSpiClock_SerialRootClock, //!< Serial Root Clock
    kFlexSpiClock_IpgClock,        //!< IPG clock
} flexspi_clock_type_t;

typedef struct LookUpTable
{
	uint32_t Read_Seq[4];
	uint32_t ReadStatus_Seq[8];
	uint32_t WriteEnable_Seq[8];
	uint32_t EraseSector_Seq[16];
	uint32_t PageProgram_Seq[8];
	uint32_t ChipErase_Seq[8];
	uint32_t ReadSfdp_Seq[4];
	uint32_t Restore_NoCmd_Seq[4];
	uint32_t Exit_NoCmd_Seq[4];
}lookuptable_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t Flexspi_Nor_Wait_Busy(uint32_t instance, uint32_t baseAddr);
static status_t Flexspi_Nor_Write_Enable(uint32_t instance, uint32_t baseAddr);
static void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode);
static void flexspi_clock_gate_enable(uint32_t instance);
static void flexspi_clock_gate_disable(uint32_t instance);
static status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq);
static status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit);
static status_t flexspi_configure_dll(uint32_t instance, flexspi_mem_config_t *config);
static status_t flexspi_config_mcr1(uint32_t instance, flexspi_mem_config_t *config);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static const lookuptable_t FlashLookupTable={
#ifndef HYPER_FLASH
/* QSPI Flash */
/* Read Status */
.ReadStatus_Seq=
{
	FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),
},
/* Write Enable */
.WriteEnable_Seq=
{
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x00),
},	
/* Erase Sector */
.EraseSector_Seq=
{
	FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18);
},
/* Page Program */
.PageProgram_Seq=
{
	FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32, RADDR_SDR, FLEXSPI_1PAD, 0x18);
	FSL_ROM_FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x00, 0x00, 0x00, 0x00);
},
#else
 /* Read Status */
.ReadStatus_Seq=
{
	// 0
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x70U),
  // 1
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0U, RADDR_DDR, FLEXSPI_8PAD, 0x18U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, DUMMY_RWDS_DDR, FLEXSPI_8PAD, 0x0BU),
  FSL_ROM_FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04U, STOP, FLEXSPI_1PAD, 0x00U),
	FSL_ROM_FLEXSPI_LUT_SEQ(0, 0, 0, 0, 0, 0),
},
/* Write Enable */
.WriteEnable_Seq=
{
	 // 0
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
   // 1
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x02U),
   FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U),
},
/* Erase Sector */
.EraseSector_Seq=
{
  //0
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x80U),
  // 1
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
  // 2
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x02U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U),
  // 3
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, RADDR_DDR, FLEXSPI_8PAD, 0x18U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x30U, STOP, FLEXSPI_1PAD, 0x0U),
  FSL_ROM_FLEXSPI_LUT_SEQ(0, 0, 0, 0, 0, 0),
},	
/* Page Program */
.PageProgram_Seq=
{
  //0
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xA0U),
  //1
  FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, RADDR_DDR, FLEXSPI_8PAD, 0x18U),
  FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, WRITE_DDR, FLEXSPI_8PAD, 0x80U),
  FSL_ROM_FLEXSPI_LUT_SEQ(0, 0, 0, 0, 0, 0),
  FSL_ROM_FLEXSPI_LUT_SEQ(0, 0, 0, 0, 0, 0),
},
#endif
};

 /**
 * @brief  获得扇区大小
 * @note   None
 * @param  None
 * @retval Flash扇区尺寸
 */
uint32_t FLASH_GetSectorSize(void)
{
#ifndef HYPER_FLASH
    return 4096;//QSPI Flash Sector Size
#else
    return 256*1024UL;//Hyper Flash Sector Size
#endif
}

 /**
 * @brief  获得最小编程长度
 * @note   None
 * @param  None
 * @retval 256 or 512 for QSPI Flash
 */
uint32_t FLASH_GetProgramCmd(void)
{
    uint32_t Program_Unit;
#ifndef HYPER_FLASH
    Program_Unit = 256;//QSPI Flash Page Program
#else
    Program_Unit = 512;//Hyper Flash Page Program
#endif

    return Program_Unit;
}
 /**
 * @brief  初始化Flash
 * @note   None
 * @param  None
 * @retval None
 */
__attribute__((section("RamFunction"))) 
void FLASH_Init(void)
{
	  /* Update LUT Table for Status, Write Enable, Erase and Program */
    ROM_FLEXSPI_NorFlash_UpdateLut(0, NOR_CMD_LUT_SEQ_IDX_READSTATUS, (const uint32_t *)FlashLookupTable.ReadStatus_Seq, 10U);
	  /* Use 30MHz Flexspi clock for safe operation */
    flexspi_clock_config(0, kFLEXSPISerialClk_30MHz, kFLEXSPIClk_DDR);
	  extern flexspi_nor_config_t norflash_config;
    flexspi_config_mcr1(0, &norflash_config.memConfig);
    flexspi_configure_dll(0, &norflash_config.memConfig);
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
}
 /**
 * @brief  反初始化Flash
 * @note   None
 * @param  None
 * @retval None
 */
__attribute__((section("RamFunction"))) 
void FLASH_DeInit(void)
{
	  lookuptable_t clearlut;
	  memset(&clearlut, 0, sizeof(lookuptable_t));
    ROM_FLEXSPI_NorFlash_UpdateLut(0, NOR_CMD_LUT_SEQ_IDX_READSTATUS, (const uint32_t *)&clearlut, 10U);
	  /* Use 30MHz Flexspi clock for safe operation */
    flexspi_clock_config(0, kFLEXSPISerialClk_133MHz, kFLEXSPIClk_DDR);
}

__attribute__((section("RamFunction"))) 
status_t Flexspi_Nor_Write_Enable(uint32_t instance, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
	  flexspi_xfer_t flashXfer;
    
    flashXfer.operation = kFLEXSPIOperation_Command;
#ifndef HYPER_FLASH
    flashXfer.seqNum = 1;
#else
    flashXfer.seqNum = 2;
#endif
    flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;
    flashXfer.isParallelModeEnable = false;
    flashXfer.baseAddress = baseAddr;
	
	  status = ROM_FLEXSPI_NorFlash_CommandXfer(instance, &flashXfer);

    return status;
}
__attribute__((section("RamFunction"))) 
status_t Flexspi_Nor_Wait_Busy(uint32_t instance, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
	  flexspi_xfer_t flashXfer;
	  uint32_t statusDataBuffer;
	  uint32_t busyMask;
    uint32_t busyPolarity;
    bool isBusy = false;
	
    flashXfer.operation = kFLEXSPIOperation_Read;
#ifndef HYPER_FLASH
    flashXfer.seqNum = 1;
	  busyMask = 1;
	  busyPolarity = 0;
#else
    flashXfer.seqNum = 2;
	  busyMask = 1<<15;
	  busyPolarity = 1;
#endif
    flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_READSTATUS;
	  flashXfer.isParallelModeEnable = false;
	  flashXfer.baseAddress = baseAddr;
    flashXfer.rxBuffer = &statusDataBuffer;	
		flashXfer.rxSize = sizeof(statusDataBuffer);
	  do
		{
	      status = ROM_FLEXSPI_NorFlash_CommandXfer(instance, &flashXfer);
		    // Busy bit is 0 if polarity is 1
        if (busyPolarity)
        {
            isBusy = (~statusDataBuffer) & busyMask;
        }
        else
        {
            isBusy = statusDataBuffer & busyMask;
        }
		}while (isBusy);
	
		return status;
}

 /**
 * @brief  擦除Flash扇区
 * @note   该功能将删除一个Flash扇区的内容
 * @param  addr: 擦除区域起始地址
 * @retval 返回操作结果
 */
__attribute__((section("RamFunction"))) 
uint8_t FLASH_EraseSector(uint32_t addr)
{
	  status_t status;
    flexspi_xfer_t flashXfer;
	  addr &= 0x0FFFFFFF;
	
	  flashXfer.operation = kFLEXSPIOperation_Command;
#ifndef HYPER_FLASH
    flashXfer.seqNum = 1;
#else
    flashXfer.seqNum = 4;
#endif
	  flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
    flashXfer.baseAddress = addr;
    flashXfer.isParallelModeEnable = false;
	  
    __disable_irq();
	  status = Flexspi_Nor_Write_Enable(0, addr);
    status = ROM_FLEXSPI_NorFlash_CommandXfer(0, &flashXfer);
	  status = Flexspi_Nor_Wait_Busy(0, addr);
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
    __enable_irq();
    
    return status;
}

 /**
 * @brief  写Flash一个扇区
 * @note   字节数必须等于扇区尺寸
 * @param  addr: 开始地址
 * @param  buf : 写入数据起始指针
 * @param  len : 字节数
 * @retval CH_OK：完成；CH_ERR：失败
 */
__attribute__((section("RamFunction"))) 
uint8_t FLASH_WriteSector(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    status_t status;
    flexspi_xfer_t flashXfer;
	  addr &= 0x0FFFFFFF;
	
	
	  flashXfer.operation = kFLEXSPIOperation_Write;
#ifndef HYPER_FLASH
    flashXfer.seqNum = 1;
#else
    flashXfer.seqNum = 4;
#endif
	  flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
    flashXfer.baseAddress = addr;
    flashXfer.isParallelModeEnable = false;	
	  flashXfer.txBuffer = (uint32_t *)buf;
    flashXfer.txSize = len;
    
    __disable_irq();
    status = Flexspi_Nor_Write_Enable(0, addr);
    status = ROM_FLEXSPI_NorFlash_CommandXfer(0, &flashXfer);
	  status = Flexspi_Nor_Wait_Busy(0, addr);
	  ROM_FLEXSPI_NorFlash_ClearCache(0);
    __enable_irq();

    return status;
}
//!@brief Configure clock for FlexSPI peripheral
__attribute__((section("RamFunction")))
void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{
    uint32_t pfd480 = 0;
    uint32_t cscmr1 = 0;
    uint32_t frac = 0;
    uint32_t podf = 0;

    typedef struct _flexspi_clock_param
    {
        uint8_t frac;
        uint8_t podf;
    } flexspi_clock_param_t;

    const flexspi_clock_param_t k_sdr_clock_config[kFLEXSPISerialClk_200MHz + 1] = {
        // Reserved, 30MHz     50MHz     60MHz        75MHz    80MHz       100MHz   133MHz       166MHz   200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 17, 4 }, { 26, 2 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kFLEXSPISerialClk_200MHz + 1] = {
        // Reserved, 30MHz,  50MHz,       60MHz,      75MHz,   80Mhz,   100MHz,      133MHz,   166MHz,     200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 26, 1 }, { 22, 1 }
    };

    do
    {
        if ((sampleClkMode != kFLEXSPIClk_SDR) && (sampleClkMode != kFLEXSPIClk_DDR))
        {
            break;
        }

        pfd480 = CCM_ANALOG->PFD_480 & (~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK);
        cscmr1 = CCM->CSCMR1 & (~CCM_CSCMR1_FLEXSPI_PODF_MASK);

        // Note: Per ANALOG IP Owner's recommendation, FRAC should be even number,
        //       PODF should be even nubmer as well if the divider is greater than 1

        const flexspi_clock_param_t *flexspi_config_array = NULL;
        if (sampleClkMode == kFLEXSPIClk_SDR)
        {
            flexspi_config_array = &k_sdr_clock_config[0];
        }
        else
        {
            flexspi_config_array = &k_ddr_clock_config[0];
        }

        if (freq >= kFLEXSPISerialClk_30MHz)
        {
            if (freq > kFLEXSPISerialClk_200MHz)
            {
                freq = kFLEXSPISerialClk_30MHz;
            }

            frac = flexspi_config_array[freq].frac;
            podf = flexspi_config_array[freq].podf;

            pfd480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(frac);
            cscmr1 |= CCM_CSCMR1_FLEXSPI_PODF(podf - 1);

            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);

            if (pfd480 != CCM_ANALOG->PFD_480)
            {
                CCM_ANALOG->PFD_480 = pfd480;
            }
            if (cscmr1 != CCM->CSCMR1)
            {
                CCM->CSCMR1 = cscmr1;
            }
            flexspi_clock_gate_enable(instance);
            FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        }
        else
        {
            // Do nothing
        }
    } while (0);
}
__attribute__((section("RamFunction")))
//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}
__attribute__((section("RamFunction")))
//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
}
__attribute__((section("RamFunction")))
//!@brief Configure DLL register
status_t flexspi_configure_dll(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    bool mdisConfigRequired;

    do
    {
        bool isUnifiedConfig = true;
        uint32_t flexspiRootClk;
        uint32_t flexspiDll[2];
        uint32_t dllValue;
        uint32_t temp;

        if (config->readSampleClkSrc > kFLEXSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            break;
        }

        switch (config->readSampleClkSrc)
        {
            case kFLEXSPIReadSampleClk_LoopbackInternally:
            case kFLEXSPIReadSampleClk_LoopbackFromDqsPad:
            case kFLEXSPIReadSampleClk_LoopbackFromSckPad:
                isUnifiedConfig = true;
                break;
            case kFLEXSPIReadSampleClk_ExternalInputFromDqsPad:
                isUnifiedConfig = false;
                break;
            default: // Never reach here
                break;
        }

        if (isUnifiedConfig)
        {
            flexspiDll[0] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
            flexspiDll[1] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
        }
				else
        {
            flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &flexspiRootClk);

            bool useDLL = false;

            // See FlexSPI Chapter for more details
            if ((flexspiRootClk >= 100 * FREQ_1MHz) &&
                (!(config->controllerMiscOption & (1U << kFLEXSPIMiscOffset_UseValidTimeForAllFreq))))
            {
                useDLL = true;
            }
            if (useDLL)
            {
                flexspiDll[0] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
                flexspiDll[1] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
            }
            // If Serial root closk is lower than 100MHz, DLL is unable to lock on
            // half cycle of serial root clock because the dealy cell number is limited
            // in delay chain, Then DLL should be configured as following instead:
            // OVRDEN = 0x01
            // OVRDVAL=N; each dealy cell in DLL is about 75ps - 225ps.
            // The delay of DLL delay chain ( N * delay_cell_delay) should be larger
            // than device output data valid time (from SCK edge to data valid).
            // The other condition is that some devices may be incompatible with current
            // FlexSPI defintions, so, use a backup way to support it.
            else
            {
                for (uint32_t i = 0; i < 2; i++)
                {
                    uint32_t dataValidTimeH = config->dataValidTime[i].delay_cells;
                    uint32_t dataValidTimeL = config->dataValidTime[i].time_100ps;
                    if (dataValidTimeH < 1)
                    {
                        // Convert the data valid time to n ps.
                        temp = dataValidTimeL * 100ul;
                        if (temp < 1)
                        {
                            uint32_t maxFreq = (166UL * 1000 * 1000);
													  bool is_ddr_enabled = (config->controllerMiscOption & 1<< kFLEXSPIMiscOffset_DdrModeEnable)?true:false;
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // Half cycle of max  supported frequency
                            if (!is_ddr_enabled)
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 2 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // 1/4 cycle of max supported frequency
                            else
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 4 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                        }
                        else
                        {
                            dllValue = temp / kFlexSpiDelayCellUnit_Min;
                            if (dllValue * kFlexSpiDelayCellUnit_Min < temp)
                            {
                                dllValue++;
                            }
                        }
                    }
                    else
                    {
                        dllValue = dataValidTimeH;
                    }
                    // Calculate maximum dll value;
                    temp = (FLEXSPI_DLLCR_OVRDVAL_MASK >> FLEXSPI_DLLCR_OVRDVAL_SHIFT);
                    if (dllValue > temp)
                    {
                        dllValue = temp;
                    }
                    flexspiDll[i] = FLEXSPI_DLLCR_OVRDEN(1) | FLEXSPI_DLLCR_OVRDVAL(dllValue);
                }
            }
        }

        if (FLEXSPI->MCR0 & FLEXSPI_MCR0_MDIS_MASK)
        {
            mdisConfigRequired = false;
        }
        else
        {
            mdisConfigRequired = true;
        }

        if (mdisConfigRequired)
        {
            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
        }

        FLEXSPI->DLLCR[0] = flexspiDll[0];
        FLEXSPI->DLLCR[1] = flexspiDll[1];

        if (mdisConfigRequired)
        {
            FLEXSPI->MCR0 &= (uint32_t)~FLEXSPI_MCR0_MDIS_MASK;
        }
        /* Wait at least 100 NOPs*/
        for (uint8_t delay = 100U; delay > 0U; delay--)
        {
            __NOP();
        }
        status = kStatus_Success;
    } while (0);

    return status;
}

__attribute__((section("RamFunction")))
status_t flexspi_config_mcr1(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t seqWaitTicks = 0xFFFFu;
    uint32_t ahbBusWaitTicks = 0xFFFFu;
    uint32_t serialRootClockFreq;
    uint32_t ahbBusClockFreq;

    if (config == NULL)
    {
        return kStatus_InvalidArgument;
    }

    flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &serialRootClockFreq);
    flexspi_get_clock(instance, kFlexSpiClock_AhbClock, &ahbBusClockFreq);
    flexspi_get_ticks(&seqWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, serialRootClockFreq, 1024);
    flexspi_get_ticks(&ahbBusWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, ahbBusClockFreq, 1024);

    if (seqWaitTicks > 0xFFFF)
    {
        seqWaitTicks = 0xFFFF;
    }
    if (ahbBusWaitTicks > 0xFFFF)
    {
        ahbBusWaitTicks = 0xFFFF;
    }

    // Configure MCR1
    FLEXSPI->MCR1 = FLEXSPI_MCR1_SEQWAIT(seqWaitTicks) | FLEXSPI_MCR1_AHBBUSWAIT(ahbBusWaitTicks);

    return kStatus_Success;
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    uint32_t ahbBusDivider;
    uint32_t seralRootClkDivider;
    uint32_t arm_clock = SystemCoreClock;

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = SystemCoreClock;
            break;
        case kFlexSpiClock_AhbClock:
        {
            // Note: In I.MXRT_512, actual AHB clock is IPG_CLOCK_ROOT
            ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
            clockFrequency = arm_clock / ahbBusDivider;
        }
        break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t pfdFrac;
            uint32_t pfdClk;

            // FLEXPI CLK SEL
            uint32_t flexspi_clk_src =
                (CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK) >> CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT;

            // PLL_480_PFD0
            pfdFrac = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
            pfdClk = (480000000UL) / pfdFrac * 18;

            seralRootClkDivider = ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT) + 1;

            clockFrequency = pfdClk / seralRootClkDivider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
    *freq = clockFrequency;

    return status;
}
// Calculate ticks for the timeout interms of specified unit.
status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((ticks == NULL) || (freq < 1) || (unit < 1))
        {
            break;
        }

        // Get clock cycle in terms of ns
        int32_t calculatedTicks;
        uint32_t cycleNs = FLEXSPI_FREQ_1GHz / freq;

        calculatedTicks = intervalNs / (cycleNs * unit);
        while (calculatedTicks * cycleNs * unit < intervalNs)
        {
            calculatedTicks++;
        }

        *ticks = calculatedTicks;

        status = kStatus_Success;

    } while (0);

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
        ret += FLASH_WriteSector(addr + sizeof(buf)*i, buf, sizeof(buf));  
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
