/**
  ******************************************************************************
  * @file    emc.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.29
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_EMC_H__
#define __CH_LIB_EMC_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>


/*! @brief EMC dynamic read strategy. */
typedef enum 
{
    kEMC_NoDelay = 0x0U,      /*!< No delay. */
    kEMC_Cmddelay,            /*!< Command delayed strategy, using EMCCLKDELAY. */
    kEMC_CmdDelayPulseOneclk, /*!< Command delayed strategy pluse one clock cycle using EMCCLKDELAY. */
    kEMC_CmddelayPulsetwoclk, /*!< Command delayed strategy pulse two clock cycle using EMCCLKDELAY. */
} emc_dynamic_read_t;

/*! @brief EMC dynamic timing/delay configure structure. */
typedef struct 
{
    emc_dynamic_read_t readConfig;  /* Dynamic read strategy. */
    uint32_t refreshPeriod_Nanosec; /*!< The refresh period in unit of nanosecond. */
    uint32_t tRp_Ns;                /*!< Precharge command period in unit of nanosecond. */
    uint32_t tRas_Ns;               /*!< Active to precharge command period in unit of nanosecond. */
    uint32_t tSrex_Ns;              /*!< Self-refresh exit time in unit of nanosecond. */
    uint32_t tApr_Ns;               /*!< Last data out to active command time in unit of nanosecond. */
    uint32_t tDal_Ns;               /*!< Data-in to active command in unit of nanosecond. */
    uint32_t tWr_Ns;                /*!< Write recovery time in unit of nanosecond. */
    uint32_t tRc_Ns;                /*!< Active to active command period in unit of nanosecond. */
    uint32_t tRfc_Ns;  /*!< Auto-refresh period and auto-refresh to active command period in unit of nanosecond. */
    uint32_t tXsr_Ns;  /*!< Exit self-refresh to active command time in unit of nanosecond. */
    uint32_t tRrd_Ns;  /*!< Active bank A to active bank B latency in unit of nanosecond. */
    uint8_t tMrd_Nclk; /*!< Load mode register to active command time in unit of EMCCLK cycles.*/
} emc_dynamic_timing_config_t;


/*! @brief EMC dynamic memory device. */
typedef enum
{
    kEMC_Sdram = 0x0U, /*!< Dynamic memory device: SDRAM. */
    kEMC_Lpsdram,      /*!< Dynamic memory device: Low-power SDRAM. */
} emc_dynamic_device_t;



/*!
 * @brief EMC dynamic memory controller independent chip configuration structure.
 * Please take refer to the address mapping table in the RM in EMC chapter when you
 * set the "devAddrMap". Choose the right Bit 14 Bit12 ~ Bit 7 group in the table
 * according to the bus width/banks/row/colum length for you device.
 * Set devAddrMap with the value make up with the seven bits (bit14 bit12 ~ bit 7)
 * and inset the bit 13 with 0.
 * for example, if the bit 14 and bit12 ~ bit7 is 1000001 is choosen according to the
 * 32bit high-performance bus width with 2 banks, 11 row lwngth, 8 column length.
 * Set devAddrMap with 0x81.
 */
typedef struct
{
    uint8_t chipIndex;                          /*!< Chip Index, range from 0 ~ EMC_DYNAMIC_MEMDEV_NUM - 1. */
    emc_dynamic_device_t   dynamicDevice;       /*!< All chips shall use the same device setting. mixed use are not supported. */
        
    uint8_t rAS_Nclk;                           /*!< Active to read/write delay tRCD. */
    uint16_t sdramModeReg;                      /*!< Sdram mode register setting. */
    uint16_t sdramExtModeReg;                   /*!< Used for low-power sdram device. The extended mode register. */
    uint8_t devAddrMap;                         /*!< dynamic device address mapping, choose the address mapping for your specific device. */
} emc_dynamic_chip_config_t;

void EMC_Init(void);
void SDRAM_Init(emc_dynamic_timing_config_t *dynTiming, emc_dynamic_chip_config_t *config);


#endif

