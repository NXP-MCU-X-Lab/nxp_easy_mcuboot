/**
  ******************************************************************************
  * @file    sdif.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.29
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_SDIF_H__
#define __CH_LIB_SDIF_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t index;              /*!< Command index */
    uint32_t argument;           /*!< Command argument */
    uint32_t response[4U];       /*!< Response for this command */
    uint32_t responseType;       /*!< Command response type */
    uint32_t flags;              /*!< Cmd flags */
    uint32_t block_size;
    uint32_t block_cnt;
    uint32_t responseErrorFlags; /*!< response error flags, need to check the flags when recieve the cmd response */
}sdif_cmd_t;


#define MMC_PRODUCT_NAME_BYTES (6U)
typedef struct
{
    uint8_t manufacturerID;                      /*!< Manufacturer ID */
    uint16_t applicationID;                      /*!< OEM/Application ID */
    uint8_t productName[MMC_PRODUCT_NAME_BYTES]; /*!< Product name */
    uint8_t productVersion;                      /*!< Product revision */
    uint32_t productSerialNumber;                /*!< Product serial number */
    uint8_t manufacturerData;                    /*!< Manufacturing date */
} mmc_cid_t;

typedef struct
{
    uint8_t csdStructure;        /*!< CSD structure [127:126] */
    uint8_t dataReadAccessTime1; /*!< Data read access-time-1 [119:112] */
    uint8_t dataReadAccessTime2; /*!< Data read access-time-2 in clock cycles (NSAC*100) [111:104] */
    uint8_t transferSpeed;       /*!< Maximum data transfer rate [103:96] */
    uint16_t cardCommandClass;   /*!< Card command classes [95:84] */
    uint8_t readBlockLength;     /*!< Maximum read data block length [83:80] */
    uint16_t flags;              /*!< Flags in _sd_csd_flag */
    uint32_t deviceSize;         /*!< Device size [73:62] */
    /* Following fields from 'readCurrentVddMin' to 'deviceSizeMultiplier' exist in CSD version 1 */
    uint8_t readCurrentVddMin;    /*!< Maximum read current at VDD min [61:59] */
    uint8_t readCurrentVddMax;    /*!< Maximum read current at VDD max [58:56] */
    uint8_t writeCurrentVddMin;   /*!< Maximum write current at VDD min [55:53] */
    uint8_t writeCurrentVddMax;   /*!< Maximum write current at VDD max [52:50] */
    uint8_t deviceSizeMultiplier; /*!< Device size multiplier [49:47] */

    uint8_t eraseSectorSize;       /*!< Erase sector size [45:39] */
    uint8_t writeProtectGroupSize; /*!< Write protect group size [38:32] */
    uint8_t writeSpeedFactor;      /*!< Write speed factor [28:26] */
    uint8_t writeBlockLength;      /*!< Maximum write data block length [25:22] */
    uint8_t fileFormat;            /*!< File format [11:10] */
} sd_csd_t;

typedef struct
{
    mmc_cid_t cid;
    sd_csd_t  csd;
    uint32_t rel_addr;
    uint32_t block_cnt;
    uint32_t block_size;
}sd_card_t;

uint32_t SDIF_SendCmd(sdif_cmd_t *cmd, uint32_t timeout);
void SDIF_Init(void);
void SDIF_SetClock(uint32_t src_clk, uint32_t hz);
void SDIF_SetBusWidth(uint8_t width_bit);



uint32_t SDCardInit(sd_card_t *card, uint32_t hz);
bool SD_IsCardInsert(void);
uint32_t SD_ReadBlock(sd_card_t *card, uint32_t block_addr, uint32_t block_cnt, uint8_t *buf);
uint32_t SD_WriteBlock(sd_card_t *card, uint32_t block_addr, uint32_t block_cnt, uint8_t *buf);
uint32_t SD_SetCardDataBusWidth(sd_card_t *card, uint8_t width_in_bit);

bool SD_IsCardIdle(sd_card_t *card);
uint32_t SD_BlockTest(sd_card_t *card, uint8_t *buf, uint32_t block_addr, uint32_t block_cnt);



#ifdef __cplusplus
}
#endif

#endif


