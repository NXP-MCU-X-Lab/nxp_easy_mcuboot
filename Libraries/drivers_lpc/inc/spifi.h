/**
  ******************************************************************************
  * @file    spifi.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.05.31
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_SPIFI_H__
#define __CH_LIB_SPIFI_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif


/*! @brief SPIFI data direction */
typedef enum _spifi_data_direction
{
    kSPIFI_DataInput = 0x0U, /*!< Data input from serial flash. */
    kSPIFI_DataOutput = 0x1U /*!< Data output to serial flash. */
} spifi_data_direction_t;

/*! @brief SPIFI command opcode format */
typedef enum _spifi_command_format
{
    kSPIFI_CommandAllSerial = 0x0,     /*!< All fields of command are serial. */
    kSPIFI_CommandDataQuad = 0x1U,     /*!< Only data field is dual/quad, others are serial. */
    kSPIFI_CommandOpcodeSerial = 0x2U, /*!< Only opcode field is serial, others are quad/dual. */
    kSPIFI_CommandAllQuad = 0x3U       /*!< All fields of command are dual/quad mode. */
} spifi_command_format_t;

/*! @brief SPIFI command type */
typedef enum _spifi_command_type
{
    kSPIFI_CommandOpcodeOnly = 0x1U,             /*!< Command only have opcode, no address field */
    kSPIFI_CommandOpcodeAddrOneByte = 0x2U,      /*!< Command have opcode and also one byte address field */
    kSPIFI_CommandOpcodeAddrTwoBytes = 0x3U,     /*!< Command have opcode and also two bytes address field */
    kSPIFI_CommandOpcodeAddrThreeBytes = 0x4U,   /*!< Command have opcode and also three bytes address field. */
    kSPIFI_CommandOpcodeAddrFourBytes = 0x5U,    /*!< Command have opcode and also four bytes address field */
    kSPIFI_CommandNoOpcodeAddrThreeBytes = 0x6U, /*!< Command have no opcode and three bytes address field */
    kSPIFI_CommandNoOpcodeAddrFourBytes = 0x7U   /*!< Command have no opcode and four bytes address field */
} spifi_command_type_t;
     
/*! @brief SPIFI command structure */
typedef struct
{
    uint16_t dataLen;                 /*!< How many data bytes are needed in this command. */
    spifi_data_direction_t direction; /*!< Data direction of this command. */
    uint8_t intermediateBytes;        /*!< How many intermediate bytes needed */
    spifi_command_format_t format;    /*!< Command format */
    spifi_command_type_t type;        /*!< Command type */
    uint8_t opcode;                   /*!< Command opcode value */
} spifi_command_t;


typedef struct
{
    spifi_command_t *cmd;
    uint32_t sector_size;
    uint32_t page_size;
}qspi_flash_t;


void SPIFI_Init(uint32_t baud);
void SPIFI_SetCmd(spifi_command_t *cmd);
void SPIFI_SetMemoryCmd(spifi_command_t *cmd);
void SPIFI_Reset(void);


#define QSPI_CMD_READ_IDX           (0)
#define QSPI_PROGRAM_PAGE_IDX       (1)
#define QSPI_GET_STATUS_IDX         (2)
#define QSPI_ERASE_SECTOR_IDX       (3)
#define QSPI_WRITE_ENABLE_IDX       (4)
#define QSPI_WRITE_REGISTER_IDX     (5)
#define QSPI_READ_ID_IDX            (6)


void QSPI_Init(qspi_flash_t * ctx, uint32_t freq);
uint32_t QSPI_GetDeviceID(qspi_flash_t *ctx);
uint32_t QSPI_WriteSector(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf);
uint32_t QSPI_EraseSector(qspi_flash_t *ctx, uint32_t addr);
uint32_t QSPI_SectorTest(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf);
uint32_t QSPI_WaitFinish(qspi_flash_t *ctx);
uint32_t qspi_flash_test(qspi_flash_t *ctx, uint8_t *test_buf, uint32_t addr, uint32_t len);
void QSPI_Read(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf, uint32_t len);
uint32_t QSPI_WritePage(qspi_flash_t *ctx, uint32_t addr, uint8_t *buf);
void QSPI_SetReadMode(qspi_flash_t *ctx);


#ifdef __cplusplus
}
#endif

#endif

