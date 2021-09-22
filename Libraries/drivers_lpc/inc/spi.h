/**
  ******************************************************************************
  * @file    spi.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.2
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_SPI_H__
#define __CH_LIB_LPC_SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define HW_SPI0     (0)
#define HW_SPI1     (1)
#define HW_SPI2     (2)
#define HW_SPI3     (3)
#define HW_SPI4     (4)
#define HW_SPI5     (5)
#define HW_SPI6     (6)
     
#include <stdint.h>
#include <stdbool.h>
     

uint32_t SPI_Init(uint32_t MAP, uint32_t baudrate);     
uint32_t SPI_ReadWriteEx(uint32_t instance, uint32_t data, uint16_t cs, uint32_t cs_state);
uint32_t SPI_ReadWrite(uint32_t instance, uint32_t data);
uint32_t SPI_WriteFIFO(uint32_t instance, uint8_t *buf, uint32_t len);
uint32_t SPI_ReadFIFO(uint32_t instance, uint8_t *buf, uint32_t len);
uint32_t SPI_ReadWriteFIFO(uint32_t instance, uint8_t *in_buf, uint8_t *out_buf, uint32_t len);

#endif
