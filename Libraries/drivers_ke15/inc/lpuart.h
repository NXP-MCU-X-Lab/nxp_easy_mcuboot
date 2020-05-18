/**
  ******************************************************************************
  * @file    lpuart.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.13
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPUART_H__
#define __CH_LIB_LPUART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    kLPUART_IntTx,
    kLPUART_IntRx,
    kLPUART_IntIdleLine,
}LPUART_Int_t;
     
/* LPUART init macro */                      
#define LPUART1_RX_PE01_TX_PE00   (0x000080E1U)
#define LPUART2_RX_PE17_TX_PE16   (0x0000A0E2U)
#define LPUART0_RX_PE21_TX_PE20   (0x0000A920U)
#define LPUART2_RX_PE23_TX_PE22   (0x0000AD22U)
#define LPUART0_RX_PA01_TX_PA02   (0x00008280U)
#define LPUART0_RX_PA15_TX_PA14   (0x00009CC0U)
#define LPUART1_RX_PA18_TX_PA19   (0x0000A4C1U)
#define LPUART0_RX_PB16_TX_PB17   (0x0000A0C8U)
#define LPUART2_RX_PD02_TX_PD03   (0x000084DAU)
#define LPUART2_RX_PD04_TX_PD05   (0x000088DAU)
#define LPUART0_RX_PD06_TX_PD07   (0x00008CD8U)
#define LPUART0_RX_PA04_TX_PA03   (0x00008700U)


/*!< LPUART instance */
#define HW_LPUART0  (0x00U)
#define HW_LPUART1  (0x01U)
#define HW_LPUART2  (0x02U)    
     
//!< API 功能接口
uint32_t LPUART_Init(uint32_t MAP, uint32_t baudrate);
void LPUART_PutChar(uint32_t instance, uint8_t ch);
uint32_t LPUART_GetChar(uint32_t instance, uint8_t *ch);
uint32_t LPUART_SetIntMode(uint32_t instance, LPUART_Int_t mode, bool val);
uint32_t LPUART_DeInit(uint32_t instance);
void LPUART_SetBaudRate(uint32_t instance, uint32_t baud);


#ifdef __cplusplus
}
#endif

#endif

