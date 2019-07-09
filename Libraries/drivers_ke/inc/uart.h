/**
  ******************************************************************************
  * @file    uart.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.2
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_UART_H__
#define __CH_LIB_UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define HW_UART0  (0x00U)
#define HW_UART1  (0x01U)
#define HW_UART2  (0x02U)

/* UART ÖÐ¶Ï´¥·¢Ô´ */
typedef enum
{
    kUART_IntTx,
    kUART_IntRx,
    kUART_IntIdleLine,
}UART_Int_t;


/* API */
uint32_t UART_Init(uint32_t MAP, uint32_t baudrate);
uint32_t UART_DeInit(uint32_t MAP);
void UART_SetBaudRate(uint32_t instance, uint32_t baud);
uint32_t UART_GetChar(uint32_t instance, uint8_t *ch);
void UART_PutChar(uint32_t instance, uint8_t ch);
uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val);
void UART_SetDebugInstance(uint32_t instance);
uint32_t UART_GetDebugInstance(void);

#ifdef __cplusplus
}
#endif


#endif


