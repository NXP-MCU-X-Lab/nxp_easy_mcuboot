/**
  ******************************************************************************
  * @file    uart.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.2
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_UART_H__
#define __CH_LIB_LPC_UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* UART 模块号 */
#define HW_UART0  (0x00U)
#define HW_UART1  (0x01U)
#define HW_UART2  (0x02U)
#define HW_UART3  (0x03U)
#define HW_UART4  (0x04U)
#define HW_UART5  (0x05U)
#define HW_UART6  (0x06U)
#define HW_UART7  (0x07U)
#define HW_UART8  (0x08U)
#define HW_UART9  (0x09U)

/* UART 中断触发源 */
typedef enum
{
    kUART_IntTx,
    kUART_IntRx,
}UART_Int_t;

/* UART DMA触发源 */
typedef enum
{
    kUART_DMATx,
    kUART_DMARx,
}UART_DMA_t;

/* API */
uint32_t UART_Init(uint32_t MAP, uint32_t baudrate);
uint32_t UART_DeInit(uint32_t MAP);
void UART_SetBaudRate(uint32_t instance, uint32_t baud);
uint32_t UART_GetChar(uint32_t instance, uint8_t *ch);
void UART_PutChar(uint32_t instance, uint8_t ch);
uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val);
uint32_t UART_SetDMAMode(uint32_t instance, UART_DMA_t mode, bool val);
void UART_EnableTxFIFO(uint32_t instance, bool val);
void UART_SetLoopbackMode(uint32_t instance, bool val);
bool UART_GetLoopbackMode(uint32_t instance);



#ifdef __cplusplus
}
#endif


#endif


