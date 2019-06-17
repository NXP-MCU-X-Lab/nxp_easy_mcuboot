/*
 * uart.h
 *
 *  Created on: This day
 *      Author: The Creator
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


/* UART 中断触发源 */
typedef enum
{
    kUART_IntTx,
    kUART_IntRx,
}UART_Int_t;


/* API */
uint32_t UART_Init(uint32_t instance, uint32_t baudrate);
void UART_SetBaudRate(uint32_t instance, uint32_t baud);
uint32_t UART_GetChar(uint32_t instance, uint8_t *ch);
void UART_PutChar(uint32_t instance, uint8_t ch);
uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val);


#ifdef __cplusplus
}
#endif


#endif


