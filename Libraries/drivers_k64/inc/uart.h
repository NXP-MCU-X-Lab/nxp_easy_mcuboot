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

/* UART 模块号 */
#define HW_UART0  (0x00U)
#define HW_UART1  (0x01U)
#define HW_UART2  (0x02U)
#define HW_UART3  (0x03U)
#define HW_UART4  (0x04U)
#define HW_UART5  (0x05U)

/* UART_Init 初始化信息参数 */                      
#define UART1_RX_PE01_TX_PE00   (0x80E1U)
#define UART0_RX_PF17_TX_PF18   (0xA528U)
#define UART3_RX_PE05_TX_PE04   (0x88E3U)
#define UART5_RX_PF19_TX_PF20   (0xA72DU)
#define UART5_RX_PE09_TX_PE08   (0x90E5U)
#define UART2_RX_PE17_TX_PE16   (0xA0E2U)
#define UART4_RX_PE25_TX_PE24   (0xB0E4U)
#define UART0_RX_PA01_TX_PA02   (0x8280U)
#define UART0_RX_PA15_TX_PA14   (0x9CC0U)
#define UART3_RX_PB10_TX_PB11   (0x94CBU)
#define UART0_RX_PB16_TX_PB17   (0xA0C8U)
#define UART1_RX_PC03_TX_PC04   (0x86D1U)
#define UART4_RX_PC14_TX_PC15   (0x9CD4U)
#define UART3_RX_PC16_TX_PC17   (0xA0D3U)
#define UART2_RX_PD02_TX_PD03   (0x84DAU)
#define UART0_RX_PD06_TX_PD07   (0x8CD8U)
#define UART2_RX_PF13_TX_PF14   (0x9B2AU) 
#define UART5_RX_PD08_TX_PD09   (0x90DDU) 
#define UART5_RX_PE08_TX_PE09   (0X90E5U)
#define UART0_RX_PB02_TX_PB01   (0x00008288U)

/* UART 中断触发源 */
typedef enum
{
    kUART_IntTx,
    kUART_IntRx,
    kUART_IntIdleLine,
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
void UART_EnableRxFIFO(uint32_t instance, bool val);

#ifdef __cplusplus
}
#endif


#endif


