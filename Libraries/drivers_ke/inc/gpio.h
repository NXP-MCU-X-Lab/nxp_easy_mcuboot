/**
  ******************************************************************************
  * @file    gpio.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.29
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_GPIO_H__
#define __CH_LIB_GPIO_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

/* GPIO 端口定义 */
#define HW_GPIOA        (0x00U)
#define HW_GPIOB        (0x01U)
#define HW_GPIOC        (0x02U)
#define HW_GPIOD        (0x03U)
#define HW_GPIOE        (0x04U)
#define HW_GPIOF        (0x05U)
#define HW_GPIOG        (0x06U)
#define HW_GPIOH        (0x07U)
#define HW_GPIOI        (0x08U)

#if (__CORTEX_M == 4)
#define PAout(n)   BITBAND_REG(PTA->PDOR, n)
#define PAin(n)    BITBAND_REG(PTA->PDIR, n)
  
#define PBout(n)   BITBAND_REG(PTB->PDOR, n)
#define PBin(n)    BITBAND_REG(PTB->PDIR, n)

#define PCout(n)   BITBAND_REG(PTC->PDOR, n)
#define PCin(n)    BITBAND_REG(PTC->PDIR, n)

#define PDout(n)   BITBAND_REG(PTD->PDOR, n)
#define PDin(n)    BITBAND_REG(PTD->PDIR, n)

#define PEout(n)   BITBAND_REG(PTE->PDOR, n)
#define PEin(n)    BITBAND_REG(PTE->PDIR, n)

#define PFout(n)   BITBAND_REG(PTF->PDOR, n)
#define PFin(n)    BITBAND_REG(PTF->PDIR, n)

#define PGout(n)   BITBAND_REG(PTG->PDOR, n)
#define PGin(n)    BITBAND_REG(PTG->PDIR, n)
#endif

typedef enum
{
    kGPIO_IFT  = 0x00,       /* 悬浮输入 */
    kGPIO_IPD  = 0x01,       /* 下拉输入 */
    kGPIO_IPU  = 0x02,       /* 上拉输入 */
    kGPIO_OPPH = 0x04,       /* 输出高电平*/
    kGPIO_OPPL = 0x08,       /* 输出低电平 */
}GPIO_t;

typedef enum
{
    kGPIO_Int_RE, /* 上升沿触发 */
    kGPIO_Int_FE, /* 下降沿触发 */
    kGPIO_Int_EE, /* 边沿触发 */
}GPIO_Int_t;

//!< API 功能接口
uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode);
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir);
int GPIO_SetIntMode(uint32_t instance, uint32_t pin, GPIO_Int_t mode, bool val);
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data);
void GPIO_PinToggle(uint32_t instance, uint8_t pin);
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif


