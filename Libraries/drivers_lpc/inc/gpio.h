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
     
#define HW_GPIO0        (HW_GPIOA)
#define HW_GPIO1        (HW_GPIOB)
#define HW_GPIO2        (HW_GPIOC)
#define HW_GPIO3        (HW_GPIOD)
#define HW_GPIO4        (HW_GPIOE)

#define PIN_SOCKET0     (0)
#define PIN_SOCKET1     (1)
#define PIN_SOCKET2     (2)
#define PIN_SOCKET3     (3)
#define PIN_SOCKET4     (4)
#define PIN_SOCKET5     (5)
#define PIN_SOCKET6     (6)
#define PIN_SOCKET7     (7)

#define PME_BIT0        (0)
#define PME_BIT1        (1)
#define PME_BIT2        (2)
#define PME_BIT3        (3)
#define PME_BIT4        (4)
#define PME_BIT5        (5)
#define PME_BIT6        (6)
#define PME_BIT7        (7)

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

/*! @brief PINT Pattern Match configuration type */
typedef enum
{
    kPINT_PatternMatchAlways = 0U,          /*!< Always Contributes to product term match */
    kPINT_PatternMatchStickyRise = 1U,      /*!< Sticky Rising edge */
    kPINT_PatternMatchStickyFall = 2U,      /*!< Sticky Falling edge */
    kPINT_PatternMatchStickyBothEdges = 3U, /*!< Sticky Rising or Falling edge */
    kPINT_PatternMatchHigh = 4U,            /*!< High level */
    kPINT_PatternMatchLow = 5U,             /*!< Low level */
    kPINT_PatternMatchNever = 6U,           /*!< Never contributes to product term match */
    kPINT_PatternMatchBothEdges = 7U,       /*!< Either rising or falling edge */
} GPIO_PME_Mode_t;


//!< API 功能接口
uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode);
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir);
int GPIO_AddIntToSocket(uint32_t instance, uint32_t pin, uint8_t socket);
int GPIO_SetIntMode(uint32_t socket, GPIO_Int_t mode, bool val);
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data);
void GPIO_PinToggle(uint32_t instance, uint8_t pin);
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin);
void GPIO_WritePort(uint32_t instance, uint32_t data);
uint32_t GPIO_ReadPort(uint32_t instance);

void GPIO_SetPMEMode(bool val);
void GPIO_AddPMEBitToSocket(uint32_t pme_bit, GPIO_PME_Mode_t mode, bool isep, uint32_t socket);

#ifdef __cplusplus
}
#endif

#endif


