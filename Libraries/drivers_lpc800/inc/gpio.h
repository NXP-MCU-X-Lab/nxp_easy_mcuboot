/*
 * gpio.h
 *
 *  Created on: This day
 *      Author: The Creator
 */
#ifndef __CH_LIB_GPIO_H__
#define __CH_LIB_GPIO_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

     
#define HW_GPIO0        (0)
#define HW_GPIO1        (1)
     

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
    kGPIO_Int_RE,
    kGPIO_Int_FE,
    kGPIO_Int_EE,
}GPIO_Int_t;



uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode);
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir);
int GPIO_SetIntMode(uint32_t socket, GPIO_Int_t mode, bool val);
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data);
void GPIO_PinToggle(uint32_t instance, uint8_t pin);
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin);
void GPIO_WritePort(uint32_t instance, uint32_t data);
uint32_t GPIO_ReadPort(uint32_t instance);


#ifdef __cplusplus
}
#endif

#endif


