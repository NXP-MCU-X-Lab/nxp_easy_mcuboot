/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "gpio.h"

/* software GPIOA/B/C/D/E/F/G/H  to hardware GPIOA, GPIOB */
#define INS2HW(instance)  (instance / 3)
#define PIN2HW(instance, pin)       ((pin + ((8*instance) % 32)))

static GPIO_Type * const GPIOBases[] = GPIO_BASES;



/**
 * @brief  设置引脚的输入输出方向
 * @note   控制引脚是输出还是输入
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚编号：0-31
 * @param  dir : 方向选择参数
 *         @arg 1 : 输入
 *         @arg 0 : 输出
 * @retval None
 */
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir)
{
    (dir == 1) ? (GPIOBases[INS2HW(instance)]->PDDR |= 1 << PIN2HW(instance, pin)):
    (GPIOBases[INS2HW(instance)]->PDDR &= ~(1 << PIN2HW(instance, pin)));
}

 /**
 * @brief  初始化配置指定引脚的工作状态
 * @note   完成初始化一个引脚配置
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  mode : 引脚操作模式
 *         @arg kGPIO_IFT : 悬浮输入
 *         @arg kGPIO_IPD : 下拉输入
 *         @arg kGPIO_IPU : 上拉输入
 *         @arg kGPIO_OOD : 开漏输出
 *         @arg kGPIO_OPP : 推挽输出
 * @retval None
 */
uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode)
{
    switch(mode)
    {
        case kGPIO_IFT:
            SetPinPull(instance, pin, 0xFF);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_IPD:
            SetPinPull(instance, pin, 0);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_IPU:
            SetPinPull(instance, pin, 1);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_OPPH:
        case kGPIO_OPPL:
            SetPinPull(instance, pin, 0xFF);
            (mode == kGPIO_OPPH)? GPIO_PinWrite(instance, pin, 1):GPIO_PinWrite(instance, pin, 0);
            GPIO_SetPinDir(instance, pin, 1);
            break;
        default:
            break;					
    }
    /* config pinMux */
    SetPinMux(instance, pin, 1);
    return instance;
}


 /**
 * @brief  指定引脚输出状态
 * @note   设置指定引脚输出为0或1
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  data : 引脚输出
 *         @arg 0 : 输出0
 *         @arg 1 : 输出1
 * @retval None
 */
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    (data) ? (GPIOBases[INS2HW(instance)]->PSOR = (1 << PIN2HW(instance, pin))):
    (GPIOBases[INS2HW(instance)]->PCOR = (1 << PIN2HW(instance, pin)));
}

 /**
 * @brief  读取指定引脚的状态
 * @note   无
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @retval 返回引脚状态0或1
 */
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return (GPIOBases[INS2HW(instance)]->PDIR >> PIN2HW(instance, pin)) & 0x01;
}

 /**
 * @brief  翻转引脚的状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号x
 * @param  pin : 引脚号码：0-31
 * @retval None
 */
void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    GPIOBases[INS2HW(instance)]->PTOR = 1 << PIN2HW(instance, pin);
}



//! @}
