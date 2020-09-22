/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "common.h"
#include "gpio.h"


#if defined(GPIO_BASE_PTRS)
#define GPIO_BASES  GPIO_BASE_PTRS
#endif

#if defined(PORT_BASE_PTRS)
#define PORT_BASES  PORT_BASE_PTRS
#endif

#ifndef GPIO_BASES
    #if defined(PTA)
    #define GPIO_BASES {PTA, PTB, PTC, PTD, PTE};
    #elif defined(GPIOA)
    #define GPIO_BASES {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};
    #endif
#endif

#ifndef PORT_BASES
#define PORT_BASES {PORTA, PORTB, PORTC, PORTD, PORTE};
#endif

static GPIO_Type * const GPIOBases[] = GPIO_BASES;
static PORT_Type * const PORTBases[] = PORT_BASES;
#if defined(MKL27Z4)
static const IRQn_Type GPIO_IrqTbl[] = 
{
    (IRQn_Type)PORTA_IRQn,
    (IRQn_Type)0,
    (IRQn_Type)PORTCD_IRQn,
    (IRQn_Type)PORTCD_IRQn,
};
#else
//static const IRQn_Type GPIO_IrqTbl[] = 
//{
//    (IRQn_Type)(PORTA_IRQn + 0),
//    (IRQn_Type)(PORTA_IRQn + 1),
//    (IRQn_Type)(PORTA_IRQn + 2),
//    (IRQn_Type)(PORTA_IRQn + 3),
//    (IRQn_Type)(PORTA_IRQn + 4),
//};
#endif

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
    (dir == 1) ? (GPIOBases[instance]->PDDR |= (1 << pin)):
    (GPIOBases[instance]->PDDR &= ~(1 << pin));
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
//        #if defined(PORT_PCR_DSE_MASK)
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_DSE_MASK;
//        #endif
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
    (data) ? (GPIOBases[instance]->PSOR |= (1 << pin)):
    (GPIOBases[instance]->PCOR |= (1 << pin));
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
    return ((GPIOBases[instance]->PDIR >> pin) & 0x01);
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
    GPIOBases[instance]->PTOR |= (1 << pin);
}

 /**
 * @brief  读取引脚端口的状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @retval 返回一个32位端口的数据
 */
uint32_t GPIO_ReadPort(uint32_t instance)
{
    return (GPIOBases[instance]->PDIR);
}

 /**
 * @brief  修改端口的输出状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  data : 32位的数据
 * @retval None
 */
void GPIO_WritePort(uint32_t instance, uint32_t data)
{
    GPIOBases[instance]->PDOR = data;
}


/**
 * @brief  设置引脚中断
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  mode : 中断模式
 *         @arg kGPIO_DMA_RE : 上升沿触发
 *         @arg kGPIO_DMA_FE ：下降沿触发
 *         @arg kGPIO_DMA_EE ：边沿触发
 * @param  val :
 *         @arg true ：开启中断
 *         @arg false ：关闭中断
 * @retval CH_OK：成功；其它：失败
 */
//int GPIO_SetIntMode(uint32_t instance, uint32_t pin, GPIO_Int_t mode, bool val)
//{
//    PORTBases[instance]->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
//    if(!val)
//    {
//        
//        return CH_OK;
//    }
//    NVIC_EnableIRQ(GPIO_IrqTbl[instance]);
//    
//    switch(mode)
//    {
//        case kGPIO_Int_RE:
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(9);
//            break;
//        case kGPIO_Int_FE:
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(10);
//            break;
//        case kGPIO_Int_EE:
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(11);
//            break;
//    }
//    return CH_OK;
//}

/**
 * @brief  设置引脚DMA触发
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  mode : 中断模式
 *         @arg kGPIO_Int_RE : 上升沿触发
 *         @arg kGPIO_Int_FE ：下降沿触发
 *         @arg kGPIO_Int_EE ：边沿触发
 * @param  val :
 *         @arg true ：开启中断
 *         @arg false ：关闭中断
 * @retval CH_OK：成功；其它：失败
 */
int GPIO_SetDMAMode(uint32_t instance, uint32_t pin, GPIO_DMA_t mode, bool val)
{
    if(!val)
    {
        PORTBases[instance]->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
        return CH_OK;
    }
    PORTBases[instance]->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
    switch(mode)
    {
        case kGPIO_DMA_RE:
            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(1);
            break;
        case kGPIO_DMA_FE:
            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(2);
            break;
        case kGPIO_DMA_EE:
            PORTBases[instance]->PCR[pin] |= PORT_PCR_IRQC(3);
            break;
    }
    return CH_OK;
}

void PORT_IRQHandler(uint32_t instance)
{
    PORTBases[instance]->ISFR = 0xFFFFFFFF;
}
//! @}
