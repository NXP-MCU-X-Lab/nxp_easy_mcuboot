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
 * @brief  �������ŵ������������
 * @note   ���������������������
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ű�ţ�0-31
 * @param  dir : ����ѡ�����
 *         @arg 1 : ����
 *         @arg 0 : ���
 * @retval None
 */
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir)
{
    (dir == 1) ? (GPIOBases[INS2HW(instance)]->PDDR |= 1 << PIN2HW(instance, pin)):
    (GPIOBases[INS2HW(instance)]->PDDR &= ~(1 << PIN2HW(instance, pin)));
}

 /**
 * @brief  ��ʼ������ָ�����ŵĹ���״̬
 * @note   ��ɳ�ʼ��һ����������
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @param  mode : ���Ų���ģʽ
 *         @arg kGPIO_IFT : ��������
 *         @arg kGPIO_IPD : ��������
 *         @arg kGPIO_IPU : ��������
 *         @arg kGPIO_OOD : ��©���
 *         @arg kGPIO_OPP : �������
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
 * @brief  ָ���������״̬
 * @note   ����ָ���������Ϊ0��1
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @param  data : �������
 *         @arg 0 : ���0
 *         @arg 1 : ���1
 * @retval None
 */
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    (data) ? (GPIOBases[INS2HW(instance)]->PSOR = (1 << PIN2HW(instance, pin))):
    (GPIOBases[INS2HW(instance)]->PCOR = (1 << PIN2HW(instance, pin)));
}

 /**
 * @brief  ��ȡָ�����ŵ�״̬
 * @note   ��
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @retval ��������״̬0��1
 */
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return (GPIOBases[INS2HW(instance)]->PDIR >> PIN2HW(instance, pin)) & 0x01;
}

 /**
 * @brief  ��ת���ŵ�״̬
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�x
 * @param  pin : ���ź��룺0-31
 * @retval None
 */
void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    GPIOBases[INS2HW(instance)]->PTOR = 1 << PIN2HW(instance, pin);
}



//! @}
