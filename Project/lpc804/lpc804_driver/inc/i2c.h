/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CH_LIB_LPC_I2C_H__
#define __CH_LIB_LPC_I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif

     
#include <stdint.h>
#include <stdbool.h>
     
typedef enum
{
    kI2C_SlvPending,
    kI2C_SlvDesel,
}I2C_Int_t;

#define HW_I2C0     (0)
#define HW_I2C1     (1)
#define HW_I2C2     (2)
     
void I2C_SetBaudRate(uint32_t instance, uint32_t baud);
uint32_t I2C_Init(uint32_t MAP, uint32_t baudrate);
uint32_t I2C_BurstRead(uint32_t instance, uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t* buf, uint32_t len);
uint32_t I2C_BurstWrite(uint32_t instance ,uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t *buf, uint32_t len);
uint32_t I2C_ReadReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t* buf);
uint32_t I2C_WriteReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t buf);
uint32_t SCCB_ReadReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t* buf);
uint32_t SCCB_WriteReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t buf);
uint32_t I2C_Probe(uint32_t instance, uint8_t addr);
void I2C_Scan(uint32_t instance);
uint32_t I2C_SetIntMode(uint32_t instance, I2C_Int_t mode, bool val);
uint32_t I2C_SetSlvAddr(uint32_t instance, uint32_t slot, uint32_t addr);

#endif
