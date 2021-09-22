/**
  ******************************************************************************
  * @file    i2c.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.2
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_I2C_H__
#define __CH_LIB_LPC_I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif

     
#include <stdint.h>
#include <stdbool.h>
     
#define HW_I2C0     (0)
#define HW_I2C1     (1)
#define HW_I2C2     (2)
#define HW_I2C3     (3)
#define HW_I2C4     (4)
#define HW_I2C5     (5)
#define HW_I2C6     (6)
#define HW_I2C7     (7)
#define HW_I2C8     (8)


typedef enum
{
    kI2C_SlavePending,
    kI2C_SlaveDeselect,
}I2C_Int_t;


     
void I2C_SetBaudRate(uint32_t instance, uint32_t baud);
uint32_t I2C_Init(uint32_t MAP, uint32_t baudrate);
uint32_t I2C_BurstRead(uint32_t instance, uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t* buf, uint32_t len);
uint32_t I2C_BurstWrite(uint32_t instance ,uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t *buf, uint32_t len);
uint32_t I2C_ReadReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t* buf);
uint32_t I2C_WriteReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t buf);
uint32_t SCCB_ReadReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t* buf);
uint32_t SCCB_WriteReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t buf);
void I2C_Scan(uint32_t instance);
uint32_t I2C_SetSalveAddr(uint32_t instance, uint8_t slot, uint8_t addr);
uint32_t I2C_Write(uint32_t instance, uint8_t addr, uint8_t *buf, uint32_t len);
uint32_t I2C_Read(uint32_t instance, uint8_t addr, uint8_t* buf, uint32_t len);
uint32_t I2C_SetIntMode(uint32_t instance, I2C_Int_t val);



#endif
