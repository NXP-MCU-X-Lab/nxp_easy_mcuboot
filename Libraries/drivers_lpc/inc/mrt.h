/**
  ******************************************************************************
  * @file    mrt.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.05.31
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_MRT_H__
#define __CH_LIB_MRT_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif


//!< hardware instances
#define HW_MRT_CH0   (0x00U)
#define HW_MRT_CH1   (0x01U)
#define HW_MRT_CH2   (0x02U)
     
void MRT_Init(uint32_t chl, uint32_t us);
void MRT_SetValue(uint8_t chl, uint32_t val);
void MRT_SetTime(uint32_t chl, uint32_t us);
uint32_t MRT_GetTime(uint32_t chl);
uint32_t MRT_GetValue(uint32_t chl);
uint32_t MRT_SetIntMode(uint32_t chl, bool val);





#ifdef __cplusplus
}
#endif

#endif

