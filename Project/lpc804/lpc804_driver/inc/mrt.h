/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
     
void MRT_Init(uint32_t ch, uint32_t us);
void MRT_SetValue(uint8_t ch, uint32_t val);
void MRT_SetTime(uint32_t ch, uint32_t us);
uint32_t MRT_GetTime(uint32_t ch);
uint32_t MRT_GetValue(uint32_t ch);
uint32_t MRT_SetIntMode(uint32_t ch, bool val);





#ifdef __cplusplus
}
#endif

#endif

