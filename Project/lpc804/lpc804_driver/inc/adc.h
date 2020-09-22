/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CH_LIB_LPC_ADC_H__
#define __CH_LIB_LPC_ADC_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>
     
#define HW_ADC_SEQA        (0)
#define HW_ADC_SEQB        (1)
     
#define HW_ADC_CH0      (0)
#define HW_ADC_CH1      (1)
#define HW_ADC_CH2      (2)
#define HW_ADC_CH3      (3)
#define HW_ADC_CH4      (4)
#define HW_ADC_CH5      (5)
#define HW_ADC_CH6      (6)
#define HW_ADC_CH7      (7)
#define HW_ADC_CH8      (8)
#define HW_ADC_CH9      (9)
#define HW_ADC_CH10     (10)
#define HW_ADC_CH11     (11)
     
     
typedef enum
{
    kADC_SEQA,
    kADC_SEQB,
    kADC_OVR,
    kADC_THR0,
    kADC_THR1,
    kADC_THR2,
    kADC_THR3,
    kADC_THR4,
    kADC_THR5,
    kADC_THR6,
    kADC_THR7,
    kADC_THR8,
    kADC_THR9,
    kADC_THR10,
    kADC_THR11,
}ADC_Int_t;


     
     
     
     
     
void ADC_Init(uint32_t sample_rate);
void ADC_ConfigSeq(uint32_t seq, uint32_t chl_bm);
void ADC_SoftSingleTrigger(uint32_t seq);
uint16_t ADC_GetResult(uint32_t ch);
uint32_t ADC_GetResultEx(uint32_t ch);
void ADC_SetContinueTrigger(uint32_t seq, bool val);
void ADC_SetIntMode(ADC_Int_t mode, bool val);
     
#endif
