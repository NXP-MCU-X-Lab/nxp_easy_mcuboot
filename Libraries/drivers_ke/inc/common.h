/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CH_LIB_COMMON_H__
#define __CH_LIB_COMMON_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>


/* 固件库版本 */
#define CHLIB_VERSION       (300)


#ifdef MK64F12
#include "MK64F12.h"
#elif MK70F15
#include "MK70F15.h"
#elif MK22F12
#include "MK22F12.h"
#elif MKS22F25612
#include "MKS22F25612.h"
#elif MK80F25615
#include "MK80F25615.h"
#elif MK02F12810
#include "MK02F12810.h"
#elif MKL28Z7
#include "MKL28Z7.h"
#elif MKL27Z4
#include "MKL27Z4.h"
#elif MKL33Z4
#include "MKL33Z4.h"
#elif MK60D10
#include "MK60D10.h"
#elif MK20D5
#include "MK20D5.h"
#elif MKL25Z4
#include "MKL25Z4.h"
#elif MKL27Z4
#include "MKL27Z4.h"
#elif  MKL26Z4
#include "MKL26Z4.h"
#elif  MKL02Z4
#include "MKL02Z4.h"
#elif  MKL03Z4
#include "MKL03Z4.h"
#elif  MKV10Z7
#include "MKV10Z7.h"
#elif  MKW40Z4
#include "MKW40Z4.h"
#elif  MKW41Z4
#include "MKW41Z4.h"
#elif  MK26F18
#include "MK26F18.h"
#elif  MKE18F16
#include "MKE18F16.h"
#elif  MKE02Z4
#include "MKE02Z4.h"
#elif  MKE04Z4
#include "MKE04Z4.h"
#elif  KEA128
#include "SKEAZ1284.h"
#else
#error "No CPU defined!"
#endif

/* NVIC 中断向量组定义 */
#define NVIC_PriorityGroup_0         ((uint32_t)0x7) /*!< 0 bits for pre-emption priority   4 bits for subpriority */                                               
#define NVIC_PriorityGroup_1         ((uint32_t)0x6) /*!< 1 bits for pre-emption priority   3 bits for subpriority */                                                  
#define NVIC_PriorityGroup_2         ((uint32_t)0x5) /*!< 2 bits for pre-emption priority   2 bits for subpriority */                                                   
#define NVIC_PriorityGroup_3         ((uint32_t)0x4) /*!< 3 bits for pre-emption priority   1 bits for subpriority */                                                   
#define NVIC_PriorityGroup_4         ((uint32_t)0x3) /*!< 4 bits for pre-emption priority   0 bits for subpriority */

#if defined(LIB_DEBUG)
#include <stdio.h>
#define LIB_TRACE	printf
#else
#define LIB_TRACE(...)
#endif

/* ALIGN */
#define ALIGN_DOWN(x, a)    ((x) & -(a))
#define ALIGN_UP(x, a)      (-(-(x) & -(a)))

#ifndef ALIGN
/* Compiler Related Definitions */
#ifdef __CC_ARM                         /* ARM Compiler */
    #define ALIGN(n)                    __attribute__((aligned(n)))
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #define PRAGMA(x)                   _Pragma(#x)
    #define ALIGN(n)                    PRAGMA(data_alignment=n)
#elif defined (__GNUC__)                /* GNU GCC Compiler */
    #define ALIGN(n)                    __attribute__((aligned(n)))
#endif /* Compiler Related Definitions */
#endif

#define MAKE_VERSION(major, minor, bugfix) (((major) << 16) | ((minor) << 8) | (bugfix))

#ifndef MIN
#define MIN(a, b)       ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)       ((a) > (b) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a)         (((a) < 0) ? (-(a)) : (a))
#endif

//! @brief Computes the number of elements in an array.
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef BSWAP_16
#define BSWAP_16(x)     (uint16_t)((((x) & 0xFF00) >> 0x8) | (((x) & 0xFF) << 0x8))
#endif

#ifndef BSWAP_32
#define BSWAP_32(val)	(uint32_t)((BSWAP_16((uint32_t)(val) & (uint32_t)0xFFFF) << 0x10) |  \
                                   (BSWAP_16((uint32_t)((val) >> 0x10))))
#endif

/* CHLib internal use */
#define REG_SET(t, x)               (*((uint32_t*) t[x].addr) |= t[x].mask)
#define REG_CLR(t, x)               (*((uint32_t*) t[x].addr) &= ~t[x].mask)
#define REG_GET(t, x)               ((*(uint32_t*) t[x].addr & t[x].mask)>>t[x].shift)
#define PIN_SET_MUX                 do {int i; for(i=0; i<pq->pin_count; i++) { LIB_TRACE("instance:%d pin:%c%02d mux:%d\r\n", pq->ip, pq->io + 'A', pq->pin_start+i, pq->mux); SetPinMux(pq->io, pq->pin_start + i, pq->mux);}}while(0)
#define PIN_SET_PULL(x)             do {int i; for(i=0; i<pq->pin_count; i++) { LIB_TRACE("instance:%d pin:%c%02d set to:%d\r\n", pq->ip, pq->io + 'A', pq->pin_start+i, x); SetPinPull(pq->io, pq->pin_start + i, x);}}while(0)
#define US_TO_COUNT(us, Hz)    (uint32_t)((uint32_t)us * (Hz / 1000000U))
#define COUNT_TO_US(count, Hz) (uint32_t)((uint32_t)(count * 1000000U) / Hz)
#define MS_TO_COUNT(ms, Hz)    (uint32_t)((uint32_t)ms * (Hz / 1000U))
#define COUNT_TO_MS(count, Hz) (uint32_t)((uint32_t)(count * 1000U) / Hz)

/* legecy support of SCG moudle */
#if !defined(PCC_CLKCFG_PCS_MASK)
#define PCC_CLKCFG_PCS_MASK                      (0x7000000U)
#define PCC_CLKCFG_PCS_SHIFT                     (24U)
#define PCC_CLKCFG_PCS(x)                        (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_PCS_SHIFT)) & PCC_CLKCFG_PCS_MASK)
#endif

#if !defined(PCC_CLKCFG_CGC_MASK)
#define PCC_CLKCFG_CGC_MASK                      (0x40000000U)
#define PCC_CLKCFG_CGC_SHIFT                     (30U)
#define PCC_CLKCFG_CGC(x)                        (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_CGC_SHIFT)) & PCC_CLKCFG_CGC_MASK)
#endif

    /* 系统时钟定义 */
typedef enum
{
    kCoreClock,
    kBusClock,
    kMCGOutClock,
    kOSCClock,
}Clock_t; 

/* 功耗模式选择 */
enum
{
    kRUN =      (0x00),  
    kHSRUN =    (0x01),
    kVLPR =     (0x02),
    kWAIT =     (0x10),
    kSTOP =     (0x20),
    kVLPS =     (0x22),
    kLLS3 =     (0x24),
};

/* 固件库返回值代码 */
enum
{
    CH_OK,
    CH_ERR,
    CH_TIMEOUT,
    CH_OVERFLOW,
    CH_IO_ERR,
    CH_BUSY,
};

/* 外设 MAP 初始化结构 */
typedef struct
{
    uint32_t ip:3;
    uint32_t io:3;
    uint32_t mux:3;
    uint32_t pin_start:5;
    uint32_t pin_count:5;
    uint32_t chl:5;
    uint32_t reserved:8;
}map_t;

typedef struct
{
    void *      addr;
    uint32_t    mask;
    uint32_t    shift;
}Reg_t;

/* Backward Compatibility */
typedef uint8_t         u8;
typedef uint16_t        u16;
typedef uint32_t        u32;


/* API */
void SystemSoftReset(void);
void SetPinMux(uint32_t instance, uint32_t pin, uint32_t mux);
void SetPinPull(uint32_t instance, uint32_t pin, uint32_t val);
void SetPinOpenDrain(uint32_t instance, uint32_t pin, bool val);
void SetPinFilter(uint32_t instance, uint32_t pin, bool val);
void SetPinHighDrive(uint32_t instance, uint32_t pin, bool val);
void DelayInit(void);
void DelayMs(uint32_t ms);
void DelayUs(uint32_t us);
void SysTick_SetIntMode(bool val);
void SysTick_SetTime(uint32_t us);
void SysTick_Init(void);
uint32_t GetClock(Clock_t clock);
uint32_t GetUID(void);
void SetConsole(int (*putc)(uint8_t ch), int (*getc)(void));
uint32_t GetResetCause(void);
void SetPowerMode(uint32_t Mode, bool enSleepOnExit);
void JumpToImage(uint32_t addr);

/* software test function */
uint32_t RAMTest(uint32_t addr, uint32_t length);
void ShowMemory(uint32_t addr, uint32_t length);

#ifdef __cplusplus
}
#endif


#endif
