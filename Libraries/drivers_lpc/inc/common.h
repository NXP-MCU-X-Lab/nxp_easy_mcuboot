/**
  ******************************************************************************
  * @file    common.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2015.6.21
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_COMMON_H__
#define __CH_LIB_LPC_COMMON_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>



#define CHLIB_VERSION       (300)


#ifdef  LPC82x
#define LPC8XX
#include "LPC82x.h"
#elif   LPC54114
#include "LPC54114_cm4.h"
#elif   LPC51U68
#include "LPC51U68.h"
#elif   LPC54608
#include "LPC54608.h"
#elif   QN9080
#include "QN908XB.h"
#elif   LPC54018
#include "LPC54018.h"
#else
#error "No CPU defined!"
#endif

/* NVIC 中断向量组定义 */
#define NVIC_PriorityGroup_0         ((uint32_t)0x7) /*!< 0 bits for pre-emption priority   4 bits for subpriority */                                               
#define NVIC_PriorityGroup_1         ((uint32_t)0x6) /*!< 1 bits for pre-emption priority   3 bits for subpriority */                                                  
#define NVIC_PriorityGroup_2         ((uint32_t)0x5) /*!< 2 bits for pre-emption priority   2 bits for subpriority */                                                   
#define NVIC_PriorityGroup_3         ((uint32_t)0x4) /*!< 3 bits for pre-emption priority   1 bits for subpriority */                                                   
#define NVIC_PriorityGroup_4         ((uint32_t)0x3) /*!< 4 bits for pre-emption priority   0 bits for subpriority */


#if !defined(SYSCON)
#define SYSCON      LPC_SYSCON

#endif

#if !defined(GPIO)
#define GPIO        LPC_GPIO
#endif

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

#ifndef BITBAND_REG
#define BITBAND_REG(Reg,Bit) (*((uint32_t volatile*)(0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))))
#endif

#if (__CORTEX_M == 4)

#define P0out(n)   BITBAND_REG(GPIO->PIN[0], n)
#define P0in(n)    BITBAND_REG(GPIO->PIN[0], n)
#define P1out(n)   BITBAND_REG(GPIO->PIN[1], n)
#define P1in(n)    BITBAND_REG(GPIO->PIN[1], n)
#define P2out(n)   BITBAND_REG(GPIO->PIN[2], n)
#define P2in(n)    BITBAND_REG(GPIO->PIN[2], n)
#define P3out(n)   BITBAND_REG(GPIO->PIN[3], n)
#define P3in(n)    BITBAND_REG(GPIO->PIN[3], n)

#endif

/* CHLib internal use */
#define US_TO_COUNT(us, Hz)    (uint32_t)((uint32_t)us * (Hz / 1000000U))
#define COUNT_TO_US(count, Hz) (uint32_t)((uint32_t)(count * 1000000U) / Hz)
#define MS_TO_COUNT(ms, Hz)    (uint32_t)((uint32_t)ms * (Hz / 1000U))
#define COUNT_TO_MS(count, Hz) (uint32_t)((uint32_t)(count * 1000U) / Hz)



    /* 系统时钟定义 */
typedef enum
{
    kCoreClock,
    kFROHfClock,
    kFROLfClock,
    kExtOSCClock,
    kUSBPLLClock,
    kEMCClock,
}Clock_t; 


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
void SetPinAnalogMode(uint32_t instance, uint32_t pin, bool val);
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
void SetFlexCommClk(uint32_t instance, uint32_t val);
void JumpToImage(uint32_t addr);

/* software test function */
uint32_t RAMTest(uint32_t address, uint32_t size);
void ShowMemory(uint32_t addr, uint32_t size);

#ifdef __cplusplus
}
#endif


#endif
