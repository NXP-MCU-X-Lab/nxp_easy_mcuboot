/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>
#include "common.h"

#ifndef DEFAULT_SYSTEM_CLOCK
#define DEFAULT_SYSTEM_CLOCK    (48000000)
#endif

/* PORT模块时钟开关位置定义 */
#if defined(SIM_SCGC5_PORTA_MASK)
static const Reg_t PORTClkGate[] =
{
    {(void*)&(SIM->SCGC5), SIM_SCGC5_PORTA_MASK},
    {(void*)&(SIM->SCGC5), SIM_SCGC5_PORTB_MASK},
#ifdef SIM_SCGC5_PORTC_MASK
    {(void*)&(SIM->SCGC5), SIM_SCGC5_PORTC_MASK},
#endif
#ifdef SIM_SCGC5_PORTD_MASK
    {(void*)&(SIM->SCGC5), SIM_SCGC5_PORTD_MASK},
#endif
#ifdef SIM_SCGC5_PORTE_MASK 
    {(void*)&(SIM->SCGC5), SIM_SCGC5_PORTE_MASK},
#endif
};
#elif defined(PCC0_PCC_PORTA_REG)
static const Reg_t PORTClkGate[] =
{
    {(void*)&(PCC0->PCC_PORTA), PCC0_PCC_PORTA_CGC_MASK},
    {(void*)&(PCC0->PCC_PORTB), PCC0_PCC_PORTB_CGC_MASK},
    {(void*)&(PCC0->PCC_PORTC), PCC0_PCC_PORTC_CGC_MASK},
    {(void*)&(PCC0->PCC_PORTD), PCC0_PCC_PORTD_CGC_MASK},
    {(void*)&(PCC0->PCC_PORTE), PCC0_PCC_PORTE_CGC_MASK},
};
#elif defined(PCC_PORTA_INDEX)
static const Reg_t PORTClkGate[] =
{
    {(void*)&(PCC->CLKCFG[PCC_PORTA_INDEX]), PCC_CLKCFG_CGC_MASK},
    {(void*)&(PCC->CLKCFG[PCC_PORTB_INDEX]), PCC_CLKCFG_CGC_MASK},
    {(void*)&(PCC->CLKCFG[PCC_PORTC_INDEX]), PCC_CLKCFG_CGC_MASK},
    {(void*)&(PCC->CLKCFG[PCC_PORTD_INDEX]), PCC_CLKCFG_CGC_MASK},
    {(void*)&(PCC->CLKCFG[PCC_PORTE_INDEX]), PCC_CLKCFG_CGC_MASK},
};
#endif

#if defined(GPIO_BASE_PTRS)
#define GPIO_BASES  GPIO_BASE_PTRS
#endif

#if defined(PORT_BASE_PTRS)
#define PORT_BASES  PORT_BASE_PTRS
#endif

#ifndef PORT_BASES
#define PORT_BASES {PORTA, PORTB, PORTC, PORTD, PORTE};
#endif

/* 控制台接口 */
typedef struct
{
    int (*putc)(uint8_t ch);
    int (*getc)(void);
}Console_t;

/* 固件库全局变量 */
//static PORT_Type * const PORTBases[] = PORT_BASES;
static uint32_t fac_us = 0;             /* usDelay Mut */
static uint32_t fac_ms = 0;             /* msDelay Mut */
volatile static Console_t Console;      /* 调试器  */
bool _is_fitst_init = true;

/* debug 宏定义 */
#ifdef LIB_DEBUG
#if (defined(__CC_ARM)) || (defined(__ICCARM__))
void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
    printf("ASSERT ERROR \" %s \": file \"%s\" Line \"%d\" \n", failedExpr, file, line);
    for (;;)
    {
    }
}
#elif(defined(__GNUC__))
void __assert_func(const char *file, int line, const char *func, const char *failedExpr)
{
    printf("ASSERT ERROR \" %s \": file \"%s\" Line \"%d\" function name \"%s\" \n", failedExpr, file, line, func);
    for (;;)
    {
    }
}
#endif /* (defined(__CC_ARM)) ||  (defined (__ICCARM__)) */
#endif /* LIB_DEBUG */


 /**
 * @brief  读取系统时钟
 * @note   None
 * @param  clock: 系统时钟
 *         @arg kCoreClock 内核时钟
 *         @arg kBusClock  总线时钟(通常为内核时钟的一半)
 * @retval 系统时钟的频率 Hz
 */
uint32_t GetClock(Clock_t clock)
{
    //SystemCoreClockUpdate();
    uint32_t val;
    val = SystemCoreClock;
    
    switch(clock)
    {
        case kCoreClock:
            val = SystemCoreClock;
            break;
        case kBusClock:
            val = SystemCoreClock;
            break;
        case kOSCClock:
            val = 10*1000*1000;
            break;
        case kMCGOutClock:
            val = SystemCoreClock;
            break;
    }
    return val;
}

 /**
 * @brief  设置引脚复用
 * @note   None
 * @param  instance: GPIO模块号
 *         @arg HW_GPIOx ：GPIOA-GPIOE
 * @param  pin: 0-31
 * @param  mux: 复用选项 0-7
 * @retval None
 */
void SetPinMux(uint32_t instance, uint32_t pin, uint32_t mux)
{
//    REG_SET(PORTClkGate, instance);
//    PORTBases[instance]->PCR[pin] &= ~PORT_PCR_MUX_MASK;
//    PORTBases[instance]->PCR[pin] |= PORT_PCR_MUX(mux);
}


 /**
 * @brief  设置引脚上下拉配置
 * @param  instance: GPIO模块号
 *         @arg HW_GPIOx ：GPIOA-GPIOE
 * @param  pin: 0-31
 * @param  val: 上下拉配置
 *         @arg 0 下拉
 *         @arg 1 上拉
 *         @arg 其他 浮空
 * @retval None
 */
void SetPinPull(uint32_t instance, uint32_t pin, uint32_t val)
{
//    REG_SET(PORTClkGate, instance);
//    switch(val)
//    {
//        case 0:
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_PE_MASK;
//            PORTBases[instance]->PCR[pin] &= ~PORT_PCR_PS_MASK;
//            break;
//        case 1:
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_PE_MASK;
//            PORTBases[instance]->PCR[pin] |= PORT_PCR_PS_MASK;
//            break;
//        default:
//            PORTBases[instance]->PCR[pin] &= ~PORT_PCR_PE_MASK;
//            break;
//    }
}

void SysTick_Init(void)
{
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
    fac_us = GetClock(kCoreClock);
    fac_us /= 1000000;
    fac_ms = fac_us * 1000;
}

 /**
 * @brief  延时初始化
 * @note   调用DelayMs和DealyUs前需要调用此函数
 * @retval None
 */
#pragma weak DelayInit
void DelayInit(void)
{
    SysTick_Init();
}

/**
 * @brief MS级延时
 * @note  None
 * @param ms : 延时数，单位MS
 * @retval None
 */
#pragma weak DelayMs
void DelayMs(uint32_t ms)
{
    volatile uint32_t temp;
    uint32_t i;
    SysTick->LOAD = fac_ms;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->VAL = 0;
    for(i=0; i<ms; i++)
	{
		do
		{
			temp = SysTick->CTRL;
		}
        while((temp & SysTick_CTRL_ENABLE_Msk) && !(temp & SysTick_CTRL_COUNTFLAG_Msk));
	}
}

/**
 * @brief  设置SysTick定时器时间
 * @note   None
 * @param  us : 定时器时间，单位US
 * @retval None
 */
void SysTick_SetTime(uint32_t us)
{
    ((us*fac_us) > 0xFFFFFF)?(SysTick->LOAD = 0xFFFFFF):(SysTick->LOAD = us*fac_us);
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

/**
 * @brief  US秒级延时
 * @note   None
 * @param  us : 延时数，单位US
 * @retval None
 */
#pragma weak DelayUs
void inline DelayUs(uint32_t us)
{
    volatile uint32_t temp;
    SysTick_SetTime(us);
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & SysTick_CTRL_ENABLE_Msk) && !(temp & SysTick_CTRL_COUNTFLAG_Msk));
}

/**
 * @brief  开启或关闭SysTick中断
 * @note   None
 * @param  val: 
 *         @arg true : 关闭中断
 *         @arg false : 开启中断
 * @retval None
 */
void SysTick_SetIntMode(bool val)
{
    (true == val)?(SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk):(SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk);
}


void SetConsole(int (*putc)(uint8_t ch), int (*getc)(void))
{
    Console.getc = getc;
    Console.putc = putc;
}

#include <stdio.h>
#ifdef __CC_ARM // MDK Support
struct __FILE 
{ 
	int handle;
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;
FILE __stdin;
int fputc(int ch,FILE *f)
{
    if(Console.putc)
    {
        Console.putc((uint8_t)ch);
    }
	return ch;
}

int fgetc(FILE *f)
{
    if(Console.getc)
    {
        return (Console.getc() & 0xFF);
    }
    return 0;
}

#elif __ICCARM__ /* IAR support */
#include <yfuns.h>

size_t __write(int handle, const unsigned char * buffer, size_t size)
{
    size_t nChars = 0;
    if (buffer == 0)
    {
        return 0;
    }
    if ((handle != _LLIO_STDOUT) && (handle != _LLIO_STDERR))
    {
        return _LLIO_ERROR;
    }
    /* Send data.*/
    while (size--)
    {
        if(Console.putc)
        {
            Console.putc(*buffer++);
        }
        ++nChars;   
    }
    return nChars;
}

size_t __read(int handle, unsigned char * buffer, size_t size)
{
    size_t nChars = 0;
    if (buffer == 0)
    {
        return 0;
    }
    if ((handle != _LLIO_STDIN) && (handle != _LLIO_STDERR))
    {
        return _LLIO_ERROR;
    }
    /* read data.*/
    while (size--)
    {
        if(Console.getc)
        {
            *buffer++ = Console.getc() & 0xFF;
        }
        ++nChars;
    }
    return nChars;
}
#endif /* comiler support */

 /**
 * @brief  软件复位
 * @retval None
 */
void SystemSoftReset(void)
{
    NVIC_SystemReset();
}


 /**
 * @brief  获得芯片 UID(唯一ID)
 * @param  None
 * @retval UID
 */
uint32_t GetUID(void) 
{
    return SIM->UUIDH ^ SIM->UUIDL;
}

#pragma weak ShowMemory
void ShowMemory(uint32_t addr, uint32_t length)
{
	int i = 0, j =0;

	addr = addr & ~0xF;
	length = 4*((length + 3)/4);

	while(i < length)
	{
		LIB_TRACE("0x%08x: ", addr );

		for(j=0; j<4; j++)
		{
			LIB_TRACE("0x%08x  ", *(uint32_t *)addr);
			addr += 4;
			i++;
		}

		LIB_TRACE("\r\n");
	}

	return;
}

#pragma weak RAMTest
uint32_t RAMTest(uint32_t addr, uint32_t length)
{
    uint32_t i;
    LIB_TRACE("memory test, address: 0x%08X length: 0x%08X\r\n", addr, length);
    
    /* 8-bit memory test */
    {
        volatile uint8_t * ptr_u8 = (uint8_t *)addr;
        for(i=0; i < length/sizeof(uint8_t); i++){
            *ptr_u8++ = (uint8_t)i;
        }

        ptr_u8 = (uint8_t *)addr;
        for(i=0; i < length/sizeof(uint8_t); i++){
            if( *ptr_u8 != (uint8_t)i ){
                LIB_TRACE("8-bit memory test fails on address 0x%08X\r\n",(uint32_t)ptr_u8);
                return CH_ERR;
            }
            ptr_u8++;
        }
        LIB_TRACE("8-bit memory test success!\r\n");
    }
    
    /* 16-bit memory test */
    {
        volatile uint16_t * ptr_u16 = (uint16_t *)addr;
        for(i=0; i < length/sizeof(uint16_t); i++){
            *ptr_u16++ = (uint16_t)i;
        }

        ptr_u16 = (uint16_t *)addr;
        for(i=0; i < length/sizeof(uint16_t); i++){
            if( *ptr_u16 != (uint16_t)i ){
                LIB_TRACE("16-bit memory test fails on address 0x%08X\r\n",(uint32_t)ptr_u16);
                return CH_ERR;
            }
            ptr_u16++;
        }
        LIB_TRACE("16-bit memory test success!\r\n");
    }

    /* 32-bit memory test */
    {
        volatile uint32_t * ptr_u32 = (uint32_t *)addr;
        for(i=0; i < length/sizeof(uint32_t); i++){
            *ptr_u32++ = (uint32_t)i;
        }

        ptr_u32 = (uint32_t *)addr;
        for(i=0; i < length/sizeof(uint32_t); i++){
            if( *ptr_u32 != (uint32_t)i ){
                LIB_TRACE("16-bit memory test fails on address 0x%08X\r\n",(uint32_t)ptr_u32);
                return CH_ERR;
            }
            ptr_u32++;
        }
        LIB_TRACE("32-bit memory test success!\r\n");
    }

    /**< 32-bit Loopback test */
    {
        volatile uint32_t * ptr_u32 = (uint32_t *)addr;
        for(i=0; i < length/sizeof(uint32_t); i++){
            *ptr_u32  = (uint32_t)ptr_u32;
            *ptr_u32++;
        }

        ptr_u32 = (uint32_t *)addr;
        for(i=0; i < length/sizeof(uint32_t); i++){
            if( *ptr_u32 != (uint32_t)ptr_u32 ){
                LIB_TRACE("32-bit memory test fails on address 0x%08X", (uint32_t)ptr_u32);
                LIB_TRACE(" data:0x%08X \r\n", (uint32_t)*ptr_u32);
                return CH_ERR;
            }
            ptr_u32++;
        }
        LIB_TRACE("32-bit Loopback test success!\r\n");
    }
    
    LIB_TRACE("all memory test are done!\r\n");
    return  CH_OK;
}

void JumpToImage(uint32_t addr)
{
    static uint32_t sp, pc;
    uint32_t *vectorTable = (uint32_t*)addr;
    sp = vectorTable[0];
    pc = vectorTable[1];
    
    typedef void(*app_entry_t)(void);

    /* must be static, otherwise SP value will be lost */
    static app_entry_t s_application = 0;

    s_application = (app_entry_t)pc;

    // Change MSP and PSP
    __set_MSP(sp);
    __set_PSP(sp);
    
    SCB->VTOR = addr;
    
    // Jump to application
    s_application();

    // Should never reach here.
    __NOP();
}

#pragma weak NMI_isr
void NMI_isr(void)
{
    
}

void NMI_Handler(void)
{
    //LIB_TRACE("NMI_Handler\r\n");
    NMI_isr();
}

/*
    int i;
    for(i=0;i<ARRAY_SIZE(MAPTbl);i++)
    {
        printf("(0X%08XU)\r\n", EncodeMAP(&MAPTbl[i]));
    }
*/

