/**
  ******************************************************************************
  * @file    common.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.17
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include <string.h>
#include "common.h"


/* 控制台接口 */
typedef struct
{
    int (*putc)(uint8_t ch);
    int (*getc)(void);
}Console_t;

/* 固件库全局变量 */
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
    uint32_t val;
    switch(clock)
    {
        case kCoreClock:
            val = SystemCoreClock;
            break;
        case kFROLfClock:
            val = 12*1000*1000;
            break;
        case kUSBPLLClock:
            val = 48*1000*1000;
            break;
        case kExtOSCClock:
            val = 12*1000*1000;
            break;
        case kFROHfClock:
            if(SYSCON->FROCTRL & SYSCON_FROCTRL_HSPDCLK_MASK)
            {
                val = (SYSCON->FROCTRL & SYSCON_FROCTRL_SEL_MASK)?(96*1000*1000):(48*1000*1000);
            }
            else
            {
                val = 0;
            }
            break;
        case kEMCClock:
            val = SystemCoreClock / 2;
            break;
        default:
            val = 0;
    }

    return val;
}

void SetFlexCommClk(uint32_t instance, uint32_t val)
{
    /*
    0x0 FRO 12 MHz (fro_12m)
    0x1 FRO 96 or 48 MHz (fro_hf)
    0x2 System PLL output (pll_clk)
    0x3 MCLK pin input, when selected in IOCON (mclk_in)
    0x4 FRG clock, the output of the fractional rate generator (frg_clk)
    0x7 None, this may be selected in order to reduce power when no output is needed.
    */
    
    #if defined(SYSCON_FXCOMCLKSEL_SEL_MASK)
    SYSCON->FXCOMCLKSEL[instance] = SYSCON_FXCOMCLKSEL_SEL(val);
    #else
    SYSCON->FCLKSEL[instance] = SYSCON_FCLKSEL_SEL(val);
    #endif
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
    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_IOCON_MASK;
    IOCON->PIO[instance][pin] &= ~IOCON_PIO_FUNC_MASK;
    IOCON->PIO[instance][pin] |= IOCON_PIO_FUNC(mux) | IOCON_PIO_DIGIMODE_MASK;
}

void SetPinAnalogMode(uint32_t instance, uint32_t pin, bool val)
{
    (val)?(IOCON->PIO[instance][pin] &= ~IOCON_PIO_DIGIMODE_MASK):(IOCON->PIO[instance][pin] |= IOCON_PIO_DIGIMODE_MASK);
}


void SetPinPull(uint32_t instance, uint32_t pin, uint32_t val)
{
    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_IOCON_MASK;
    
    IOCON->PIO[instance][pin] &= ~IOCON_PIO_MODE_MASK;
    switch(val)
    {
        case 0:
            IOCON->PIO[instance][pin] |= IOCON_PIO_MODE(1);
            break;
        case 1:
            IOCON->PIO[instance][pin] |= IOCON_PIO_MODE(2);
            break;
        default:
            break;
    }
}

uint32_t EncodeMAP(map_t * type)
{
    return *(uint32_t*)type;
}

void DecodeMAP(uint32_t map, map_t * type)
{
    map_t * pMap = (map_t*)&(map);
    memcpy(type, pMap, sizeof(map_t));  
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
 * @brief  获得复位原因
 * @note   复位原因代码具体需要查看芯片手册 SMC章节
 * @retval 复位原因代码
 */
uint32_t GetResetCause(void)
{
    return CH_ERR;
}

 /**
 * @brief  获得芯片 UID(唯一ID)
 * @param  None
 * @retval UID
 */
uint32_t GetUID(void) 
{
  //  return SIM->UIDL ^ SIM->UIDML ^ SIM->UIDMH;
    return CH_ERR;
}


#pragma weak ShowMemory
void ShowMemory(uint32_t addr, uint32_t size)
{
	int i = 0, j =0;

	addr = addr & ~0xF;
	size = 4*((size + 3)/4);

	while(i < size)
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

void JumpToImage(uint32_t addr)
{
    uint32_t *vectorTable = (uint32_t*)addr;
    uint32_t sp = vectorTable[0];
    uint32_t pc = vectorTable[1];
    
    typedef void(*app_entry_t)(void);

    uint32_t s_stackPointer = 0;
    uint32_t s_applicationEntry = 0;
    app_entry_t s_application = 0;

    s_stackPointer = sp;
    s_applicationEntry = pc;
    s_application = (app_entry_t)s_applicationEntry;

    // Change MSP and PSP
    #if !defined(LPC51U68)
    __set_MSP(s_stackPointer);
    #endif
    
    __set_PSP(s_stackPointer);
    
    SCB->VTOR = addr;
    
    // Jump to application
    s_application();

    // Should never reach here.
    __NOP();
}

#pragma weak RAMTest
uint32_t RAMTest(uint32_t address, uint32_t size)
{
    uint32_t i;
    LIB_TRACE("memtest,address: 0x%08X size: 0x%08X\r\n", address, size);
    
    /**< 8bit test */
    {
        volatile uint8_t * p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            *p_uint8_t++ = (uint8_t)i;
        }

        p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            if( *p_uint8_t != (uint8_t)i )
            {
                LIB_TRACE("8bit test fail @ 0x%08X\r\n",(uint32_t)p_uint8_t);
                return CH_ERR;
            }
            p_uint8_t++;
        }
        LIB_TRACE("8bit test pass!\r\n");
    }
    
    /**< 16bit test */
    {
        volatile uint16_t * p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            *p_uint16_t++ = (uint16_t)i;
        }

        p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            if( *p_uint16_t != (uint16_t)i )
            {
                LIB_TRACE("16bit test fail @ 0x%08X\r\n",(uint32_t)p_uint16_t);
                return CH_ERR;
            }
            p_uint16_t++;
        }
        LIB_TRACE("16bit test pass!\r\n");
    }

    /**< 32bit test */
    {
        volatile uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t++ = (uint32_t)i;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)i )
            {
                LIB_TRACE("32bit test fail @ 0x%08X\r\n",(uint32_t)p_uint32_t);
                return CH_ERR;
            }
            p_uint32_t++;
        }
        LIB_TRACE("32bit test pass!\r\n");
    }

    /**< 32bit Loopback test */
    {
        volatile uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t  = (uint32_t)p_uint32_t;
            *p_uint32_t++;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)p_uint32_t )
            {
                LIB_TRACE("32bit Loopback test fail @ 0x%08X", (uint32_t)p_uint32_t);
                LIB_TRACE(" data:0x%08X \r\n", (uint32_t)*p_uint32_t);
                return CH_ERR;
            }
            p_uint32_t++;
        }
        LIB_TRACE("32bit Loopback test pass!\r\n");
    }
    
    LIB_TRACE("all memory test complete\r\n");
    return  CH_OK;
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

