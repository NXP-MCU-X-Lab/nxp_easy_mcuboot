/**
  ******************************************************************************
  * @file    lpuart.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.13
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "common.h"
#include "lpuart.h"

#if defined(LPUART0)

#if defined(LPUART_BASE_PTRS)
#define LPUART_BASES    LPUART_BASE_PTRS
#endif

LPUART_Type * const LPUARTBases[] = LPUART_BASES;

extern bool _is_fitst_init;
static uint8_t UART_DebugInstance;

#if defined(PCC_LPUART0_INDEX)
static const Reg_t LPUARTClkGate[] =
{
    {(void*)&(PCC->CLKCFG[PCC_LPUART0_INDEX]), PCC_CLKCFG_CGC_MASK},
    {(void*)&(PCC->CLKCFG[PCC_LPUART1_INDEX]), PCC_CLKCFG_CGC_MASK},
};
#else
static const Reg_t LPUARTClkGate[] =
{
    /* LPUART0 */
#if defined(SIM_SCGC5_LPUART0_MASK) 
    {(void*)&(SIM->SCGC5), SIM_SCGC5_LPUART0_MASK, SIM_SCGC5_LPUART0_SHIFT},
#elif defined(SIM_SCGC2_LPUART0_MASK) 
    {(void*)&(SIM->SCGC2), SIM_SCGC2_LPUART0_MASK, SIM_SCGC2_LPUART0_SHIFT}, 
#elif defined(PCC1_PCC_LPUART0_CGC_MASK)
    {(void*)&(PCC1->PCC_LPUART0), PCC1_PCC_LPUART0_CGC_MASK, PCC1_PCC_LPUART0_CGC_SHIFT},
#elif defined(SIM_SCGC6_LPUART0_MASK)
    {(void*)&(SIM->SCGC6), SIM_SCGC6_LPUART0_MASK, SIM_SCGC6_LPUART0_SHIFT},
#else 
    #error "no LPUART0 clock gate defined!"
#endif
    
    /* LPUART1 */
#if defined(SIM_SCGC5_LPUART1_MASK)
    {(void*)&(SIM->SCGC5), SIM_SCGC5_LPUART1_MASK, SIM_SCGC5_LPUART1_SHIFT}, 
#elif defined(SIM_SCGC2_LPUART1_MASK)
    {(void*)&(SIM->SCGC2), SIM_SCGC2_LPUART1_MASK, SIM_SCGC2_LPUART1_SHIFT}, 
#elif defined(PCC1_PCC_LPUART1_CGC_MASK)
    {(void*)&(PCC1->PCC_LPUART1), PCC1_PCC_LPUART1_CGC_MASK, PCC1_PCC_LPUART1_CGC_SHIFT},
#else 
#endif
    
    /* LPUART2 */
#if defined(SIM_SCGC5_LPUART2_MASK)
    {(void*)&(SIM->SCGC5), SIM_SCGC5_LPUART2_MASK, SIM_SCGC5_LPUART2_SHIFT}, 
#elif defined(SIM_SCGC2_LPUART2_MASK)
    {(void*)&(SIM->SCGC2), SIM_SCGC2_LPUART2_MASK, SIM_SCGC2_LPUART2_SHIFT}, 
#elif defined(PCC0_PCC_LPUART2_CGC_MASK)
    {(void*)&(PCC0->PCC_LPUART2), PCC0_PCC_LPUART2_CGC_MASK, PCC0_PCC_LPUART2_CGC_SHIFT},
#else 
#endif
};
#endif

#if defined(MKE18F16)

static const IRQn_Type LPUART_IRQTbl[] = 
{
    LPUART0_RX_IRQn,
    LPUART1_RX_IRQn, 
};
#else
static const IRQn_Type LPUART_IRQTbl[] = 
{
    (IRQn_Type)(LPUART0_IRQn),
#if defined(LPUART1)
    (IRQn_Type)(LPUART1_IRQn),
#endif
#if defined(LPUART2)
    (IRQn_Type)(LPUART2_IRQn),
#endif 
};
#endif

static inline int Getc(void)
{
    uint8_t ch;
    while(LPUART_GetChar(UART_DebugInstance, &ch) != CH_OK);
    return ch;
}

static inline int Putc(uint8_t ch)
{
    LPUART_PutChar(UART_DebugInstance, ch);
    return ch;
}

uint32_t LPUART_SetClock(uint32_t instance, uint32_t opt)
{
    uint32_t clk;
#if defined(SCG)
    /* All PD functional clock fixed to FIRC 48M, FIRC must be opened */
    REG_CLR(LPUARTClkGate, instance);
    *((uint32_t*)LPUARTClkGate[instance].addr) = PCC_CLKCFG_PCS(3);
    REG_SET(LPUARTClkGate, instance);
    clk = (48*1000*1000);

#elif defined(MCG)
    /* clock config use PLL or FLL */
#if defined(SIM_SOPT2_LPUARTSRC_MASK)
    SIM->SOPT2 &= ~SIM_SOPT2_LPUARTSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_LPUARTSRC(1);
#endif
    
    /* if PLL enabled, use PLL */
#if defined(SIM_SOPT2_PLLFLLSEL_MASK)
    if(MCG->C6 & MCG_C6_PLLS_MASK) 
    {
        SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
        SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL(1);
    }
#endif
        
    clk = GetClock(kCoreClock);
            
    switch(instance)
    {
        case HW_LPUART0:
            #if defined(SIM_SOPT2_LPUART0SRC_MASK)
            SIM->SOPT2 &= ~SIM_SOPT2_LPUART0SRC_MASK;
            SIM->SOPT2 |= SIM_SOPT2_LPUART0SRC(1);
            #elif defined(SIM_SOPT2_LPUARTSRC_MASK)
            SIM->SOPT2 &= ~SIM_SOPT2_LPUARTSRC_MASK;
            SIM->SOPT2 |= SIM_SOPT2_LPUARTSRC(1);
            #else
            #warning "No LPUART0 clock src select!"
            #endif
            break;
#if defined(LPUART1)
        case HW_LPUART1:
            #if defined(SIM_SOPT2_LPUART1SRC_MASK)
            SIM->SOPT2 &= ~SIM_SOPT2_LPUART1SRC_MASK;
            SIM->SOPT2 |= SIM_SOPT2_LPUART1SRC(1);
            #elif defined(SIM_SOPT2_LPUARTSRC_MASK)
            SIM->SOPT2 &= ~SIM_SOPT2_LPUARTSRC_MASK;
            SIM->SOPT2 |= SIM_SOPT2_LPUARTSRC(1);
            #else
            #warning "No LPUART1 clock src select!"
            #endif
            break;
#endif
    }
#endif /* MCG */
    return clk;
}

/**
 * @brief  设置串口波特率
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @param  baud : 通信波特率
 * @retval None
 */
void LPUART_SetBaudRate(uint32_t instance, uint32_t baud)
{
    LPUART_Type *LPUARTx;
    LPUARTx = LPUARTBases[instance];
    uint32_t clk, sbr, osr, temp, calculatedBaud;
    
    LPUARTx->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
    
    /* baudrate */
    clk = LPUART_SetClock(instance, 0);
    for(osr=4; osr<32; osr++)
    {
        sbr = clk/(baud * osr);
        calculatedBaud = clk/(osr*sbr);
        if(ABS((int)calculatedBaud - (int)baud) < (baud*0.03))
        {
            break;
        }
    }
    
    temp = LPUARTx->BAUD;
    if ((osr > 3) && (osr < 8))
    {
        temp |= LPUART_BAUD_BOTHEDGE_MASK;
    }
    temp &= ~(LPUART_BAUD_SBR_MASK | LPUART_BAUD_OSR_MASK);
    temp |= LPUART_BAUD_SBR(sbr) | LPUART_BAUD_OSR(osr-1);
    LPUARTx->BAUD = temp;
    
    LPUARTx->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
}

/**
 * @brief  删除初始化串口配置
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @retval CH_OK 成功 其他 错误代码
 */
uint32_t LPUART_DeInit(uint32_t instance)
{
    LPUART_Type *LPUARTx = (LPUART_Type*)LPUARTBases[instance];
    
    LPUARTx->CTRL = 0;
    LPUARTx->BAUD = 0x0F000004;
    LPUARTx->STAT = 0xFFFFFFFF; /* clear all status */
    LPUARTx->STAT = 0xC01FC000; /* set to default state */
    LPUARTx->DATA = 0x00001000;
    return CH_OK;
}

/**
 * @brief  初始化UART
 * @note   None
 * @param  MAP ：串口引脚位图，详见lpuart.h文件
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @param  baud : 通信波特率
 * @retval 串口端口号
 */
uint32_t LPUART_Init(uint32_t MAP, uint32_t baudrate)
{
    map_t * pq = (map_t*)&(MAP);
    
    REG_SET(LPUARTClkGate, pq->ip);
    
    /* set baudrate */
    LPUART_SetBaudRate(pq->ip, baudrate);
    
    PIN_SET_MUX;
    if(_is_fitst_init)
    {
        UART_DebugInstance = pq->ip;
        SetConsole(Putc, Getc);
        _is_fitst_init = false;
    }
    return pq->ip;
}

/**
 * @brief  串口发送一个字符
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @param  ch: 需要发送的字符
 * @retval None
 */
void LPUART_PutChar(uint32_t instance, uint8_t ch)
{
    while(!(LPUARTBases[instance]->STAT & LPUART_STAT_TDRE_MASK));
    LPUARTBases[instance]->DATA = (ch & 0xFF);
}

/**
 * @brief  串口接收一个字符
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @param  ch: 获得字符
 * @retval CH_OK：成功；CH_ERR：错误
 */
uint32_t LPUART_GetChar(uint32_t instance, uint8_t *ch)
{
    LPUART_Type *LPUARTx = (LPUART_Type*)LPUARTBases[instance];
    if(LPUARTx->STAT & LPUART_STAT_RDRF_MASK)
    {
        *ch = (uint8_t)(LPUARTx->DATA);	
        return CH_OK; 		  
    }
    
    if(LPUARTx->STAT & LPUART_STAT_OR_MASK)
    {
        LPUARTx->STAT |= LPUART_STAT_OR_MASK;
    //    ch = (uint8_t)LPUARTx->DATA;
    }
    
    return CH_ERR;
}

/**
 * @brief  设置串口中断模式
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : 串口端口UART0-UART2
 * @param  mode:
 *         @arg kUART_IntTx : UART发送完成中断
 *         @arg kUART_IntRx : UART接收完成中断
 *         @arg kUART_IntIdleLine : UART数据线空闲中断
 * @param  val : 
 *         @arg true : 开启中断模式
 *         @arg false : 关闭中断模式
 * @retval CH_OK：成功；其他：错误代码
 */
uint32_t LPUART_SetIntMode(uint32_t instance, LPUART_Int_t mode, bool val)
{
    REG_SET(LPUARTClkGate, instance);
    LPUART_Type *LPUARTx = (LPUART_Type*)LPUARTBases[instance];
    NVIC_EnableIRQ(LPUART_IRQTbl[instance]);
    switch(mode)
    {
        case kLPUART_IntTx:
            (val)?(LPUARTx->CTRL |= LPUART_CTRL_TIE_MASK):(LPUARTx->CTRL &= ~LPUART_CTRL_TIE_MASK);
            break;
        case kLPUART_IntRx:
            (val)?(LPUARTx->CTRL |= LPUART_CTRL_RIE_MASK):(LPUARTx->CTRL &= ~LPUART_CTRL_RIE_MASK);
            break;
        case kLPUART_IntIdleLine:
            (val)?(LPUARTx->CTRL |= LPUART_CTRL_ILIE_MASK):(LPUARTx->CTRL &= ~LPUART_CTRL_ILIE_MASK);
            break;
        default:
            break;
    }
    return CH_OK;
}


void LPUART_IRQHandler(uint32_t instance)
{
    //uint8_t ch;
    LPUART_Type *LPUARTx = (LPUART_Type*)LPUARTBases[instance];

    if(LPUARTx->STAT & LPUART_STAT_TDRE_MASK)
    {
        /* send data */
      //  LPUARTx->DATA = (uint8_t)ch;
    }
    
    if(LPUARTx->STAT & LPUART_STAT_RDRF_MASK)
    {
    //    ch = (uint8_t)LPUARTx->DATA;
    }
    
    if(LPUARTx->STAT & LPUART_STAT_OR_MASK)
    {
        LPUARTx->STAT |= LPUART_STAT_OR_MASK;
    }
}

#endif
