/**
  ******************************************************************************
  * @file    uart.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2015.6.21
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "uart.h"
#include "common.h"

#if defined(UART0)

extern bool _is_fitst_init;
static uint8_t UART_DebugInstance;

#ifdef UART0_BDH_SBR_MASK
    #if defined(UART2)
    const void* UARTBases[] = {UART0, UART1, UART2};
    #elif defined(UART1)
    const void* UARTBases[] = {UART0, UART1};
    #elif defined(UART0)
    const void* UARTBases[] = {UART0};
    #else
    const void* UARTBases[] = UART_BASES;
    #endif

#else

#ifndef UART_BASES
#define UART_BASES {UART0, UART1};
#endif

const void* UARTBases[] = UART_BASES;
#endif /* UART0_BDH_SBR_MASK */

#if defined(SIM_SCGC4_UART0_MASK)
static const Reg_t UARTClkGate[] =
{
    {(void*)&(SIM->SCGC4), SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART0_SHIFT},
#if defined(SIM_SCGC4_UART1_MASK) 
    {(void*)&(SIM->SCGC4), SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART1_SHIFT},
#endif
#if defined(SIM_SCGC4_UART2_MASK) 
    {(void*)&(SIM->SCGC4), SIM_SCGC4_UART2_MASK, SIM_SCGC4_UART2_SHIFT},
#endif
#if defined(SIM_SCGC4_UART3_MASK) 
    {(void*)&(SIM->SCGC4), SIM_SCGC4_UART3_MASK, SIM_SCGC4_UART3_SHIFT},
#endif
#if defined(SIM_SCGC1_UART4_MASK) 
    {(void*)&(SIM->SCGC1), SIM_SCGC1_UART4_MASK, SIM_SCGC1_UART4_SHIFT},
#endif  
#if defined(SIM_SCGC1_UART5_MASK) 
    {(void*)&(SIM->SCGC1), SIM_SCGC1_UART5_MASK, SIM_SCGC1_UART5_SHIFT},
#endif   
};
#endif


#if (defined(MK64F12) || defined(MK70F15) || defined(MK60D10) || defined(MK22F12) || defined(MKS22F25612) || defined(MK26F18) || defined(MK20D5) || defined(MK02F12810))
static const IRQn_Type UART_IRQTbl[] =
{
    (IRQn_Type)(UART0_RX_TX_IRQn),
    (IRQn_Type)(UART1_RX_TX_IRQn),
#if defined(UART2)
    (IRQn_Type)(UART2_RX_TX_IRQn),
#endif
#if defined(UART3)
    (IRQn_Type)(UART3_RX_TX_IRQn),
#endif
#if defined(UART4)
    (IRQn_Type)(UART4_RX_TX_IRQn),
#endif
#if defined(UART5)
    (IRQn_Type)(UART5_RX_TX_IRQn),
#endif
};
#else
static const IRQn_Type UART_IRQTbl[] =
{
    (IRQn_Type)(UART0_IRQn),
#if defined(UART1)
    (IRQn_Type)(UART1_IRQn),
#endif
#if defined(UART2)
    (IRQn_Type)(UART2_IRQn),
#endif
};
#endif

static inline int Getc(void)
{
    uint8_t ch;
    while(UART_GetChar(UART_DebugInstance, &ch) != CH_OK);
    return ch;
}

static inline int Putc(uint8_t ch)
{
    UART_PutChar(UART_DebugInstance, ch);
    return ch;
}
/**
 * @brief  设置串口时钟源
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  opt : 无意义
 * @retval 时钟速度
 */
uint32_t UART_SetClock(uint32_t instance)
{
    uint32_t clk;
#ifdef SIM_SOPT2_UART0SRC_MASK
    if(instance == HW_UART0)
    {
        clk = GetClock(kMCGOutClock);
        SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK; /* use PLL/2 or FLL */
        SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);  
        #ifdef MCG_C6_PLLS_MASK
        if(MCG->C6 & MCG_C6_PLLS_MASK) /* PLL */
        {
            SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
            clk /= 2;
        }
        else /* FLL */
        {
            SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
        }
        #endif /* MCG_C6_PLLS_MASK */
    }
    else
    {
        clk = GetClock(kBusClock);
    }
#else
    /* UART0 and UART1 use SystemClock, others use BusClock */
    (instance == HW_UART0) || (instance == HW_UART1)?(clk = GetClock(kCoreClock)):(clk = GetClock(kBusClock));
#endif /* SIM_SOPT2_UART0SRC_MASK */
    return clk;
}

/**
 * @brief  设置串口波特率
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  baud : 通信波特率
 * @retval None
 */
void UART_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t clk, sbr;
    UART_Type *UARTx = (UART_Type*)UARTBases[instance];
    clk = UART_SetClock(instance);
    /* disable module */
    UARTx->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    
    /* caluate OSR */
    
    if(instance == HW_UART0)
    {
        #if defined(UART0_C4_OSR_MASK)
        uint32_t osr, calculatedBaud;
        for(osr=5; osr<32; osr++)
        {
                
            sbr = clk/(baud * osr);
            calculatedBaud = clk/(osr*sbr);
            if(ABS((int)calculatedBaud - (int)baud) < (baud*0.03))
            {
                break;
            }
        }
        UART0->C4 = UART0_C4_OSR(osr-1);
        #else
        sbr = clk/(baud * 16);
        #endif
    }
    else
    {
        sbr = clk/(baud * 16);
    }
    
    UARTx->BDH &= ~(UART_BDH_SBR_MASK);
    UARTx->BDH |= (sbr>>8) & UART_BDH_SBR_MASK;
    UARTx->BDL = (sbr & UART_BDL_SBR_MASK);
    
    #ifdef UART_C4_BRFA_MASK
    uint32_t brfa;
    brfa = (2*clk)/baud - 32*sbr;
    UARTx->C4 &= ~UART_C4_BRFA_MASK;
    UARTx->C4 |= UART_C4_BRFA(brfa);
    #endif /* UART_C4_BRFA_MASK */
    
    /* enable moudle */
    UARTx->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
}

/**
 * @brief  初始化配置串口
 * @note   None
 * @param  MAP : 串口引脚位图,详见uart.h文件
 * @param  baudrate : 通信波特率
 * @retval 通信波特率
 */
uint32_t UART_Init(uint32_t MAP, uint32_t baudrate)
{
    map_t * pq = (map_t*)&(MAP);
    
    uint32_t instance = pq->ip;
    REG_SET(UARTClkGate, instance);
    
    UART_SetBaudRate(pq->ip, baudrate);
    
    PIN_SET_MUX;
    if(_is_fitst_init)
    {
        UART_DebugInstance = instance;
        SetConsole(Putc, Getc);
        _is_fitst_init = false;
    }
    return pq->ip;
}

/**
 * @brief  删除初始化UART配置
 * @note   None
 * @param  MAP : 串口引脚位图,详见uart.h文件
 * @retval CH_OK ：成功；其他:失败
 */
uint32_t UART_DeInit(uint32_t MAP)
{
    map_t * pq = (map_t*)&(MAP);
    UART_Type *UARTx = (UART_Type*)UARTBases[pq->ip];
    UARTx->BDH = 0;
    UARTx->BDL = 0;
    UARTx->C1 = 0;
    UARTx->C2 = 0;
    UARTx->C3 = 0;
    UARTx->C4 = 0;
    return CH_OK;
}

/**
 * @brief  设置串口DMA模式
 * @note   开启DMA触发模式后，必须再开启Tx 中断 才能真正使能DMA信号(仅限UART, LPUART无此限制)
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  mode:
 *         @arg kUART_DMATx : UART-DMA 模式发送
 *         @arg kUART_DMARx : UART-DMA 模式接收
 * @param  val : 
 *         @arg 0 : 开启UART DMA请求
 *         @arg 1 : 关闭UART DMA请求
 * @retval CH_OK :成功; 其他:失败
 */
uint32_t UART_SetDMAMode(uint32_t instance, UART_DMA_t mode, bool val)
{
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];

    if(val == false)
    {
        UART_SetIntMode(instance, (UART_Int_t)mode, false);
    }
    
    /* for SCI */
#if defined(UART0_C5_TDMAE_MASK)
    if(instance == 0)
    {
        switch(mode)
        {
            case kUART_DMATx:
                (val)?(UART0->C5 |= UART0_C5_TDMAE_MASK):(UART0->C5 &= ~UART0_C5_TDMAE_MASK);
                break;
            case kUART_DMARx:
                (val)?(UART0->C5 |= UART0_C5_RDMAE_MASK):(UART0->C5 &= ~UART0_C5_RDMAE_MASK);
                break;
        }
        return CH_OK;  
    }
#endif
    
    switch(mode)
    {
        case kUART_DMATx:
#if defined(UART_C4_TDMAS_MASK)
            (val)?(UARTx->C4 |= UART_C4_TDMAS_MASK):(UARTx->C4 &= ~UART_C4_TDMAS_MASK);
#elif defined(UART_C5_TDMAS_MASK)
            (val)?(UARTx->C5 |= UART_C5_TDMAS_MASK):(UARTx->C5 &= ~UART_C5_TDMAS_MASK);
#else
            #error "Cannot enable UART Tx"
#endif
            break;
        case kUART_DMARx:
#if defined(UART_C4_RDMAS_MASK)
            (val)?(UARTx->C4 |= UART_C4_RDMAS_MASK):(UARTx->C4 &= ~UART_C4_RDMAS_MASK);
#elif defined(UART_C5_RDMAS_MASK)
            (val)?(UARTx->C5 |= UART_C5_RDMAS_MASK):(UARTx->C5 &= ~UART_C5_RDMAS_MASK);
#else
            #error "Cannot enable UART Rx"
#endif 
            break;
    }
    
    if(val == true)
    {
        UART_SetIntMode(instance, (UART_Int_t)mode, true);
    }
    
    return CH_OK;
}

/**
 * @brief  设置串口中断模式
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  mode:
 *         @arg kUART_IntTx : UART- 发送完成中断
 *         @arg kUART_IntRx : UART- 接收完成中断
 *         @arg kUART_IntIdleLine : UART-IDLE模式接收
 * @param  val : 
 *         @arg true : 开启中断模式
 *         @arg false : 关闭中断模式
 * @retval CH_OK :成功; 其他:失败
 */
uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val)
{
    REG_SET(UARTClkGate, instance);
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    NVIC_EnableIRQ(UART_IRQTbl[instance]);
    switch(mode)
    {
        case kUART_IntTx:
            (val)?(UARTx->C2 |= UART_C2_TIE_MASK):(UARTx->C2 &= ~UART_C2_TIE_MASK);
            break;
        case kUART_IntRx:
            (val)?(UARTx->C2 |= UART_C2_RIE_MASK):(UARTx->C2 &= ~UART_C2_RIE_MASK);
            break;
        case kUART_IntIdleLine:
            (val)?(UARTx->C2 |= UART_C2_ILIE_MASK):(UARTx->C2 &= ~UART_C2_ILIE_MASK);
            break;
        default:
            break;
    }
    return CH_OK;
}

void UART_EnableTxFIFO(uint32_t instance, bool val)
{
#if defined(UART_PFIFO_TXFE_MASK)
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    (val)?
    (UARTx->PFIFO |= UART_PFIFO_TXFE_MASK):
    (UARTx->PFIFO &= ~UART_PFIFO_TXFE_MASK);
#endif
}

void UART_EnableRxFIFO(uint32_t instance, bool val)
{
#if defined(UART_PFIFO_RXFE_MASK)
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    (val)?
    (UARTx->PFIFO |= UART_PFIFO_RXFE_MASK):
    (UARTx->PFIFO &= ~UART_PFIFO_RXFE_MASK);
#endif
}

/**
 * @brief  串口发送一个字符
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  ch : 需要发送的字符
 * @retval None
 */
void UART_PutChar(uint32_t instance, uint8_t ch)
{
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    while(!(UARTx->S1 & UART_S1_TDRE_MASK));
    UARTx->D = (uint8_t)ch;
}

/**
 * @brief  串口接收一个字符
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART端口号0-5
 * @param  ch : 获得字符
 * @retval CH_OK :成功; 其他:失败
 */
uint32_t UART_GetChar(uint32_t instance, uint8_t *ch)
{
    UART_Type * UARTx = (UART_Type*)UARTBases[instance];
    if(UARTx->S1 & UART_S1_RDRF_MASK)
    {
        *ch = (uint8_t)(UARTx->D);	
        return CH_OK; 		  
    }
    if(UARTx->S1 & UART_S1_OR_MASK)
    {
        uint8_t *dummy = (uint8_t*)&UARTx->S1;
        (*dummy) |= UART_S1_OR_MASK;
    }
    return CH_ERR;
}


/*
static const QuickInit_Type UART_QuickInitTable[] =
{
    { 1, 4, 3, 0, 2, 0}, //UART1_RX_PE01_TX_PE00
    { 2, 4, 3,16, 2, 0}, //UART2_RX_PE17_TX_PE16
    { 0, 4, 4,20, 2, 0}, //UART0_RX_PE21_TX_PE20
    { 2, 4, 4,22, 2, 0}, //UART2_RX_PE23_TX_PE22
    { 0, 0, 2, 1, 2, 0}, //UART0_RX_PA01_TX_PA02
    { 0, 0, 3,14, 2, 0}, //UART0_RX_PA15_TX_PA14
    { 1, 0, 3,18, 2, 0}, //UART1_RX_PA18_TX_PA19
    { 0, 1, 3,16, 2, 0}, //UART0_RX_PB16_TX_PB17
    { 2, 3, 3, 2, 2, 0}, //UART2_RX_PD02_TX_PD03
    { 2, 3, 3, 4, 2, 0}, //UART2_RX_PD04_TX_PD05
    { 0, 3, 3, 6, 2, 0}, //UART0_RX_PD06_TX_PD07
};

void UART0_RX_TX_IRQHandler(void)
{
    uint8_t ch;
    if(UART_GetChar(HW_UART0, &ch) == CH_OK)
    {
        printf("UART interrupt:%c\r\n", ch);
    }
    
}

*/


#endif
