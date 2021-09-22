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

#if defined(USART0)


USART_Type* const UARTBases[] = USART_BASE_PTRS;
static const IRQn_Type UART_IRQTbl[] = USART_IRQS;
static FLEXCOMM_Type* const FLEXCOMMBases[] = FLEXCOMM_BASE_PTRS;

extern bool _is_fitst_init;
static uint8_t UART_DebugInstance;


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
 * @brief  ���ô���ʱ��Դ
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  opt : ������
 * @retval ʱ���ٶ�
 */
uint32_t UART_SetClock(uint32_t instance)
{
    SetFlexCommClk(instance, 1);
    return GetClock(kFROHfClock);
}

void UART_SetLoopbackMode(uint32_t instance, bool val)
{
    USART_Type *USARTx = UARTBases[instance];
    
    USARTx->CFG &= ~USART_CFG_ENABLE_MASK;
    (val)?(USARTx->CFG |= USART_CFG_LOOP_MASK):(USARTx->CFG &= ~USART_CFG_LOOP_MASK);
    USARTx->CFG |= USART_CFG_ENABLE_MASK;
}

bool UART_GetLoopbackMode(uint32_t instance)
{
    USART_Type *USARTx = UARTBases[instance];
    if(USARTx->CFG & USART_CFG_LOOP_MASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}



/**
 * @brief  ���ô��ڲ�����
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  baud : ͨ�Ų�����
 * @retval None
 */
void UART_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t best_diff = (uint32_t)-1, best_osrval = 0xf, best_brgval = (uint32_t)-1;
    uint32_t osrval, brgval, diff, baudrate;

    uint32_t clk = UART_SetClock(instance);
    
    USART_Type *USARTx = UARTBases[instance];
    
    for (osrval = best_osrval; osrval >= 4; osrval--)
    {
        brgval = (clk / ((osrval + 1) * baud)) - 1;
        if (brgval > 0xFFFF)
        {
            continue;
        }
        baudrate = clk / ((osrval + 1) * (brgval + 1));
        diff = baud < baudrate ? baudrate - baud : baud - baudrate;
        if (diff < best_diff)
        {
            best_diff = diff;
            best_osrval = osrval;
            best_brgval = brgval;
        }
    }

    USARTx->OSR = best_osrval;
    USARTx->BRG = best_brgval;
}


/**
 * @brief  ��ʼ�����ô���
 * @note   None
 * @param  MAP : ��������λͼ,���uart.h�ļ�
 * @param  baudrate : ͨ�Ų�����
 * @retval ͨ�Ų�����
 */
uint32_t UART_Init(uint32_t MAP, uint32_t baudrate)
{
    map_t * pq = (map_t*)&(MAP);
    
    uint32_t instance = MAP;
    
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK 
                            | SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK;
                     
#if defined(SYSCON_AHBCLKCTRL_FLEXCOMM8_MASK)    
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_FLEXCOMM8_MASK;
#endif
    
#if defined(SYSCON_AHBCLKCTRL_FLEXCOMM9_MASK)    
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_FLEXCOMM9_MASK;
#endif
    
    /* select flexcomm to UART */
    FLEXCOMMBases[instance]->PSELID &= ~FLEXCOMM_PSELID_PERSEL_MASK;
    FLEXCOMMBases[instance]->PSELID |= FLEXCOMM_PSELID_PERSEL(1);
    
    
	// Get the System Clock frequency for the BRG calculation.
//	SystemCoreClockUpdate();
	
    UART_SetBaudRate(instance, baudrate);

    UARTBases[instance]->CTL = 0;

    // Clear any pending flags (Just to be safe, isn't necessary after the peripheral reset)
    UARTBases[instance]->STAT = 0xFFFF;

    // Enable USART
    UARTBases[instance]->CFG |= USART_CFG_ENABLE_MASK | USART_CFG_DATALEN(1);

    if(_is_fitst_init)
    {
        UART_DebugInstance = instance;
        SetConsole(Putc, Getc);
        _is_fitst_init = false;
    }
    
    return pq->ip;
}

/**
 * @brief  ɾ����ʼ��UART����
 * @note   None
 * @param  MAP : ��������λͼ,���uart.h�ļ�
 * @retval CH_OK ���ɹ�������:ʧ��
 */
uint32_t UART_DeInit(uint32_t MAP)
{
    return CH_OK;
}

/**
 * @brief  ���ô���DMAģʽ
 * @note   ����DMA����ģʽ�󣬱����ٿ���Tx �ж� ��������ʹ��DMA�ź�(����UART, LPUART�޴�����)
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  mode:
 *         @arg kUART_DMATx : UART-DMA ģʽ����
 *         @arg kUART_DMARx : UART-DMA ģʽ����
 * @param  val : 
 *         @arg 0 : ����UART DMA����
 *         @arg 1 : �ر�UART DMA����
 * @retval CH_OK :�ɹ�; ����:ʧ��
 */
uint32_t UART_SetDMAMode(uint32_t instance, UART_DMA_t mode, bool val)
{
    UARTBases[instance]->FIFOCFG |= USART_FIFOCFG_ENABLETX_MASK | USART_FIFOCFG_ENABLERX_MASK | USART_FIFOCFG_DMATX_MASK | USART_FIFOCFG_DMARX_MASK;
    return CH_OK;
}

/**
 * @brief  ���ô����ж�ģʽ
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  mode:
 *         @arg kUART_IntTx : UART- ��������ж�
 *         @arg kUART_IntRx : UART- ��������ж�
 *         @arg kUART_IntIdleLine : UART-IDLEģʽ����
 * @param  val : 
 *         @arg true : �����ж�ģʽ
 *         @arg false : �ر��ж�ģʽ
 * @retval CH_OK :�ɹ�; ����:ʧ��
 */
uint32_t UART_SetIntMode(uint32_t instance, UART_Int_t mode, bool val)
{
    
    NVIC_EnableIRQ(UART_IRQTbl[instance]);
    
    switch(mode)
    {
        case kUART_IntRx:
            (val)?(UARTBases[instance]->FIFOINTENSET |= USART_FIFOINTENSET_RXLVL_MASK):(UARTBases[instance]->FIFOINTENSET &= ~USART_FIFOINTENSET_RXLVL_MASK);
           // (val)?(UARTBases[instance]->INTENSET |= RXRDY):(UARTBases[instance]->INTENSET &= ~RXRDY);
            break;
        case kUART_IntTx:
          //  (val)?(UARTBases[instance]->INTENSET |= TXRDY):(UARTBases[instance]->INTENSET &= ~TXRDY);
            break;
        default:
            break;
    }
    
    if(!val)
    {
        NVIC_DisableIRQ(UART_IRQTbl[instance]);
    }
    
    return CH_OK;
}

    
/**
 * @brief  ���ڷ���һ���ַ�
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  ch : ��Ҫ���͵��ַ�
 * @retval None
 */
void UART_PutChar(uint32_t instance, uint8_t ch)
{
    while (!((UARTBases[instance]->STAT) & USART_STAT_TXIDLE_MASK));   // Wait for TX Ready
    UARTBases[instance]->FIFOWR = ch;
  //  UARTBases[instance]->TXDAT = ch;
}

/**
 * @brief  ���ڽ���һ���ַ�
 * @note   None
 * @param  instance:
 *         @arg HW_UARTx : UART�˿ں�0-5
 * @param  ch : ����ַ�
 * @retval CH_OK :�ɹ�; ����:ʧ��
 */
uint32_t UART_GetChar(uint32_t instance, uint8_t *ch)
{
    if(UARTBases[instance]->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK)
    {
        *ch = (uint8_t)UARTBases[instance]->FIFORD;
        return CH_OK;
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
