/**
  ******************************************************************************
  * @file    wdog.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.1
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "wdog.h"
#include "common.h"

#if defined(WDOG) /* WDOG16 */

/**
 * @brief  解锁看门狗
 * @note   None
 * @retval None
 */
static void WDOG_Unlock(void)
{
    __disable_irq();
    WDOG->UNLOCK = 0xC520u;
    WDOG->UNLOCK = 0xD928u;
    __enable_irq();
}
/**
 * @brief  复位看门狗
 * @note   None
 * @retval None
 */
uint32_t WDOG_GetResetCnt(void)
{
    return (WDOG->RSTCNT);
}
/**
 * @brief  获得看门狗计数器数值
 * @note   None
 * @retval 计数器中的数值
 */
uint32_t WDOG_GetValue(void)
{
    uint32_t val;
    val = (WDOG->TMROUTH << 16);
    val |= WDOG->TMROUTL;
    return val;
}

/**
 * @brief  喂狗
 * @note   None
 * @retval None
 */
void WDOG_Refresh(void)
{
    uint32_t i;
    __disable_irq();
	WDOG->REFRESH = 0xA602u;
	WDOG->REFRESH = 0xB480u;
    __enable_irq();
    /* a gap of more then 20 bus cycle between 2 refresh sequence */
    for(i = 0; i < 20; i++)
    {
        __NOP();
    }
}
/**
 * @brief  关闭看门狗
 * @note   None
 * @retval None
 */
void WDOG_Disable(void)
{
    uint32_t wdog_value;
    WDOG_Unlock();
    wdog_value = WDOG->STCTRLH;
    wdog_value &= ~WDOG_STCTRLH_WDOGEN_MASK;
    WDOG->STCTRLH = wdog_value;
}
/**
 * @brief  初始化看门狗
 * @note   None
 * @param  ms: 设置看门狗喂狗最大时间间隔 单位ms
 * @retval None
 */
void WDOG_Init(uint32_t ms)
{
    uint32_t wdog_value = 0x01D3u;
    
    wdog_value &= ~(WDOG_STCTRLH_WINEN_MASK | WDOG_STCTRLH_CLKSRC_MASK);
    WDOG_Unlock();
    
    /* set timeout value */
    WDOG->TOVALH = (ms & 0xFFFF0000)>>16;
    WDOG->TOVALL = (ms & 0x0000FFFF)>>0;
    
    /* set window time value :timeout must greater then window time */
    WDOG->PRESC = WDOG_PRESC_PRESCVAL(0); // perscale = 8
    
    /* enable wdog */
    wdog_value |= WDOG_STCTRLH_WDOGEN_MASK;
    WDOG->STCTRLH = wdog_value;
}

#elif defined(WDOG0) /* WDOG32 */
/**
 * @brief  初始化看门狗
 * @note   None
 * @param  ms: 设置看门狗时间间隔 单位ms
 * @retval None
 */
void WDOG_Init(uint32_t ms)
{
    __disable_irq();
    WDOG0->CNT = WDOG_UPDATE_KEY;
    WDOG0->WIN = 0;
    WDOG0->TOVAL = ms;
    WDOG0->CS = WDOG_CS_UPDATE_MASK | WDOG_CS_CLK(1) | WDOG_CS_WAIT_MASK | WDOG_CS_STOP_MASK | WDOG_CS_EN_MASK;
    __enable_irq();
}

/**
 * @brief  关闭看门狗
 * @note   None
 * @retval None
 */
void WDOG_Disable(void)
{
    WDOG0->CNT = WDOG_UPDATE_KEY;
    WDOG0->CS &= ~WDOG_CS_EN_MASK;
}
/**
 * @brief  获得看门狗计数器数值
 * @note   None
 * @retval 计数器中的数值
 */
uint32_t WDOG_GetValue(void)
{
    return WDOG0->CNT;
}
/**
 * @brief  喂狗
 * @note   None
 * @retval None
 */
void WDOG_Refresh(void)
{
    __disable_irq();
    WDOG0->CNT = WDOG_REFRESH_KEY;
    __enable_irq();
}


#else /* COP */

void WDOG_Init(uint32_t ms)
{
    //SIM->COPC = SIM_COPC_COPT(1);
}

uint32_t WDOG_GetValue(void)
{
    return 0xFFFFFFFF;
}

uint32_t WDOG_GetResetCnt(void)
{
    return 0;
}

void WDOG_Refresh(void)
{
    __disable_irq();
#if defined(SIM_SRVCOP_SRVCOP_MASK)
    SIM->SRVCOP = 0x55;
    SIM->SRVCOP = 0xAA;
#endif
    __enable_irq();
}

#endif

