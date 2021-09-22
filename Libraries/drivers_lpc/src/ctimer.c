/**
  ******************************************************************************
  * @file    ctimer.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
  
#include "ctimer.h"
#include "common.h"

static uint32_t fac_us = 0;

CTIMER_Type* const CTIMERBases[] = CTIMER_BASE_PTRS;
static const IRQn_Type CTIMER_IRQTbl[] = CTIMER_IRQS;

static void CTIMER_EnableAllClocks(void)
{
    SYSCON->ASYNCAPBCTRL |= SYSCON_ASYNCAPBCTRL_ENABLE_MASK;
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_CTIMER0_MASK | SYSCON_AHBCLKCTRL_CTIMER1_MASK | SYSCON_AHBCLKCTRL_CTIMER2_MASK;
    ASYNC_SYSCON->ASYNCAPBCLKCTRL |= ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_MASK | ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER4_MASK;
}

void CTIMER_PWM_Init(uint32_t instance, uint32_t pwm_chl, uint32_t freq)
{
    uint32_t period;
    CTIMER_Type *CTIMERx = CTIMERBases[instance];
    
    /* enable clock */
    CTIMER_EnableAllClocks();
    
    /* Setup the cimer mode and count select */
    CTIMERx->CTCR = CTIMER_CTCR_CTMODE(0);
    CTIMERx->PR = 0;
    
    CTIMERx->PWMC |= (1U << pwm_chl);
    CTIMERx->MCR = CTIMER_MCR_MR3R_MASK;
    
    LIB_TRACE("PWM input clock after prescaler:%d\r\n", (GetClock(kCoreClock)/(CTIMERx->PR + 1)));
    
    period = ((GetClock(kCoreClock)/(CTIMERx->PR + 1)) / freq) - 1;
    CTIMERx->MR[3] = period;
    
    LIB_TRACE("PWM MR[3](perdiod):%d\r\n", CTIMERx->MR[3]);
    
    /* initial duty */
    CTIMERx->MR[pwm_chl] = period/2;
    
    /* clear IT flags */
    CTIMERx->IR = 0xFF;
    
    CTIMERx->TCR |= CTIMER_TCR_CEN_MASK;
}

void CTIMER_PWM_SetDuty(uint32_t instance, uint32_t pwm_chl, uint32_t duty)
{
    CTIMERBases[instance]->MR[pwm_chl] = (CTIMERBases[instance]->MR[3] * (10000 - duty)) / 10000;
}

void CTIMER_TC_Init(uint32_t instance, uint32_t us)
{
    CTIMER_Type *CTIMERx = CTIMERBases[instance];
    
    /* enable clock */
    CTIMER_EnableAllClocks();
    
    /* Setup the cimer mode and count select */
    CTIMERx->CTCR = CTIMER_CTCR_CTMODE(0);

    /* clear IT flags */
    CTIMERx->IR = 0xFF;
    
    /* reset on match */
    CTIMERx->MCR = CTIMER_MCR_MR0R_MASK;

    /* prescale value */
    CTIMERx->PR = 0;
    
    fac_us = GetClock(kCoreClock) / (CTIMERx->PR + 1);
    fac_us /= 1000000;
    
    LIB_TRACE("CTIMER TC fac_us:%d\r\n", fac_us);
    
    /* match value */
    CTIMERx->MR[0] = us * fac_us;
    
    CTIMER_Start(instance);
}

void CTIMER_CAP_Init(uint32_t instance, uint32_t chl)
{
    CTIMER_Type *CTIMERx = CTIMERBases[instance];
    
    /* enable clock */
    CTIMER_EnableAllClocks();
    
    /* Setup the cimer mode and count select */
    CTIMERx->CTCR = CTIMER_CTCR_CTMODE(2) | CTIMER_CTCR_CINSEL(chl);

    /* clear IT flags */
    CTIMERx->IR = 0xFF;
    
    /* prescale value */
    CTIMERx->PR = 0;
    
    fac_us = GetClock(kCoreClock) / (CTIMERx->PR + 1);
    fac_us /= 1000000;
    
    CTIMERx->PR = 0;
    CTIMERx->TC = 0;
    CTIMER_Start(instance);
}



void CTIMER_Stop(uint32_t instance)
{
    CTIMERBases[instance]->TCR &= ~CTIMER_TCR_CEN_MASK;
}

void CTIMER_Start(uint32_t instance)
{
    CTIMERBases[instance]->TCR |= CTIMER_TCR_CEN_MASK;
}

void CTIMER_TC_SetIntMode(uint32_t instance, uint32_t chl, bool val)
{
    switch(chl)
    {
        case HW_CTIMER_CH0:
            (val)?(CTIMERBases[instance]->MCR |= CTIMER_MCR_MR0I_MASK):(CTIMERBases[instance]->MCR &= ~CTIMER_MCR_MR0I_MASK);
            break;
        case HW_CTIMER_CH1:
            (val)?(CTIMERBases[instance]->MCR |= CTIMER_MCR_MR1I_MASK):(CTIMERBases[instance]->MCR &= ~CTIMER_MCR_MR1I_MASK);
            break;
        case HW_CTIMER_CH2:
            (val)?(CTIMERBases[instance]->MCR |= CTIMER_MCR_MR2I_MASK):(CTIMERBases[instance]->MCR &= ~CTIMER_MCR_MR2I_MASK);
            break;
        case HW_CTIMER_CH3:
            (val)?(CTIMERBases[instance]->MCR |= CTIMER_MCR_MR3I_MASK):(CTIMERBases[instance]->MCR &= ~CTIMER_MCR_MR3I_MASK);
            break;
        default:
            break;
    }

    NVIC_EnableIRQ(CTIMER_IRQTbl[instance]);
}

/* 
    mode: HW_CTIMER_CAP_INT_EVT_RE:  generate interrupt: 0->1 
    mode: HW_CTIMER_CAP_INT_EVT_FE:  generate interrupt: 1->0
    mode: HW_CTIMER_CAP_INT_EVT_ALL: any edge
*/
void CIMER_CAP_SetITandLoadMode(uint32_t instance, uint32_t chl, uint32_t mode)
{
    switch(chl)
    {
        case HW_CTIMER_CH0:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP0FE_MASK):(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP0RE_MASK);
            break;
        case HW_CTIMER_CH1:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP1FE_MASK):(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP1RE_MASK);
            break;
        case HW_CTIMER_CH2:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP2FE_MASK):(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP2RE_MASK);
            break;
        case HW_CTIMER_CH3:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP3FE_MASK):(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP3RE_MASK);
            break;
        default:
            break;
    }
}


void CTIMER_CAP_SetIntMode(uint32_t instance, uint32_t chl, bool val)
{
    
    switch(chl)
    {
        case HW_CTIMER_CH0:
            (val)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP0I_MASK):(CTIMERBases[instance]->CCR &= ~CTIMER_CCR_CAP0I_MASK);
            break;
        case HW_CTIMER_CH1:
            (val)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP1I_MASK):(CTIMERBases[instance]->CCR &= ~CTIMER_CCR_CAP1I_MASK);
            break;
        case HW_CTIMER_CH2:
            (val)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP2I_MASK):(CTIMERBases[instance]->CCR &= ~CTIMER_CCR_CAP2I_MASK);
            break;
        case HW_CTIMER_CH3:
            (val)?(CTIMERBases[instance]->CCR |= CTIMER_CCR_CAP3I_MASK):(CTIMERBases[instance]->CCR &= ~CTIMER_CCR_CAP3I_MASK);
            break;
        default:
            break;
    }

    NVIC_EnableIRQ(CTIMER_IRQTbl[instance]);
}


void CTIMER_SetCounter(uint32_t instance, uint32_t val)
{
    CTIMERBases[instance]->TC = val;
}

uint32_t CTIMER_GetCounter(uint32_t instance)
{
    return CTIMERBases[instance]->TC;
}

uint32_t CTIMER_GetCAPCounter(uint32_t instance, uint32_t chl)
{
    uint32_t CR;
    CR = CTIMERBases[instance]->CR[chl];
    return CR;
}

void CTIMER_SetTimerClearEvt(uint32_t instance, uint32_t chl, uint32_t mode, bool val)
{
    (val)?(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_ENCC_MASK):(CTIMERBases[instance]->CTCR &= ~CTIMER_CTCR_ENCC_MASK);
    CTIMERBases[instance]->CTCR &= ~CTIMER_CTCR_SELCC_MASK;
    
    switch(chl)
    {
        case HW_CTIMER_CH0:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(1)):(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(0));
            break;
        case HW_CTIMER_CH1:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(3)):(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(2));
            break;
        case HW_CTIMER_CH2:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(5)):(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(4));
            break;
        case HW_CTIMER_CH3:
            (HW_CTIMER_CAP_INT_EVT_FE)?(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(7)):(CTIMERBases[instance]->CTCR |= CTIMER_CTCR_SELCC(6));
            break;
        default:
            break;
    }
}
    
void CTIMER_IRQHandler(uint32_t instance)
{
    CTIMERBases[instance]->IR |= CTIMER_IR_MR0INT_MASK;
}
    
