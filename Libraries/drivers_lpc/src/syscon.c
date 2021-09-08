/**
  ******************************************************************************
  * @file    syscon.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
  
  
#include "syscon.h"
#include "common.h"
#include "fsl_power.h"


#define PLL_SSCG0_MDEC_VAL_P (0U)                                /* MDEC is in bits  16 downto 0 */
#define PLL_SSCG0_MDEC_VAL_M (0x1FFFFUL << PLL_SSCG0_MDEC_VAL_P) /* NDEC is in bits  9 downto 0 */
#define PLL_NDEC_VAL_P (0U)                                      /* NDEC is in bits  9:0 */
#define PLL_NDEC_VAL_M (0x3FFUL << PLL_NDEC_VAL_P)
#define PLL_PDEC_VAL_P (0U) /* PDEC is in bits 6:0 */
#define PLL_PDEC_VAL_M (0x3FFUL << PLL_PDEC_VAL_P)
#define NVALMAX (0x100U)
#define PVALMAX (0x20U)
#define MVALMAX (0x8000U)


/* Find encoded NDEC value for raw N value, max N = NVALMAX */
static uint32_t pllEncodeN(uint32_t N)
{
    uint32_t x, i;

    /* Find NDec */
    switch (N)
    {
        case 0U:
            x = 0xFFFU;
            break;

        case 1U:
            x = 0x302U;
            break;

        case 2U:
            x = 0x202U;
            break;

        default:
            x = 0x080U;
            for (i = N; i <= NVALMAX; i++)
            {
                x = (((x ^ (x >> 2U) ^ (x >> 3U) ^ (x >> 4U)) & 1U) << 7U) | ((x >> 1U) & 0x7FU);
            }
            break;
    }

    return x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P);
}

static uint32_t pllEncodeM(uint32_t M)
{
    uint32_t i, x;

    /* Find MDec */
    switch (M)
    {
        case 0U:
            x = 0xFFFFFU;
            break;

        case 1U:
            x = 0x18003U;
            break;

        case 2U:
            x = 0x10003U;
            break;

        default:
            x = 0x04000U;
            for (i = M; i <= MVALMAX; i++)
            {
                x = (((x ^ (x >> 1U)) & 1U) << 14U) | ((x >> 1U) & 0x3FFFU);
            }
            break;
    }

    return x & (PLL_SSCG0_MDEC_VAL_M >> PLL_SSCG0_MDEC_VAL_P);
}

/* Find encoded PDEC value for raw P value, max P = PVALMAX */
//static uint32_t pllEncodeP(uint32_t P)
//{
//    uint32_t x, i;

//    /* Find PDec */
//    switch (P)
//    {
//        case 0U:
//            x = 0xFFU;
//            break;

//        case 1U:
//            x = 0x62U;
//            break;

//        case 2U:
//            x = 0x42U;
//            break;

//        default:
//            x = 0x10U;
//            for (i = P; i <= PVALMAX; i++)
//            {
//                x = (((x ^ (x >> 2U)) & 1U) << 4U) | ((x >> 1U) & 0xFU);
//            }
//            break;
//    }

//    return x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P);
//}


void SetFlashClock(uint32_t clk)
{
#if defined(SYSCON_FLASHCFG_FLASHTIM_MASK)
    if (clk <= 12000000U)
    {
        SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(0);
    }
    else if (clk <= 30000000U)
    {
        SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(1);
    }
    else if (clk <= 60000000U)
    {
        SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(2);
    }
    else if (clk <= 85000000U)
    {
        SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(3);
    }
    else
    {
        SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(8);
    }
#endif
}


void SetFROClock(uint32_t freq, bool val)
{
    uint32_t reg;
    
    if(val)
    {
        reg = SYSCON->FROCTRL;
        
        SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_FRO_MASK;
        
        reg |= SYSCON_FROCTRL_HSPDCLK_MASK | SYSCON_FROCTRL_USBCLKADJ_MASK;
        (freq == 48*1000*1000)?(reg &= ~SYSCON_FROCTRL_SEL_MASK):(reg |= SYSCON_FROCTRL_SEL_MASK);
        
        SYSCON->FROCTRL = reg;
    }
    else
    {
        SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK_MASK;
    }
}
    
void SetClockOut(uint32_t opt)
{
    /*
        0x0 Main clock (main_clk)
        0x1 CLKIN (clk_in)
        0x2 Watchdog oscillator (wdt_clk)
        0x3 FRO 96 or 48 MHz (fro_hf)
        0x4 PLL output (pll_clk)
        0x5 FRO 12 MHz (fro_12m)
        0x6 RTC oscillator 32 kHz output (32k_clk)
    */
    SYSCON->CLKOUTSELA = SYSCON_CLKOUTSELA_SEL(opt);
//    SYSCON->CLKOUTDIV &= ~SYSCON_CLKOUTDIV_HALT_MASK;
}

void SetExtOSC(uint32_t freq, bool val)
{
#if defined(SYSCON_SYSOSCCTRL_FREQRANGE_MASK)
    if(val)
    {
        (freq <= 20*1000*1000)?(SYSCON->SYSOSCCTRL &= ~SYSCON_SYSOSCCTRL_FREQRANGE_MASK):(SYSCON->SYSOSCCTRL |= SYSCON_SYSOSCCTRL_FREQRANGE_MASK);
        SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK;
        SYSCON->PDRUNCFG[1] &= ~SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
    }
    
#endif
    
}


/* Find encoded PDEC value for raw P value, max P = PVALMAX */
static uint32_t pllEncodeP(uint32_t P)
{
    uint32_t x, i;

    /* Find PDec */
    switch (P)
    {
        case 0U:
            x = 0xFFU;
            break;

        case 1U:
            x = 0x62U;
            break;

        case 2U:
            x = 0x42U;
            break;

        default:
            x = 0x10U;
            for (i = P; i <= PVALMAX; i++)
            {
                x = (((x ^ (x >> 2U)) & 1U) << 4U) | ((x >> 1U) & 0xFU);
            }
            break;
    }

    return x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P);
}


/* select 
    0x0 FRO 12 MHz (fro_12m)
    0x1 CLKIN (clk_in)
*/

uint32_t SetupSystemPLL(uint32_t opt, uint32_t out_clk)
{
    uint32_t SELR, SELI, SELP, in_clk;
    bool isFind = false;
    uint32_t M, m, N, n, P, Fcco, Fout;
    
    SYSCON->SYSPLLCLKSEL = SYSCON_SYSPLLCLKSEL_SEL(opt);
    
    if(opt == 0)
    {
        in_clk = GetClock(kFROLfClock);
    }

    if(opt == 1)
    {
        in_clk = GetClock(kExtOSCClock);
        SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_IOCON_MASK;
        IOCON->PIO[0][22] = IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK;
        SetExtOSC(in_clk, true);
    }

    /* power off PLL */
    #if defined(SYSCON_PDRUNCFG_PDEN_VD3_MASK)
    SYSCON->PDRUNCFG[0] |= (SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK | SYSCON_PDRUNCFG_PDEN_VD3_MASK);
    #else
    SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK;
    #endif
    
    #if defined(SYSCON_SYSPLLSSCTRL1_PD_MASK)
    SYSCON->SYSPLLSSCTRL1 |= SYSCON_SYSPLLSSCTRL1_PD_MASK;
    #endif
    
    /* n: pre divider, p: post div, m feedback */
    /* n:1-256, m:1-32768, p:1-32 */
    /* Fout = Fcco = 2 x M x Fin / N */
    for(m=1; m<=60; m++)
    {
        for(n=1; n<256; n++)
        {
            Fcco = 2 * m * in_clk/n;
            Fout = Fcco/2; /* P = 1 */

            if(Fout == out_clk && Fcco >= 60*1000*1000 && Fcco <= 500*1000*1000)
            {
                M = m;
                N = n;
                LIB_TRACE("m:%d, n:%d Fcco:%dHz\r\n", M, N, Fcco);
                isFind = true;
                goto FIND_SYS_PLL_PARAM;
            }
        }
    }

    if(isFind == false)
    {
        return CH_ERR;
    }
    
    FIND_SYS_PLL_PARAM:
    
    SELR = 0U;
    SELI = (M & 0x3cU) + 4U;
    SELP = (M >> 1U) + 1U;
    
    #if defined(SYSCON_SYSPLLCTRL_BANDSEL_MASK)
    SYSCON->SYSPLLCTRL = SYSCON_SYSPLLCTRL_BANDSEL_MASK | SYSCON_SYSPLLCTRL_SELR(SELR) | SYSCON_SYSPLLCTRL_SELP(SELP) | SYSCON_SYSPLLCTRL_SELI(SELI);
    #else
    SYSCON->SYSPLLCTRL =  SYSCON_SYSPLLCTRL_SELR(SELR) | SYSCON_SYSPLLCTRL_SELP(SELP) | SYSCON_SYSPLLCTRL_SELI(SELI);
    #endif

    #if defined(SYSCON_SYSPLLSSCTRL0_SEL_EXT_MASK)
    SYSCON->SYSPLLSSCTRL0 = SYSCON_SYSPLLSSCTRL0_SEL_EXT_MASK | pllEncodeM(M) | SYSCON_SYSPLLSSCTRL0_MREQ_MASK; /* M */
    #else
    SYSCON->SYSPLLMDEC = pllEncodeM(M) | SYSCON_SYSPLLMDEC_MREQ_MASK;
    #endif

    /* post div, div by 2 */
    P = 1;
    SYSCON->SYSPLLNDEC = SYSCON_SYSPLLNDEC_NREQ_MASK | pllEncodeN(N); /* N */
    SYSCON->SYSPLLPDEC = SYSCON_SYSPLLPDEC_PREQ_MASK | pllEncodeP(P); /* P */
        
    /* power up PLL */
    #if defined(SYSCON_PDRUNCFG_PDEN_VD3_MASK)
    SYSCON->PDRUNCFG[0] &= ~(SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK | SYSCON_PDRUNCFG_PDEN_VD3_MASK);
    #else
    SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK;
    #endif


    /* wait lock */
    while((SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCK_MASK) == 0);
    
    POWER_SetVoltageForFreq(Fout);
    SetFlashClock(Fout);
    
    /* select main clock to be PLL */
    SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(2);
     
    SystemCoreClockUpdate();
    SystemCoreClock = out_clk;
    
    return CH_OK;
}

/* USB PLL in clock fixed to exteral clk, USB PLL output clock is fixed to 48Mhz */
uint32_t SetupUSBPLL(void)
{
    #if defined(SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK)
    uint32_t M, m, N, n, p, P, Fcco, Fout, in_clk;
    
    Fout = 48*1000*1000;
    
    in_clk = GetClock(kExtOSCClock);
    SYSCON->PDRUNCFG[1] |= SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
    
    for(m=1; m<=256; m++)
    {
        for(n=1; n<4; n++)
        {
            for(p=1; p<8; p++)
            {
                Fcco = Fout*(2*p);
                if((in_clk / n) * m == Fcco)
                {
                    M = m;
                    N = n;
                    P = p;
                    LIB_TRACE("m:%d, n:%d p:%d Fcco:%dHz\r\n", M, N, P, Fcco);
                    goto FIND_USB_PLL_PARAM;
                }
                
            }
        }
    }
    
    FIND_USB_PLL_PARAM:
    SYSCON->USBPLLCTRL = SYSCON_USBPLLCTRL_MSEL(M-1) | SYSCON_USBPLLCTRL_PSEL(P-1) | SYSCON_USBPLLCTRL_NSEL(N-1);
    SYSCON->PDRUNCFG[1] &= ~SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
    
    /* wait lock */
    while((SYSCON->USBPLLSTAT & SYSCON_USBPLLSTAT_LOCK_MASK) == 0);
    #endif
    return 0;
}
