/*
** ###################################################################
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    MKE02Z64M20SF0RM, Rev.2.1, Apr-23 2013; KEAZ64RM, Rev.1, Sep 2013
**     Version:             rev. 1.0, 2013-12-19
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2013 Freescale, Inc. All Rights Reserved.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2013-12-19)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MKE02Z4
 * @version 1.0
 * @date 2013-12-19
 * @brief Device specific configuration file for MKE02Z4 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MKE02Z4.h"


#define DISABLE_WDOG    1

#define CLOCK_SETUP     1
/* Predefined clock setups
   0 ... Internal Clock Source (ICS) in FLL Engaged Internal (FEI) mode
         Default  part configuration.
         Reference clock source for ICS module is the slow internal clock source 32.768kHz
         Core clock = 16.78MHz, BusClock = 16.78MHz
   1 ... Internal Clock Source (ICS) in FLL Engaged External (FEE) mode
         Maximum achievable clock frequency configuration.
         Reference clock source for ICS module is an external 8MHz crystal
         Core clock = 20MHz, BusClock = 20MHz
   2 ... Internal Clock Source (ICS) in Bypassed Low Power Internal (FBILP) mode
         Core clock/Bus clock derived directly from an  internal clock 32.769kHz with no multiplication
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 32.769kHz, BusClock = 32.769kHz
   3 ... Internal Clock Source (ICS) in Bypassed Low Power External (BLPE) mode
         Core clock/Bus clock derived directly from the external 8MHz crystal
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 8MHz, BusClock = 8MHz
*/

/*----------------------------------------------------------------------------
  Define clock source values
 *----------------------------------------------------------------------------*/
#if (CLOCK_SETUP == 0)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_CLK_HZ                  32768u   /* Value of the internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            16777216u /* Default System clock value */
#elif (CLOCK_SETUP == 1)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_CLK_HZ                  32768u   /* Value of the internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            20000000u /* Default System clock value */
#elif (CLOCK_SETUP == 2)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_CLK_HZ                  32768u   /* Value of the internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            32768u   /* Default System clock value */
#elif (CLOCK_SETUP == 3)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_CLK_HZ                  32768u   /* Value of the internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            8000000u /* Default System clock value */
#endif /* (CLOCK_SETUP == 4) */



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {

#if (DISABLE_WDOG)
  /* WDOG->TOVAL: TOVAL=0xE803 */
  WDOG->TOVAL = WDOG_TOVAL_TOVAL(0xE803); /* Timeout value */
  /* WDOG->CS2: WIN=0,FLG=0,??=0,PRES=0,??=0,??=0,CLK=1 */
  WDOG->CS2 = WDOG_CS2_CLK(0x01);       /* 1-kHz clock source */
  /* WDOG->CS1: EN=0,INT=0,UPDATE=1,TST=0,DBG=0,WAIT=1,STOP=1 */
  WDOG->CS1 = WDOG_CS1_UPDATE_MASK |
             WDOG_CS1_TST(0x00) |
             WDOG_CS1_WAIT_MASK |
             WDOG_CS1_STOP_MASK;

#endif /* (DISABLE_WDOG) */
#if (CLOCK_SETUP == 0)
  /* ICS->C2: BDIV|=1 */
  ICS->C2 |= ICS_C2_BDIV(0x01);         /* Update system prescalers */
  /* SIM->BUSDIV: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,BUSDIV=0 */
  SIM->BUSDIV = 0x00U;                  /* Update system prescalers */
  /* Switch to FEI Mode */
  /* ICS->C1: CLKS=0,RDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  ICS->C1 = ICS_C1_CLKS(0x00) |
           ICS_C1_RDIV(0x00) |
           ICS_C1_IREFS_MASK |
           ICS_C1_IRCLKEN_MASK;
  /* ICS->C2: BDIV=1,LP=0 */
  ICS->C2 = (uint8_t)((ICS->C2 & (uint8_t)~(uint8_t)(
            ICS_C2_BDIV(0x06) |
            ICS_C2_LP_MASK
           )) | (uint8_t)(
            ICS_C2_BDIV(0x01)
           ));
  /* OSC->CR: OSCEN=0,??=0,OSCSTEN=0,OSCOS=0,??=0,RANGE=0,HGO=0,OSCINIT=0 */
  OSC->CR = 0x00U;
  while((ICS->S & ICS_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((ICS->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
  }
#elif (CLOCK_SETUP == 1)
  /* ICS->C2: BDIV|=1 */
  ICS->C2 |= ICS_C2_BDIV(0x01);         /* Update system prescalers */
  /* SIM->BUSDIV: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,BUSDIV=0 */
  SIM->BUSDIV = 0x00U;                  /* Update system prescalers */
  /* Switch to FEE Mode */
  /* ICS->C2: BDIV=1,LP=0 */
  ICS->C2 = (uint8_t)((ICS->C2 & (uint8_t)~(uint8_t)(
            ICS_C2_BDIV(0x06) |
            ICS_C2_LP_MASK
           )) | (uint8_t)(
            ICS_C2_BDIV(0x01)
           ));
  /* OSC->CR: OSCEN=1,??=0,OSCSTEN=0,OSCOS=1,??=0,RANGE=1,HGO=0,OSCINIT=0 */
  OSC->CR = (OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_RANGE_MASK);
  /* ICS->C1: CLKS=0,RDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  ICS->C1 = (ICS_C1_CLKS(0x00) | ICS_C1_RDIV(0x03) | ICS_C1_IRCLKEN_MASK);
  while((ICS->S & ICS_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((ICS->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
  }
#elif (CLOCK_SETUP == 2)
  /* SIM->BUSDIV: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,BUSDIV=0 */
  SIM->BUSDIV = 0x00U;                  /* Update system prescalers */
  /* Switch to FBI Mode */
  /* ICS->C1: CLKS=1,RDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  ICS->C1 = ICS_C1_CLKS(0x01) |
           ICS_C1_RDIV(0x00) |
           ICS_C1_IREFS_MASK |
           ICS_C1_IRCLKEN_MASK;
  /* ICS->C2: BDIV=0,LP=0 */
  ICS->C2 &= (uint8_t)~(uint8_t)((ICS_C2_BDIV(0x07) | ICS_C2_LP_MASK));
  /* OSC->CR: OSCEN=0,??=0,OSCSTEN=0,OSCOS=0,??=0,RANGE=0,HGO=0,OSCINIT=0 */
  OSC->CR = 0x00U;
  while((ICS->S & ICS_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((ICS->S & 0x0CU) != 0x04U) {    /* Wait until internal reference clock is selected as ICS output */
  }
  /* Switch to BLPI Mode */
  /* ICS->C2: BDIV=0,LP=1 */
  ICS->C2 = (uint8_t)((ICS->C2 & (uint8_t)~(uint8_t)(
            ICS_C2_BDIV(0x07)
           )) | (uint8_t)(
            ICS_C2_LP_MASK
           ));
  while((ICS->S & ICS_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
#elif (CLOCK_SETUP == 3)
  /* SIM->BUSDIV: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,BUSDIV=0 */
  SIM->BUSDIV = 0x00U;                  /* Update system prescalers */
  /* Switch to FBE Mode */
  /* ICS->C2: BDIV=0,LP=0 */
  ICS->C2 &= (uint8_t)~(uint8_t)((ICS_C2_BDIV(0x07) | ICS_C2_LP_MASK));
  /* OSC->CR: OSCEN=1,??=0,OSCSTEN=0,OSCOS=1,??=0,RANGE=1,HGO=0,OSCINIT=0 */
  OSC->CR = (OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_RANGE_MASK);
  /* ICS->C1: CLKS=2,RDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  ICS->C1 = (ICS_C1_CLKS(0x02) | ICS_C1_RDIV(0x03) | ICS_C1_IRCLKEN_MASK);
  while((ICS->S & ICS_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((ICS->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as ICS output */
  }
  /* Switch to BLPE Mode */
  /* ICS->C2: BDIV=0,LP=1 */
  ICS->C2 = (uint8_t)((ICS->C2 & (uint8_t)~(uint8_t)(
            ICS_C2_BDIV(0x07)
           )) | (uint8_t)(
            ICS_C2_LP_MASK
           ));
  while((ICS->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as ICS output */
  }
#endif

}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t ICSOUTClock;                                                        /* Variable to store output clock frequency of the ICS module */
  uint8_t Divider;

  if ((ICS->C1 & ICS_C1_CLKS_MASK) == 0x0u) {
    /* Output of FLL is selected */
    if ((ICS->C1 & ICS_C1_IREFS_MASK) == 0x0u) {
      /* External reference clock is selected */
      ICSOUTClock = CPU_XTAL_CLK_HZ;                                         /* System oscillator drives ICS clock */
      Divider = (uint8_t)(1u << ((ICS->C1 & ICS_C1_RDIV_MASK) >> ICS_C1_RDIV_SHIFT));
      ICSOUTClock = (ICSOUTClock / Divider);  /* Calculate the divided FLL reference clock */
      if ((OSC->CR & OSC_CR_RANGE_MASK) != 0x0u) {
        ICSOUTClock /= 32u;                                                  /* If high range is enabled, additional 32 divider is active */
      }
    } else {
      ICSOUTClock = CPU_INT_CLK_HZ;                                          /* The internal reference clock is selected */
    }
    ICSOUTClock *= 1280u;                                                    /* Apply 1280 FLL multiplier */
  } else if ((ICS->C1 & ICS_C1_CLKS_MASK) == 0x40u) {
    /* Internal reference clock is selected */
    ICSOUTClock = CPU_INT_CLK_HZ;
  } else if ((ICS->C1 & ICS_C1_CLKS_MASK) == 0x80u) {
    /* External reference clock is selected */
    ICSOUTClock = CPU_XTAL_CLK_HZ;
  } else {
    /* Reserved value */
    return;
  }
  ICSOUTClock = ICSOUTClock >> ((ICS->C2 & ICS_C2_BDIV_MASK) >> ICS_C2_BDIV_SHIFT);
  SystemCoreClock = (ICSOUTClock / (1u + ((SIM->BUSDIV & SIM_BUSDIV_BUSDIV_MASK) >> SIM_BUSDIV_BUSDIV_SHIFT)));

}
