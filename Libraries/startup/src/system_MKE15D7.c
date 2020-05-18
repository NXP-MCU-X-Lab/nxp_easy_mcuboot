/*
** ###################################################################
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K15 Sub-Family Reference Manual Rev. 1, 9 Feb 2012
**     Version:             rev. 2.0, 2012-02-20
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2012 Freescale Semiconductor, Inc. All Rights Reserved.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2012-01-15)
**         Initial public version.
**     - rev. 2.0 (2012-02-20)
**         Update according to the Rev 1. reference manual.
**
** ###################################################################
*/

/**
 * @file MKE15D7
 * @version 2.0
 * @date 2012-02-20
 * @brief Device specific configuration file for MKE15D7 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MKE15D7.h"

  #define DEFAULT_SYSTEM_CLOCK            20971520u /* Default System clock value */
  #define DISABLE_WDOG                            1

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if (DISABLE_WDOG)  // 15.02.2012 Added by ARM 
  WDOG->UNLOCK = 0xC520;                               /* Write 0xC520 to the unlock register */
  WDOG->UNLOCK = 0xD928;                               /* Followed by 0xD928 to complete the unlock */
  WDOG->STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;          /* Clear the WDOGEN bit to disable the watchdog */
#endif /* (DISABLE_WDOG) */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
}
