/*
** ###################################################################
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    MKE04Z24M48SF0RM, Rev.1, May-23 2013
**     Version:             rev. 1.0, 2013-05-09
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
**     - rev. 1.0 (2013-05-09)
**         Initial version.
**
**     Added DISABLE_WDOG part (by ARM)
** ###################################################################
*/

/*!
 * @file MKE04Z4
 * @version 1.0
 * @date 2013-05-09
 * @brief Device specific configuration file for MKE04Z4 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MKE04Z4.h"

  #define DISABLE_WDOG                    1

  #define DEFAULT_SYSTEM_CLOCK            20971520u /* Default System clock value */


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */
#define WDOG_TOVAL                               *((volatile uint16_t *)(&WDOG->TOVALH))
#define WDOG_CNT                                 *((volatile uint16_t *)(&WDOG->CNTH))

void SystemInit (void) {
#if (DISABLE_WDOG)
  /* Disable the WDOG module */
  WDOG_CNT   = 0x20C5; 
  WDOG_CNT   = 0x28D9;
  WDOG_TOVAL = 0xFFFF;
    //WDOG_WIN = 0; /* NOTE: this will cause HARDFAULT error*/
  WDOG->WINH = 0xFF;
  WDOG->WINL = 0xFF;

  WDOG->CS2 = 0x01;
  WDOG->CS1 = 0x20; /* WDOGA = 1 to allow reconfigure watchdog at any time by executing an unlock sequence */
#endif
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
}
