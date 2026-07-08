/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*${header:start}*/
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
/*${header:end}*/

/*${function:start}*/
void BOARD_InitHardware(void)
{
    BOARD_InitDEBUG_UARTPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    
    /* Enable internal flash RWX */
    MBC->MBC_INDEX[0].MBC_MEMN_GLBAC[0] = 0x00007777;
    MBC->MBC_INDEX[0].MBC_DOM0_MEM0_BLK_CFG_W[0] = 0x00000000;
}
/*${function:end}*/
