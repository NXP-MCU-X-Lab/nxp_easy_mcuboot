/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include <stdio.h>
#include <string.h>

#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_lpuart.h"



static void app_clk_init(void)
{
    // Enable clocks for ports and GPIOs
    CLOCK_EnableClock(kCLOCK_GatePORT0);
    CLOCK_EnableClock(kCLOCK_GatePORT1);
    CLOCK_EnableClock(kCLOCK_GatePORT2);
    CLOCK_EnableClock(kCLOCK_GatePORT3);
    CLOCK_EnableClock(kCLOCK_GateGPIO0);
    CLOCK_EnableClock(kCLOCK_GateGPIO1);
    CLOCK_EnableClock(kCLOCK_GateGPIO2);
    CLOCK_EnableClock(kCLOCK_GateGPIO3);
    
    // Release peripheral resets
    RESET_ReleasePeripheralReset(kPORT0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT2_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT3_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO2_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kGPIO3_RST_SHIFT_RSTn);
    
    RESET_ReleasePeripheralReset(kLPUART0_RST_SHIFT_RSTn);
    
    // Initialize GPIO
    PORT_SetPinMux(PORT3, 12, kPORT_MuxAsGpio);
    
    gpio_pin_config_t gpio_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0
    };
    GPIO_PinInit(GPIO3, 12, &gpio_config);
    
    // Configure UART pins
    PORT_SetPinMux(PORT0, 2, kPORT_MuxAlt2);
    PORT_SetPinMux(PORT0, 3, kPORT_MuxAlt2);
    PORT0->PCR[2] |= PORT_PCR_IBE_MASK;
    PORT0->PCR[3] |= PORT_PCR_IBE_MASK;
    
    // Configure UART clock
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPUART0);
}

int main(void)
{
    uint8_t c;

    app_clk_init();

    lpuart_config_t config;
    
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(LPUART0, &config, CLOCK_GetLpuartClkFreq(0));

    printf("I am application, main address:0x%X\r\n", main);
    
    while(1)
    {
        GPIO_PortToggle(GPIO3, 1<<12);
        SDK_DelayAtLeastUs(100*1000, CLOCK_GetCoreSysClkFreq());
    }
    
    while(1)
    {

    }
    
}


void SysTick_Handler(void)
{

}


void HardFault_Handler(void)
{
    NVIC_SystemReset();
    
    while(1)
    {
    }
}


void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
    
}



#pragma weak __stdout
#pragma weak __stdin
FILE __stdout;
FILE __stdin;

#pragma weak fputc
int fputc(int ch, FILE *f)
{
    LPUART_WriteByte(LPUART0, (uint8_t)ch);
    
    while (!(LPUART0->STAT & LPUART_STAT_TDRE_MASK))
    {
    }
    
    return ch;
}

