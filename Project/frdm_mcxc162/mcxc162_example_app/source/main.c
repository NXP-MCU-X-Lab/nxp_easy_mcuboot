/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>

#include "app.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "pin_mux.h"

#define APP_SLOT_BASE       (0x00004000u)
#define APP_UART_BAUDRATE   (115200u)
#define APP_SYSTICK_MS      (10u)
#define APP_LED_PERIOD_MS   (500u)
#define APP_LED_TICK_COUNT  (APP_LED_PERIOD_MS / APP_SYSTICK_MS)

static volatile uint32_t s_led_ticks;

static void app_uart_write(const char *text)
{
    while (*text != '\0')
    {
        uint8_t ch = (uint8_t)*text++;
        (void)LPUART_WriteBlocking(LPUART0, &ch, 1u);
    }
}

static void app_uart_init(void)
{
    lpuart_config_t config;

    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(kFRO_LF_DIV_to_LPUART0);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = APP_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    (void)LPUART_Init(LPUART0, &config, CLOCK_GetLpuartClkFreq(0u));
}

static void app_start_systick(void)
{
    uint32_t reload = (CLOCK_GetCoreSysClkFreq() / 1000u) * APP_SYSTICK_MS;

    if (SysTick_Config(reload) != 0u)
    {
        app_uart_write("systick failed\r\n");
        while (1)
        {
        }
    }
}

void SysTick_Handler(void)
{
    s_led_ticks++;
    if (s_led_ticks >= APP_LED_TICK_COUNT)
    {
        s_led_ticks = 0u;
        LED_RED_TOGGLE();
    }
}

int main(void)
{
    BOARD_InitHardware();
    app_uart_init();

    app_uart_write("MCXC162 app\r\n");
    app_uart_write("slot=0x00004000\r\n");

    app_start_systick();

    while (1)
    {
        __WFI();
    }
}
