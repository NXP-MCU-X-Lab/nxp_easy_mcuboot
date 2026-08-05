/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "app_version.h"
#include "bl_trial.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "fsl_wwdt.h"
#include "pin_mux.h"

#define APP_SLOT_BASE            (0x00004000u)
#define APP_IMAGE_MAGIC          (0x4D435831u)
#define APP_IMAGE_HEADER_OFFSET  (0x200u)
#define APP_IMAGE_HEADER_ADDR    (APP_SLOT_BASE + APP_IMAGE_HEADER_OFFSET)
#define APP_IMAGE_HEADER_VER     (1u)
#define APP_IMAGE_HEADER_SIZE    (64u)
#define APP_UART_BAUDRATE        (115200u)
#define APP_SYSTICK_MS           (10u)
#define APP_LED_PERIOD_MS        (500u)
#define APP_LED_TICK_COUNT       (APP_LED_PERIOD_MS / APP_SYSTICK_MS)
#define APP_LOG_PERIOD_MS        (1000u)
#define APP_LOG_TICK_COUNT       (APP_LOG_PERIOD_MS / APP_SYSTICK_MS)
#define APP_WWDT_TIMEOUT_MS      (5000u)
#define APP_WWDT_FEED_PERIOD_MS  (1000u)
#define APP_WWDT_FEED_TICK_COUNT (APP_WWDT_FEED_PERIOD_MS / APP_SYSTICK_MS)
#define APP_TRIAL_HEALTHY_MS     (30000u)
#define APP_TRIAL_HEALTHY_TICKS  (APP_TRIAL_HEALTHY_MS / APP_SYSTICK_MS)
#define APP_WWDT_CLOCK_DIVISOR   (4u)

typedef struct
{
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t image_version;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t flags;
    uint32_t reserved[10];
} app_image_header_t;

static volatile uint32_t s_led_ticks;
static volatile uint32_t s_log_ticks;
static volatile uint32_t s_wwdt_feed_ticks;
static volatile uint32_t s_trial_healthy_ticks;
static volatile bool s_log_pending;
static volatile bool s_wwdt_feed_pending;
static volatile bool s_trial_healthy_pending;
static uint32_t s_log_counter;

static void app_uart_write(const char *text)
{
    while (*text != '\0')
    {
        uint8_t ch = (uint8_t)*text++;
        (void)LPUART_WriteBlocking(LPUART0, &ch, 1u);
    }
}

static void app_uart_write_hex32(uint32_t value)
{
    char text[11];
    uint32_t i;

    text[0] = '0';
    text[1] = 'x';
    for (i = 0u; i < 8u; i++)
    {
        uint32_t digit = (value >> ((7u - i) * 4u)) & 0xFu;
        text[2u + i] = (char)((digit < 10u) ? ('0' + digit) : ('A' + digit - 10u));
    }
    text[10] = '\0';

    app_uart_write(text);
}

static void app_uart_write_kv_hex32(const char *key, uint32_t value)
{
    app_uart_write(key);
    app_uart_write_hex32(value);
    app_uart_write("\r\n");
}

static void app_print_banner(void)
{
    const app_image_header_t *header = (const app_image_header_t *)APP_IMAGE_HEADER_ADDR;

    app_uart_write("================================\r\n");
    app_uart_write("MCXC162 Example App\r\n");
    app_uart_write_kv_hex32("slot=", APP_SLOT_BASE);
    app_uart_write_kv_hex32("header=", APP_IMAGE_HEADER_ADDR);

    if ((header->magic == APP_IMAGE_MAGIC) &&
        (header->header_version == APP_IMAGE_HEADER_VER) &&
        (header->header_size == APP_IMAGE_HEADER_SIZE))
    {
        app_uart_write_kv_hex32("version=", header->image_version);
        app_uart_write_kv_hex32("size=", header->image_size);
        app_uart_write_kv_hex32("crc=", header->image_crc32);
    }
    else
    {
        app_uart_write("image=unpatched\r\n");
        app_uart_write_kv_hex32("build_version=", APP_IMAGE_VERSION);
    }

    app_uart_write("================================\r\n");
}

static void app_print_counter(void)
{
    s_log_counter++;
    app_uart_write_kv_hex32("MCXC162 app counter=", s_log_counter);
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

static void app_wwdt_init(void)
{
    wwdt_config_t config;
    uint32_t clock_hz = CLOCK_GetWwdtClkFreq();

    WWDT_GetDefaultConfig(&config);
    config.enableWatchdogReset = true;
    config.timeoutValue = (clock_hz / APP_WWDT_CLOCK_DIVISOR) * (APP_WWDT_TIMEOUT_MS / 1000u);
    config.clockFreq_Hz = clock_hz;
    WWDT_Init(WWDT0, &config);
}

void SysTick_Handler(void)
{
    s_led_ticks++;
    if (s_led_ticks >= APP_LED_TICK_COUNT)
    {
        s_led_ticks = 0u;
        LED_RED_TOGGLE();
    }

    s_log_ticks++;
    if (s_log_ticks >= APP_LOG_TICK_COUNT)
    {
        s_log_ticks = 0u;
        s_log_pending = true;
    }

    s_wwdt_feed_ticks++;
    if (s_wwdt_feed_ticks >= APP_WWDT_FEED_TICK_COUNT)
    {
        s_wwdt_feed_ticks = 0u;
        s_wwdt_feed_pending = true;
    }

    if (s_trial_healthy_ticks < APP_TRIAL_HEALTHY_TICKS)
    {
        s_trial_healthy_ticks++;
        if (s_trial_healthy_ticks == APP_TRIAL_HEALTHY_TICKS)
        {
            s_trial_healthy_pending = true;
        }
    }
}

int main(void)
{
    const app_image_header_t *header = (const app_image_header_t *)APP_IMAGE_HEADER_ADDR;
    uint32_t image_crc32 = header->image_crc32;

    BOARD_InitHardware();
    app_uart_init();

    app_print_banner();
    app_uart_write("Send F to simulate WWDT failure\r\n");

    app_start_systick();
    app_wwdt_init();

    while (1)
    {
        if ((LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag) != 0u)
        {
            uint8_t ch = LPUART_ReadByte(LPUART0);
            if (ch == (uint8_t)'F')
            {
                BL_TrialEnableDemoFault(BL_TRIAL_STATE, image_crc32);
                app_uart_write("WWDT fault enabled\r\n");
            }
        }

        if (s_log_pending)
        {
            s_log_pending = false;
            app_print_counter();
        }

        if (s_wwdt_feed_pending)
        {
            s_wwdt_feed_pending = false;
            if (!BL_TrialDemoFaultEnabled(BL_TRIAL_STATE, image_crc32))
            {
                WWDT_Refresh(WWDT0);
            }
        }

        if (s_trial_healthy_pending)
        {
            s_trial_healthy_pending = false;
            if (!BL_TrialDemoFaultEnabled(BL_TRIAL_STATE, image_crc32) &&
                BL_TrialMarkHealthy(BL_TRIAL_STATE, image_crc32))
            {
                app_uart_write("Trial healthy\r\n");
            }
        }
    }
}
