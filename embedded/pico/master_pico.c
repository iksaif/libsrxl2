/*
 * SRXL2 Bus Master for Raspberry Pi Pico (PIO UART)
 *
 * Runs the full libsrxl2 state machine as bus master: performs handshake,
 * sends channel data every 11ms, polls slaves for telemetry, and prints
 * decoded telemetry over USB CDC.
 *
 * Uses PIO for true single-pin half-duplex UART:
 *   - SM0 = RX (runs when listening)
 *   - SM1 = TX (enabled only during transmit)
 *   - RX is disabled during TX, so no echo and no guard timer needed.
 *
 * Wiring:
 *   GPIO 0              -> SRXL2 bus data line (single wire)
 *   GND                 -> SRXL2 bus ground
 *   USB                 -> Host computer (telemetry output)
 *   External 5V         -> Slave VCC
 *
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"

/* PIO half-duplex UART HAL (shared) */
#include "pio_uart.pio.h"
#include "pio_uart_hal.h"

#include "board_config.h"

#define LED_PIN         PICO_DEFAULT_LED_PIN

static pio_uart_t pio_uart;

/* Static allocation for srxl2 context (no malloc) */
static uint8_t ctx_buf[sizeof(srxl2_ctx_t)] __attribute__((aligned(4)));
static srxl2_ctx_t *ctx;

/* Telemetry print rate limiting */
static uint32_t last_telem_print_ms;
#define TELEM_PRINT_INTERVAL_MS  500

/* Status print rate */
static uint32_t last_status_ms;
#define STATUS_INTERVAL_MS  5000

/*---------------------------------------------------------------------------
 * HAL callbacks
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    pio_uart_send(&pio_uart, buf, len);
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    pio_uart_set_baud(&pio_uart, baud);
}

static uint32_t hal_time_ms(void *user)
{
    (void)user;
    return (uint32_t)(time_us_64() / 1000);
}

/*---------------------------------------------------------------------------
 * Telemetry print helper
 *---------------------------------------------------------------------------*/

static void print_telem_decoded(const srxl2_telem_decoded_t *d)
{
    switch (d->type) {
    case SRXL2_TELEM_TYPE_ESC:
        printf("ESC: %.0fRPM %.2fV %.2fA %.1fC",
               d->esc.rpm, d->esc.voltage,
               d->esc.current, d->esc.temp_fet);
        break;
    case SRXL2_TELEM_TYPE_FP_MAH:
        printf("FP: %.1fA %.0fmAh",
               d->fp_mah.current_a, d->fp_mah.charge_used_a);
        break;
    case SRXL2_TELEM_TYPE_SMART_BAT_REALTIME:
        printf("BAT: %.2fA %.1fmAh %.3f-%.3fV",
               d->smart_bat_realtime.current,
               d->smart_bat_realtime.consumption,
               d->smart_bat_realtime.min_cell,
               d->smart_bat_realtime.max_cell);
        break;
    case SRXL2_TELEM_TYPE_LIPOMON: {
        printf("LIPO:");
        for (int i = 0; i < 6; i++) {
            float v = d->lipomon.cell[i];
            if (v == v) /* not NAN */
                printf(" %.2fV", v);
        }
        break;
    }
    case SRXL2_TELEM_TYPE_RPM:
        printf("RPM: %.0f %.2fV", d->rpm.rpm, d->rpm.voltage);
        break;
    default:
        printf("%s", srxl2_telem_type_name(d->type));
        break;
    }
}

/*---------------------------------------------------------------------------
 * Event callback
 *---------------------------------------------------------------------------*/

static void on_event(srxl2_ctx_t *c, const srxl2_event_t *evt, void *user)
{
    (void)c;
    (void)user;

    switch (evt->type) {
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        printf("[M] Handshake done, %u peer(s)\n", evt->handshake.peer_count);
        break;

    case SRXL2_EVT_TELEMETRY: {
        uint32_t now = hal_time_ms(NULL);
        if (now - last_telem_print_ms < TELEM_PRINT_INTERVAL_MS)
            break;
        last_telem_print_ms = now;

        srxl2_telem_decoded_t decoded;
        if (srxl2_decode_telemetry(evt->telemetry.payload, &decoded)) {
            printf("[T 0x%02X] ", evt->telemetry.device_id);
            print_telem_decoded(&decoded);
            printf("\n");
        }
        break;
    }

    case SRXL2_EVT_TIMEOUT:
        printf("[M] Timeout\n");
        break;

    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

int main(void)
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* Init PIO UART on single pin */
    pio_uart_init(&pio_uart, pio0, SRXL2_PIN, SRXL2_BAUD_INIT);

    /* Wait for USB */
    uint32_t start = (uint32_t)(time_us_64() / 1000);
    while (!stdio_usb_connected() && (uint32_t)(time_us_64() / 1000) - start < 3000)
        tight_loop_contents();

    printf("\n=== SRXL2 Pico Master ===\n");
    printf("PIO UART on GPIO %d @ %u baud, single-wire half-duplex\n",
           SRXL2_PIN, SRXL2_BAUD_INIT);

    /* Init libsrxl2 as master */
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = {
            .device_id = 0x10,
            .priority  = 20,
            .info      = SRXL2_DEVINFO_TELEM_TX_ENABLED,
            .uid       = 0xC0DE0001,
        },
        .hal = {
            .uart_send     = hal_uart_send,
            .uart_set_baud = hal_uart_set_baud,
            .time_ms       = hal_time_ms,
            .user          = NULL,
        },
        .baud_supported = SRXL2_BAUD_115200,
    };

    ctx = srxl2_init_static(ctx_buf, sizeof(ctx_buf), &cfg);
    if (!ctx) {
        printf("ERR: srxl2_init failed\n");
        while (true) tight_loop_contents();
    }
    srxl2_on_event(ctx, on_event, NULL);

    /* Channels at center */
    uint16_t channels[32] = {0};
    for (int i = 0; i < 16; i++)
        channels[i] = 32768;

    printf("Running...\n\n");

    last_telem_print_ms = 0;
    last_status_ms = 0;

    while (true) {
        /* Read PIO RX FIFO -- no echo guard needed with PIO */
        while (pio_uart_readable(&pio_uart)) {
            uint8_t byte = pio_uart_getc(&pio_uart);
            srxl2_feed(ctx, &byte, 1);
        }

        /* Tick state machine */
        srxl2_tick(ctx);

        /* Keep channel data fresh */
        if (srxl2_is_connected(ctx)) {
            srxl2_set_channels(ctx, channels, 0x0000FFFF);
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        /* Periodic status */
        uint32_t now = hal_time_ms(NULL);
        if (now - last_status_ms >= STATUS_INTERVAL_MS) {
            last_status_ms = now;
            printf("[M] state=%s peers=%u\n",
                   srxl2_get_state(ctx), srxl2_peer_count(ctx));
        }

        tight_loop_contents();
    }
}
