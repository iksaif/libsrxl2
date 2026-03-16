/*
 * SRXL2 Battery Sensor Simulator for Raspberry Pi Pico (PIO UART)
 *
 * Simulates a Spektrum Smart Battery sensor (device type 0xB0) using
 * libsrxl2. Responds to handshake, sends Flight Pack MAH (0x34)
 * telemetry when polled via pull-model.
 *
 * Simulates a 4S LiPo: voltage sag, current drain with sine-wave
 * variation, temperature rise proportional to capacity used.
 *
 * Uses PIO for true single-pin half-duplex UART:
 *   - SM0 = RX (runs when listening)
 *   - SM1 = TX (enabled only during transmit)
 *   - RX is disabled during TX, so no echo and no guard timer needed.
 *
 * Wiring:
 *   GPIO 0              -> SRXL2 bus data line (single wire)
 *   GND                 -> SRXL2 bus ground
 *   USB                 -> Host computer (debug output via CDC)
 *   External 5V         -> Master / Receiver VCC
 *
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
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

#define SRXL2_PIN       0       /* Single data pin (GPIO 0) */
#define SRXL2_BAUD_INIT 115200

#define LED_PIN         PICO_DEFAULT_LED_PIN

static pio_uart_t pio_uart;

/* Static allocation for srxl2 context (no malloc) */
static uint8_t ctx_buf[sizeof(srxl2_ctx_t)] __attribute__((aligned(4)));
static srxl2_ctx_t *ctx;

/* Status print rate */
static uint32_t last_status_ms;
#define STATUS_INTERVAL_MS  5000

/* Battery update rate */
static uint32_t last_batt_update_ms;
#define BATT_UPDATE_INTERVAL_MS  100

/*---------------------------------------------------------------------------
 * Battery simulation state
 *---------------------------------------------------------------------------*/

static struct {
    float voltage;          /* Volts */
    float current;          /* Amps */
    float capacity_used;    /* mAh */
    float capacity_total;   /* mAh */
    float temperature;      /* Celsius */
    uint32_t time_ms;       /* simulated uptime */
} battery = {
    .voltage        = 16.8f,    /* 4S LiPo fully charged */
    .current        = 5.0f,
    .capacity_used  = 0.0f,
    .capacity_total = 5000.0f,
    .temperature    = 25.0f,
    .time_ms        = 0,
};

static void update_battery(uint32_t delta_ms)
{
    battery.time_ms += delta_ms;

    float hours = delta_ms / (1000.0f * 3600.0f);
    battery.capacity_used += battery.current * hours * 1000.0f;

    float remaining = 1.0f - (battery.capacity_used / battery.capacity_total);
    if (remaining < 0.0f) remaining = 0.0f;

    battery.voltage = 12.0f + (remaining * 4.8f)
                    + sinf(battery.time_ms / 1000.0f) * 0.1f;

    battery.current = 5.0f + sinf(battery.time_ms / 500.0f) * 1.0f;
    if (battery.current < 0.0f) battery.current = 0.0f;

    battery.temperature = 25.0f
        + (battery.capacity_used / battery.capacity_total) * 15.0f;
}

/*---------------------------------------------------------------------------
 * Build FP_MAH telemetry payload using encoder API
 *---------------------------------------------------------------------------*/

static void build_fp_mah_payload(uint8_t payload[16])
{
    srxl2_telem_fp_mah_t data = {
        .current_a     = battery.current,
        .charge_used_a = battery.capacity_used,
        .temp_a        = battery.temperature,
        .current_b     = 0.0f,
        .charge_used_b = 0.0f,
        .temp_b        = NAN,
        .s_id          = 0x00,
    };
    srxl2_encode_fp_mah(payload, &data);
}

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
 * Event callback
 *---------------------------------------------------------------------------*/

static void on_event(srxl2_ctx_t *c, const srxl2_event_t *evt, void *user)
{
    (void)user;

    switch (evt->type) {
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        printf("[BAT] Handshake done, %u peer(s)\n", evt->handshake.peer_count);
        break;

    case SRXL2_EVT_TELEM_REQUEST: {
        /* Pull-model: fill telemetry just-in-time when polled */
        uint8_t payload[16];
        build_fp_mah_payload(payload);
        srxl2_set_telemetry(c, payload);
        break;
    }

    case SRXL2_EVT_TIMEOUT:
        printf("[BAT] Connection lost\n");
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

    /* Wait for USB CDC */
    uint32_t start = (uint32_t)(time_us_64() / 1000);
    while (!stdio_usb_connected() && (uint32_t)(time_us_64() / 1000) - start < 3000)
        tight_loop_contents();

    printf("\n=== SRXL2 Pico Battery Sensor ===\n");
    printf("Device ID: 0xB0 (Sensor)\n");
    printf("PIO UART on GPIO %d @ %u baud, single-wire half-duplex\n",
           SRXL2_PIN, SRXL2_BAUD_INIT);
    printf("Battery: 4S LiPo, %.0f mAh\n", battery.capacity_total);
    printf("Telemetry: FP_MAH (0x34)\n\n");

    /* Init libsrxl2 as SLAVE with Sensor device ID */
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_SLAVE,
        .device = {
            .device_id = 0xB0,  /* Sensor */
            .priority  = 30,
            .info      = SRXL2_DEVINFO_NO_RF,
            .uid       = 0xBA770001,
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

    printf("Waiting for master handshake...\n\n");

    last_status_ms = 0;
    last_batt_update_ms = 0;

    while (true) {
        /* Read PIO RX FIFO -- no echo guard needed with PIO */
        while (pio_uart_readable(&pio_uart)) {
            uint8_t byte = pio_uart_getc(&pio_uart);
            srxl2_feed(ctx, &byte, 1);
        }

        /* Tick state machine */
        srxl2_tick(ctx);

        uint32_t now = hal_time_ms(NULL);

        if (srxl2_is_connected(ctx)) {
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        /* Update battery physics every 100ms */
        if (now - last_batt_update_ms >= BATT_UPDATE_INTERVAL_MS) {
            uint32_t dt = now - last_batt_update_ms;
            last_batt_update_ms = now;
            update_battery(dt);
        }

        /* Periodic status */
        if (now - last_status_ms >= STATUS_INTERVAL_MS) {
            last_status_ms = now;
            float rem_pct = 100.0f *
                (1.0f - battery.capacity_used / battery.capacity_total);
            printf("[BAT] state=%s peers=%u  %.2fV %.2fA %.0f/%.0fmAh (%.1f%%) %.1fC\n",
                   srxl2_get_state(ctx), srxl2_peer_count(ctx),
                   battery.voltage, battery.current,
                   battery.capacity_used, battery.capacity_total,
                   rem_pct, battery.temperature);
        }

        tight_loop_contents();
    }
}
