/*
 * SRXL2 Flight Controller Example for Raspberry Pi Pico
 *
 * Demonstrates an FC connecting to a Spektrum SRXL2 receiver.
 * The receiver is bus master; the FC registers as a slave with
 * device ID 0x30 (Flight Controller), receives channel data,
 * and sends back telemetry that an FC would typically provide:
 *   - Flight Pack Capacity (0x34): battery current, mAh consumed
 *   - RPM/Volts/Temp (0x7E): motor RPM, pack voltage
 *
 * This is a reference implementation for integrating SRXL2 into
 * real FC firmware (iNav, ArduPilot, etc).
 *
 * Half-duplex: UART0 TX (GPIO 0) and RX (GPIO 1) are both wired
 * to the SRXL2 bus data line.
 *
 * Wiring:
 *   GPIO 0 (UART0 TX) -> SRXL2 bus data line (via open-drain / bus driver)
 *   GPIO 1 (UART0 RX) -> SRXL2 bus data line
 *   GND               -> SRXL2 bus ground
 *   USB               -> Host computer (debug output)
 *
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"

/* UART config */
#define SRXL2_UART      uart0
#define SRXL2_TX_PIN    0
#define SRXL2_RX_PIN    1

#define LED_PIN         PICO_DEFAULT_LED_PIN

/* Static allocation for srxl2 context (no malloc) */
static uint8_t ctx_buf[sizeof(srxl2_ctx_t)] __attribute__((aligned(4)));
static srxl2_ctx_t *ctx;

/* Echo suppression: ignore RX bytes for this many us after TX */
static uint64_t tx_done_us;
#define ECHO_GUARD_US   2000  /* 2ms guard after last TX byte */

/* Print rate limiting */
static uint32_t last_chan_print_ms;
#define CHAN_PRINT_INTERVAL_MS  500

static uint32_t last_status_ms;
#define STATUS_INTERVAL_MS  5000

/* Simulated FC sensor data (in a real FC, these come from ADC/sensors) */
static float fc_battery_voltage = 22.2f; /* 6S at nominal */
static float fc_current_amps    = 12.5f;
static float fc_mah_consumed    = 0.0f;
static float fc_motor_rpm       = 0.0f;
static uint32_t last_sensor_update_ms;

/* Which telemetry payload to send next (alternate between types) */
static uint8_t telem_index;

/*---------------------------------------------------------------------------
 * HAL callbacks
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    uart_write_blocking(SRXL2_UART, buf, len);
    tx_done_us = time_us_64() + (uint64_t)len * 87;
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    uart_set_baudrate(SRXL2_UART, baud);
}

static uint32_t hal_time_ms(void *user)
{
    (void)user;
    return (uint32_t)(time_us_64() / 1000);
}

/*---------------------------------------------------------------------------
 * Telemetry payload builders (using encoder API)
 *---------------------------------------------------------------------------*/

static void build_fp_mah_payload(uint8_t payload[16])
{
    srxl2_telem_fp_mah_t data = {
        .current_a     = fc_current_amps,
        .charge_used_a = fc_mah_consumed,
        .temp_a        = NAN,
        .current_b     = 0.0f,
        .charge_used_b = 0.0f,
        .temp_b        = NAN,
        .s_id          = 0x00,
    };
    srxl2_encode_fp_mah(payload, &data);
}

static void build_rpm_payload(uint8_t payload[16])
{
    srxl2_telem_rpm_t data = {
        .rpm         = fc_motor_rpm,
        .voltage     = fc_battery_voltage,
        .temperature = NAN,
        .rssi_a      = 0,
        .rssi_b      = 0,
        .s_id        = 0x00,
    };
    srxl2_encode_rpm(payload, &data);
}

/*---------------------------------------------------------------------------
 * Update simulated sensor data
 *---------------------------------------------------------------------------*/

static void update_simulated_sensors(uint32_t now_ms)
{
    uint32_t dt = now_ms - last_sensor_update_ms;
    if (dt < 100)
        return;
    last_sensor_update_ms = now_ms;

    /* Simulate battery drain */
    fc_mah_consumed += fc_current_amps * (float)dt / 3600.0f;

    /* Simulate voltage sag under load */
    fc_battery_voltage = 22.2f - (fc_mah_consumed / 5000.0f) * 3.0f;
    if (fc_battery_voltage < 19.8f)
        fc_battery_voltage = 19.8f;

    /* Simulate motor RPM (proportional to throttle -- fake it) */
    fc_motor_rpm = 15000.0f;
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
        printf("[FC] Handshake done, %u peer(s)\n", evt->handshake.peer_count);
        break;

    case SRXL2_EVT_CHANNEL: {
        uint32_t now = hal_time_ms(NULL);
        if (now - last_chan_print_ms < CHAN_PRINT_INTERVAL_MS)
            break;
        last_chan_print_ms = now;

        const srxl2_channel_data_t *ch = evt->channel.data;
        printf("[FC] CH: %5u %5u %5u %5u  RSSI:%d%s\n",
               ch->values[0], ch->values[1],
               ch->values[2], ch->values[3],
               ch->rssi,
               ch->is_failsafe ? " FAILSAFE" : "");
        break;
    }

    case SRXL2_EVT_TELEM_REQUEST: {
        /* Pull-model: fill telemetry just-in-time when polled */
        uint8_t payload[16];
        if (telem_index == 0)
            build_fp_mah_payload(payload);
        else
            build_rpm_payload(payload);
        telem_index = (telem_index + 1) % 2;
        srxl2_set_telemetry(c, payload);
        break;
    }

    case SRXL2_EVT_TIMEOUT:
        printf("[FC] Connection lost\n");
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

    /* UART init */
    uart_init(SRXL2_UART, 115200);
    gpio_set_function(SRXL2_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SRXL2_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(SRXL2_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(SRXL2_UART, false, false);

    /* Wait for USB CDC */
    uint32_t start = (uint32_t)(time_us_64() / 1000);
    while (!stdio_usb_connected() && (uint32_t)(time_us_64() / 1000) - start < 3000)
        tight_loop_contents();

    printf("\n=== SRXL2 Pico FC (Slave) ===\n");
    printf("Device ID: 0x30 (Flight Controller)\n");
    printf("UART0 (GPIO %d/%d) @ 115200, half-duplex\n", SRXL2_TX_PIN, SRXL2_RX_PIN);
    printf("Telemetry: FP_MAH (0x34), RPM (0x7E)\n\n");

    /* Init libsrxl2 as SLAVE with FC device ID */
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_SLAVE,
        .device = {
            .device_id = 0x30,  /* Flight Controller */
            .priority  = 30,
            .info      = SRXL2_DEVINFO_TELEM_TX_ENABLED,
            .uid       = 0xFC300001,
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

    printf("Waiting for receiver handshake...\n\n");

    tx_done_us = 0;
    last_chan_print_ms = 0;
    last_status_ms = 0;
    last_sensor_update_ms = 0;
    telem_index = 0;

    while (true) {
        /* Read UART, skip echo bytes */
        while (uart_is_readable(SRXL2_UART)) {
            uint8_t byte = uart_getc(SRXL2_UART);
            if (time_us_64() > tx_done_us + ECHO_GUARD_US)
                srxl2_feed(ctx, &byte, 1);
        }

        /* Tick state machine */
        srxl2_tick(ctx);

        uint32_t now = hal_time_ms(NULL);

        if (srxl2_is_connected(ctx)) {
            gpio_put(LED_PIN, 1);
            update_simulated_sensors(now);
        } else {
            gpio_put(LED_PIN, 0);
        }

        /* Periodic status */
        if (now - last_status_ms >= STATUS_INTERVAL_MS) {
            last_status_ms = now;
            printf("[FC] state=%s peers=%u  %.1fV %.1fA %.0fmAh %.0fRPM\n",
                   srxl2_get_state(ctx), srxl2_peer_count(ctx),
                   fc_battery_voltage, fc_current_amps,
                   fc_mah_consumed, fc_motor_rpm);
        }

        tight_loop_contents();
    }
}
