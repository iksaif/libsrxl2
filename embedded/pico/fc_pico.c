/*
 * SRXL2 Flight Controller Example for Raspberry Pi Pico (PIO UART)
 *
 * Demonstrates an FC connecting to a Spektrum SRXL2 receiver.
 * The receiver is bus master; the FC registers as a slave with
 * device ID 0x30 (Flight Controller), receives channel data,
 * and sends back telemetry that an FC would typically provide:
 *   - Flight Pack Capacity (0x34): battery current, mAh consumed
 *   - RPM/Volts/Temp (0x7E): motor RPM, pack voltage
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
 *   External 5V         -> Receiver VCC
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

/* Generated PIO header (built by pioasm from pio_uart.pio) */
#include "pio_uart.pio.h"

/* PIO / pin config */
#define SRXL2_PIO       pio0
#define SRXL2_SM_RX     0
#define SRXL2_SM_TX     1
#define SRXL2_PIN       0       /* Single data pin (GPIO 0) */
#define SRXL2_BAUD_INIT 115200

#define LED_PIN         PICO_DEFAULT_LED_PIN

/* PIO program offsets (set during init) */
static uint pio_rx_offset;
static uint pio_tx_offset;
static uint32_t current_baud;

/* Static allocation for srxl2 context (no malloc) */
static uint8_t ctx_buf[sizeof(srxl2_ctx_t)] __attribute__((aligned(4)));
static srxl2_ctx_t *ctx;

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
 * PIO UART helpers
 *---------------------------------------------------------------------------*/

static void pio_uart_init(uint pin, uint baud)
{
    current_baud = baud;

    /* Load RX program */
    pio_rx_offset = pio_add_program(SRXL2_PIO, &uart_rx_program);
    uart_rx_program_init(SRXL2_PIO, SRXL2_SM_RX, pio_rx_offset, pin, baud);

    /* Load TX program */
    pio_tx_offset = pio_add_program(SRXL2_PIO, &uart_tx_program);
    uart_tx_program_init(SRXL2_PIO, SRXL2_SM_TX, pio_tx_offset, pin, baud);
    /* TX SM is NOT enabled -- uart_tx_program_init leaves it stopped */
}

static void pio_uart_set_baud(uint baud)
{
    current_baud = baud;
    float div = (float)clock_get_hz(clk_sys) / (8.0f * baud);
    pio_sm_set_clkdiv(SRXL2_PIO, SRXL2_SM_RX, div);
    pio_sm_set_clkdiv(SRXL2_PIO, SRXL2_SM_TX, div);
    /* Clkdiv takes effect after restart/next instruction boundary */
}

/*---------------------------------------------------------------------------
 * HAL callbacks for libsrxl2
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;

    /* 1. Disable RX SM */
    pio_sm_set_enabled(SRXL2_PIO, SRXL2_SM_RX, false);

    /* 2. Set pin as output for TX */
    pio_sm_set_consecutive_pindirs(SRXL2_PIO, SRXL2_SM_TX, SRXL2_PIN, 1, true);

    /* 3. Restart TX SM from the beginning of its program */
    pio_sm_restart(SRXL2_PIO, SRXL2_SM_TX);
    pio_sm_exec(SRXL2_PIO, SRXL2_SM_TX,
                pio_encode_jmp(pio_tx_offset));
    pio_sm_set_enabled(SRXL2_PIO, SRXL2_SM_TX, true);

    /* 4. Write bytes to TX FIFO */
    for (uint8_t i = 0; i < len; i++) {
        /* TX program pulls 32-bit words, data in bits [7:0] */
        pio_sm_put_blocking(SRXL2_PIO, SRXL2_SM_TX, (uint32_t)buf[i]);
    }

    /* 5. Wait for TX to finish: FIFO drains + last byte clocks out
     *    Each byte = 10 bit-times (start + 8 data + stop)
     *    Bit time at 115200 = ~8.68us, at 400000 = 2.5us
     *    Wait for FIFO to drain, then one extra byte time for the
     *    last byte being shifted out */

    /* Wait until TX FIFO is empty */
    while (!pio_sm_is_tx_fifo_empty(SRXL2_PIO, SRXL2_SM_TX))
        tight_loop_contents();

    /* Wait for the last byte to finish shifting out.
     * The pull instruction stalls when FIFO is empty, so once we see
     * FIFO empty AND the SM stalls, the last bit has been sent.
     * Conservative: wait one full byte time after FIFO empty. */
    uint32_t byte_us = (10 * 1000000UL) / current_baud;
    busy_wait_us(byte_us + 5);

    /* 6. Disable TX SM */
    pio_sm_set_enabled(SRXL2_PIO, SRXL2_SM_TX, false);

    /* 7. Set pin back to input (high-Z, internal pull-up keeps line high) */
    pio_sm_set_consecutive_pindirs(SRXL2_PIO, SRXL2_SM_RX, SRXL2_PIN, 1, false);

    /* 8. Clear any stale data in RX FIFO, restart RX SM */
    pio_sm_clear_fifos(SRXL2_PIO, SRXL2_SM_RX);
    pio_sm_restart(SRXL2_PIO, SRXL2_SM_RX);
    pio_sm_exec(SRXL2_PIO, SRXL2_SM_RX,
                pio_encode_jmp(pio_rx_offset));
    pio_sm_set_enabled(SRXL2_PIO, SRXL2_SM_RX, true);
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    pio_uart_set_baud(baud);
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

    /* Init PIO UART on single pin */
    pio_uart_init(SRXL2_PIN, SRXL2_BAUD_INIT);

    /* Wait for USB CDC */
    uint32_t start = (uint32_t)(time_us_64() / 1000);
    while (!stdio_usb_connected() && (uint32_t)(time_us_64() / 1000) - start < 3000)
        tight_loop_contents();

    printf("\n=== SRXL2 Pico FC (Slave) ===\n");
    printf("Device ID: 0x30 (Flight Controller)\n");
    printf("PIO UART on GPIO %d @ %u baud, single-wire half-duplex\n",
           SRXL2_PIN, SRXL2_BAUD_INIT);
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

    last_chan_print_ms = 0;
    last_status_ms = 0;
    last_sensor_update_ms = 0;
    telem_index = 0;

    while (true) {
        /* Read PIO RX FIFO -- no echo guard needed with PIO */
        while (!pio_sm_is_rx_fifo_empty(SRXL2_PIO, SRXL2_SM_RX)) {
            uint8_t byte = (uint8_t)(pio_sm_get(SRXL2_PIO, SRXL2_SM_RX) >> 24);
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
