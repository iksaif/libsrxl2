/*
 * SRXL2 Bus Master for Arduino Nano 33 BLE (Rev2)
 *
 * Runs the full libsrxl2 state machine as bus master: performs handshake,
 * sends channel data every 11ms, polls slaves for telemetry, and prints
 * decoded telemetry over USB CDC.
 *
 * Half-duplex: Serial1 TX (pin 0) and RX (pin 1) are both wired to the
 * SRXL2 bus data line. The master filters its own echo by discarding
 * bytes received during/just after a transmit.
 *
 * Wiring:
 *   Pin 0 (Serial1 TX) -> SRXL2 bus data line (via open-drain / bus driver)
 *   Pin 1 (Serial1 RX) -> SRXL2 bus data line
 *   GND                 -> SRXL2 bus ground
 *   USB                 -> Host computer (telemetry output)
 *
 * MIT License
 */

#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"

/* Static allocation for srxl2 context (no malloc) */
static uint8_t ctx_buf[sizeof(srxl2_ctx_t)] __attribute__((aligned(4)));
static srxl2_ctx_t *ctx;

/* Echo suppression */
static uint32_t tx_done_us;
#define ECHO_GUARD_US  2000

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
    Serial1.write(buf, len);
    Serial1.flush();
    tx_done_us = micros() + (uint32_t)len * 87;
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    Serial1.end();
    Serial1.begin(baud);
}

static uint32_t hal_time_ms(void *user)
{
    (void)user;
    return millis();
}

/*---------------------------------------------------------------------------
 * Telemetry print helper
 *---------------------------------------------------------------------------*/

static void print_telem_decoded(const srxl2_telem_decoded_t *d)
{
    switch (d->type) {
    case SRXL2_TELEM_TYPE_ESC:
        Serial.print("ESC: ");
        Serial.print(d->esc.rpm, 0);
        Serial.print("RPM ");
        Serial.print(d->esc.voltage, 2);
        Serial.print("V ");
        Serial.print(d->esc.current, 2);
        Serial.print("A ");
        Serial.print(d->esc.temp_fet, 1);
        Serial.print("C");
        break;
    case SRXL2_TELEM_TYPE_FP_MAH:
        Serial.print("FP: ");
        Serial.print(d->fp_mah.current_a, 1);
        Serial.print("A ");
        Serial.print(d->fp_mah.charge_used_a, 0);
        Serial.print("mAh");
        break;
    case SRXL2_TELEM_TYPE_SMART_BAT_REALTIME:
        Serial.print("BAT: ");
        Serial.print(d->smart_bat_realtime.current, 2);
        Serial.print("A ");
        Serial.print(d->smart_bat_realtime.consumption, 1);
        Serial.print("mAh ");
        Serial.print(d->smart_bat_realtime.min_cell, 3);
        Serial.print("-");
        Serial.print(d->smart_bat_realtime.max_cell, 3);
        Serial.print("V");
        break;
    case SRXL2_TELEM_TYPE_LIPOMON: {
        Serial.print("LIPO:");
        for (int i = 0; i < 6; i++) {
            float v = d->lipomon.cell[i];
            if (v == v) { /* not NAN */
                Serial.print(" ");
                Serial.print(v, 2);
                Serial.print("V");
            }
        }
        break;
    }
    case SRXL2_TELEM_TYPE_RPM:
        Serial.print("RPM: ");
        Serial.print(d->rpm.rpm, 0);
        Serial.print(" ");
        Serial.print(d->rpm.voltage, 2);
        Serial.print("V");
        break;
    default:
        Serial.print(srxl2_telem_type_name(d->type));
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
        Serial.print("[M] Handshake done, ");
        Serial.print(evt->handshake.peer_count);
        Serial.println(" peer(s)");
        break;

    case SRXL2_EVT_TELEMETRY: {
        uint32_t now = millis();
        if (now - last_telem_print_ms < TELEM_PRINT_INTERVAL_MS)
            break;
        last_telem_print_ms = now;

        srxl2_telem_decoded_t decoded;
        if (srxl2_decode_telemetry(evt->telemetry.payload, &decoded)) {
            Serial.print("[T 0x");
            Serial.print(evt->telemetry.device_id, HEX);
            Serial.print("] ");
            print_telem_decoded(&decoded);
            Serial.println();
        }
        break;
    }

    case SRXL2_EVT_TIMEOUT:
        Serial.println("[M] Timeout");
        break;

    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Setup / Loop
 *---------------------------------------------------------------------------*/

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    uint32_t start = millis();
    while (!Serial && millis() - start < 3000)
        ;

    Serial.println();
    Serial.println("=== SRXL2 Nano 33 BLE Master ===");
    Serial.println("Serial1 (pins 0/1) @ 115200, half-duplex");

    srxl2_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.role = SRXL2_ROLE_MASTER;
    cfg.device.device_id = 0x10;
    cfg.device.priority  = 20;
    cfg.device.info      = SRXL2_DEVINFO_TELEM_TX_ENABLED;
    cfg.device.uid       = 0xC0DE0002;
    cfg.hal.uart_send     = hal_uart_send;
    cfg.hal.uart_set_baud = hal_uart_set_baud;
    cfg.hal.time_ms       = hal_time_ms;
    cfg.hal.user          = NULL;
    cfg.baud_supported = SRXL2_BAUD_115200;

    ctx = srxl2_init_static(ctx_buf, sizeof(ctx_buf), &cfg);
    if (!ctx) {
        Serial.println("ERR: srxl2_init failed");
        while (true) delay(1000);
    }
    srxl2_on_event(ctx, on_event, NULL);

    tx_done_us = 0;
    last_telem_print_ms = 0;
    last_status_ms = 0;

    Serial.println("Running...");
    Serial.println();
}

/* Channels at center */
static uint16_t channels[32];
static bool channels_init = false;

void loop()
{
    if (!channels_init) {
        memset(channels, 0, sizeof(channels));
        for (int i = 0; i < 16; i++)
            channels[i] = 32768;
        channels_init = true;
    }

    /* Read UART, skip echo bytes */
    while (Serial1.available()) {
        uint8_t byte = (uint8_t)Serial1.read();
        if (micros() > tx_done_us + ECHO_GUARD_US)
            srxl2_feed(ctx, &byte, 1);
    }

    srxl2_tick(ctx);

    if (srxl2_is_connected(ctx)) {
        srxl2_set_channels(ctx, channels, 0x0000FFFF);
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    /* Periodic status */
    uint32_t now = millis();
    if (now - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now;
        Serial.print("[M] state=");
        Serial.print(srxl2_get_state(ctx));
        Serial.print(" peers=");
        Serial.println(srxl2_peer_count(ctx));
    }
}
