/*
 * SRXL2 Flight Controller Example for Arduino Nano 33 BLE (Rev2)
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
 * Half-duplex: Serial1 TX (pin 0) and RX (pin 1) are both wired
 * to the SRXL2 bus data line.
 *
 * Wiring:
 *   Pin 0 (Serial1 TX) -> SRXL2 bus data line (via open-drain / bus driver)
 *   Pin 1 (Serial1 RX) -> SRXL2 bus data line
 *   GND                 -> SRXL2 bus ground
 *   USB                 -> Host computer (debug output)
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

/* Which telemetry payload to send next */
static uint8_t telem_index;

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

    fc_mah_consumed += fc_current_amps * (float)dt / 3600.0f;

    fc_battery_voltage = 22.2f - (fc_mah_consumed / 5000.0f) * 3.0f;
    if (fc_battery_voltage < 19.8f)
        fc_battery_voltage = 19.8f;

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
        Serial.print("[FC] Handshake done, ");
        Serial.print(evt->handshake.peer_count);
        Serial.println(" peer(s)");
        break;

    case SRXL2_EVT_CHANNEL: {
        uint32_t now = millis();
        if (now - last_chan_print_ms < CHAN_PRINT_INTERVAL_MS)
            break;
        last_chan_print_ms = now;

        const srxl2_channel_data_t *ch = evt->channel.data;
        Serial.print("[FC] CH:");
        for (int i = 0; i < 4; i++) {
            Serial.print(" ");
            Serial.print(ch->values[i]);
        }
        Serial.print(" RSSI:");
        Serial.print(ch->rssi);
        if (ch->is_failsafe)
            Serial.print(" FAILSAFE");
        Serial.println();
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
        Serial.println("[FC] Connection lost");
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
    Serial.println("=== SRXL2 Nano 33 BLE FC (Slave) ===");
    Serial.println("Device ID: 0x30 (Flight Controller)");
    Serial.println("Serial1 (pins 0/1) @ 115200, half-duplex");
    Serial.println("Telemetry: FP_MAH (0x34), RPM (0x7E)");

    srxl2_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.role = SRXL2_ROLE_SLAVE;
    cfg.device.device_id = 0x30;  /* Flight Controller */
    cfg.device.priority  = 30;
    cfg.device.info      = SRXL2_DEVINFO_TELEM_TX_ENABLED;
    cfg.device.uid       = 0xFC300002;
    cfg.hal.uart_send     = hal_uart_send;
    cfg.hal.uart_set_baud = hal_uart_set_baud;
    cfg.hal.time_ms       = hal_time_ms;
    cfg.hal.user          = NULL;
    cfg.baud_supported = SRXL2_BAUD_115200;
    cfg.unprompted_hs  = true;  /* kick receivers that need handshake to enter SRXL2 mode */

    ctx = srxl2_init_static(ctx_buf, sizeof(ctx_buf), &cfg);
    if (!ctx) {
        Serial.println("ERR: srxl2_init failed");
        while (true) delay(1000);
    }
    srxl2_on_event(ctx, on_event, NULL);

    tx_done_us = 0;
    last_chan_print_ms = 0;
    last_status_ms = 0;
    last_sensor_update_ms = 0;
    telem_index = 0;

    Serial.println("Waiting for receiver handshake...");
    Serial.println();
}

void loop()
{
    /* Read UART, skip echo bytes */
    while (Serial1.available()) {
        uint8_t byte = (uint8_t)Serial1.read();
        if (micros() > tx_done_us + ECHO_GUARD_US)
            srxl2_feed(ctx, &byte, 1);
    }

    srxl2_tick(ctx);

    uint32_t now = millis();

    if (srxl2_is_connected(ctx)) {
        digitalWrite(LED_BUILTIN, HIGH);
        update_simulated_sensors(now);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    /* Periodic status */
    if (now - last_status_ms >= STATUS_INTERVAL_MS) {
        last_status_ms = now;
        Serial.print("[FC] state=");
        Serial.print(srxl2_get_state(ctx));
        Serial.print(" peers=");
        Serial.print(srxl2_peer_count(ctx));
        Serial.print("  ");
        Serial.print(fc_battery_voltage, 1);
        Serial.print("V ");
        Serial.print(fc_current_amps, 1);
        Serial.print("A ");
        Serial.print(fc_mah_consumed, 0);
        Serial.print("mAh ");
        Serial.print(fc_motor_rpm, 0);
        Serial.println("RPM");
    }
}
