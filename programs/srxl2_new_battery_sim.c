/*
 * SRXL2 Battery Simulator (libsrxl2)
 *
 * Simulates an SRXL2 battery telemetry sensor (device type 0xB0) using the
 * new context-based libsrxl2 stack. Responds to handshake, sends Flight Pack
 * MAH (0x34) telemetry when polled.
 *
 * Usage: ./srxl2_new_battery_sim [--bus <name>] [--id <0xB0-0xBF>] [--help]
 *
 * MIT License
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include "srxl2.h"
#include "srxl2_packet.h"
#include "fakeuart.h"

static volatile bool g_running = true;

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
} g_battery = {
    .voltage        = 16.8f,    /* 4S LiPo fully charged */
    .current        = 5.0f,
    .capacity_used  = 0.0f,
    .capacity_total = 5000.0f,
    .temperature    = 25.0f,
    .time_ms        = 0,
};

static void update_battery(uint32_t delta_ms)
{
    g_battery.time_ms += delta_ms;

    float hours = delta_ms / (1000.0f * 3600.0f);
    g_battery.capacity_used += g_battery.current * hours * 1000.0f;

    float remaining = 1.0f - (g_battery.capacity_used / g_battery.capacity_total);
    if (remaining < 0.0f) remaining = 0.0f;

    g_battery.voltage = 12.0f + (remaining * 4.8f)
                      + sinf(g_battery.time_ms / 1000.0f) * 0.1f;

    g_battery.current = 5.0f + sinf(g_battery.time_ms / 500.0f) * 1.0f;
    if (g_battery.current < 0.0f) g_battery.current = 0.0f;

    g_battery.temperature = 25.0f
        + (g_battery.capacity_used / g_battery.capacity_total) * 15.0f;
}

/*---------------------------------------------------------------------------
 * Build FP_MAH telemetry payload (big-endian wire format, 16 bytes)
 *---------------------------------------------------------------------------*/

static void build_fp_mah_payload(uint8_t payload[16])
{
    memset(payload, 0, 16);

    payload[0] = 0x34;  /* TELE_DEVICE_FP_MAH */
    payload[1] = 0x00;  /* sID */

    /* current_A: 0.1A resolution, big-endian int16 */
    int16_t current_a = (int16_t)(g_battery.current * 10.0f);
    srxl2_wr_be16(&payload[2], (uint16_t)current_a);

    /* chargeUsed_A: 1mAh, big-endian int16 */
    int16_t charge_a = (int16_t)(g_battery.capacity_used);
    srxl2_wr_be16(&payload[4], (uint16_t)charge_a);

    /* temp_A: 0.1°C, big-endian uint16 */
    uint16_t temp_a = (uint16_t)(g_battery.temperature * 10.0f);
    srxl2_wr_be16(&payload[6], temp_a);

    /* Battery B: not populated */
    srxl2_wr_be16(&payload[8],  0x7FFF);  /* current_B */
    srxl2_wr_be16(&payload[10], 0x7FFF);  /* chargeUsed_B */
    srxl2_wr_be16(&payload[12], 0x7FFF);  /* temp_B */
    srxl2_wr_be16(&payload[14], 0x0000);  /* spare */
}

/*---------------------------------------------------------------------------
 * Signal handler
 *---------------------------------------------------------------------------*/

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
}

/*---------------------------------------------------------------------------
 * HAL callbacks
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    fakeuart_send(buf, len);
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    (void)baud;
}

static uint32_t hal_time_ms(void *user)
{
    (void)user;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/*---------------------------------------------------------------------------
 * Event callback
 *---------------------------------------------------------------------------*/

static uint32_t g_telem_sent = 0;
static uint32_t g_chan_recv = 0;

static void on_event(srxl2_ctx_t *ctx, const srxl2_event_t *evt, void *user)
{
    (void)ctx;
    (void)user;

    switch (evt->type) {
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        printf("[Battery] Handshake complete\n");
        break;

    case SRXL2_EVT_CHANNEL:
        g_chan_recv++;
        if (g_chan_recv == 1)
            printf("[Battery] Receiving channel data from master\n");
        else if (g_chan_recv % 500 == 0)
            printf("[Battery] Channel data count: %u\n", g_chan_recv);
        break;

    case SRXL2_EVT_TIMEOUT:
        printf("[Battery] Connection timeout, restarting discovery\n");
        break;

    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    const char *bus_name = "srxl2bus";
    uint8_t device_id = 0xB0;

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--bus") == 0 || strcmp(argv[i], "-b") == 0) &&
            i + 1 < argc) {
            bus_name = argv[++i];
        } else if ((strcmp(argv[i], "--id") == 0 || strcmp(argv[i], "-i") == 0) &&
                   i + 1 < argc) {
            unsigned long val = strtoul(argv[++i], NULL, 0);
            if ((val & 0xF0) != 0xB0 || val > 0xBF) {
                fprintf(stderr, "Device ID must be in range 0xB0-0xBF\n");
                return 1;
            }
            device_id = (uint8_t)val;
        } else if (strcmp(argv[i], "--help") == 0 ||
                   strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [--bus <name>] [--id <0xB0-0xBF>]\n", argv[0]);
            printf("  --bus, -b   Bus name (default: srxl2bus)\n");
            printf("  --id,  -i   Device ID (default: 0xB0)\n");
            return 0;
        } else {
            bus_name = argv[i];
        }
    }

    printf("=== SRXL2 New Battery Simulator (libsrxl2) ===\n");
    printf("  Bus: %s\n", bus_name);
    printf("  Device ID: 0x%02X (Sensor, Unit %u)\n",
           device_id, device_id & 0x0F);
    printf("  Battery: 4S LiPo, %.0f mAh\n\n", g_battery.capacity_total);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (!fakeuart_init(bus_name, "NewBattery")) {
        fprintf(stderr, "Failed to init fakeuart\n");
        return 1;
    }

    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_SLAVE,
        .device = {
            .device_id = device_id,
            .priority  = 30,
            .info      = SRXL2_DEVINFO_NO_RF,
            .uid       = 0xAABBCCDD,
        },
        .hal = {
            .uart_send     = hal_uart_send,
            .uart_set_baud = hal_uart_set_baud,
            .time_ms       = hal_time_ms,
            .user          = NULL,
        },
        .baud_supported = SRXL2_BAUD_115200,
    };

    srxl2_ctx_t *ctx = srxl2_init(&cfg);
    if (!ctx) {
        fprintf(stderr, "Failed to init srxl2 context\n");
        fakeuart_close();
        return 1;
    }
    srxl2_on_event(ctx, on_event, NULL);

    printf("[Battery] Running (Ctrl+C to stop)\n\n");

    struct timespec last_batt_update;
    clock_gettime(CLOCK_MONOTONIC, &last_batt_update);

    while (g_running) {
        /* Receive from bus (1ms timeout) */
        uint8_t buf[128];
        int n = fakeuart_receive(buf, sizeof(buf), 1, NULL, 0);
        if (n > 0)
            srxl2_feed(ctx, buf, (size_t)n);

        /* Tick state machine */
        srxl2_tick(ctx);

        /* Update battery physics and telemetry every 100ms */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - last_batt_update.tv_sec) * 1000 +
                          (now.tv_nsec - last_batt_update.tv_nsec) / 1000000;

        if (elapsed_ms >= 100) {
            update_battery((uint32_t)elapsed_ms);

            uint8_t payload[16];
            build_fp_mah_payload(payload);
            srxl2_set_telemetry(ctx, payload);
            g_telem_sent++;

            if (g_telem_sent % 50 == 0) {
                float rem_pct = 100.0f *
                    (1.0f - g_battery.capacity_used / g_battery.capacity_total);
                printf("[Battery] Telem #%u: %.2fV, %.2fA, "
                       "%.0f/%.0fmAh (%.1f%% left), %.1f°C\n",
                       g_telem_sent, g_battery.voltage, g_battery.current,
                       g_battery.capacity_used, g_battery.capacity_total,
                       rem_pct, g_battery.temperature);
            }

            last_batt_update = now;
        }
    }

    /* Stats */
    fakeuart_stats_t stats;
    fakeuart_get_stats(&stats);
    printf("\n[Battery] Shutting down.\n");
    printf("  TX: %llu pkts, %llu bytes\n", stats.tx_packets, stats.tx_bytes);
    printf("  RX: %llu pkts, %llu bytes\n", stats.rx_packets, stats.rx_bytes);
    printf("  Channel data received: %u\n", g_chan_recv);
    printf("  Telemetry updates: %u\n", g_telem_sent);
    printf("  Final: %.2fV, %.2fA, %.0f/%.0fmAh, %.1f°C\n",
           g_battery.voltage, g_battery.current,
           g_battery.capacity_used, g_battery.capacity_total,
           g_battery.temperature);

    srxl2_destroy(ctx);
    fakeuart_close();
    return 0;
}
