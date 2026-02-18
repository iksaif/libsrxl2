/*
 * SRXL2 Master Simulator (libsrxl2)
 *
 * Bus master using the new context-based libsrxl2 stack.
 * Performs device discovery, sends channel data every 11ms,
 * polls slaves for telemetry.
 *
 * Usage: ./srxl2_new_master_sim [--bus <name>] [--help]
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

#include "srxl2.h"
#include "srxl2_telemetry.h"
#include "fakeuart.h"

static volatile bool g_running = true;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
}

/*---------------------------------------------------------------------------
 * HAL callbacks (wrapping fakeuart)
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    fakeuart_send(buf, len);
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    (void)baud;  /* fakeuart has no baud concept */
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

static void on_event(srxl2_ctx_t *ctx, const srxl2_event_t *evt, void *user)
{
    (void)user;

    switch (evt->type) {
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        printf("[Master] Handshake complete, %u peer(s)\n",
               evt->handshake.peer_count);
        break;

    case SRXL2_EVT_TELEMETRY: {
        static uint32_t telem_count = 0;
        telem_count++;

        srxl2_telem_decoded_t decoded;
        if (srxl2_decode_telemetry(evt->telemetry.payload, &decoded)) {
            if (telem_count % 50 == 0) {
                printf("[Master] Telemetry #%u from 0x%02X: %s",
                       telem_count, evt->telemetry.device_id,
                       srxl2_telem_type_name(decoded.type));
                if (decoded.type == SRXL2_TELEM_TYPE_FP_MAH) {
                    printf(" (%.1fA, %.0fmAh, %.1f°C)",
                           decoded.fp_mah.current_a,
                           decoded.fp_mah.charge_used_a,
                           decoded.fp_mah.temp_a);
                }
                printf("\n");
            }
        } else if (telem_count % 50 == 0) {
            printf("[Master] Telemetry #%u from 0x%02X (raw)\n",
                   telem_count, evt->telemetry.device_id);
        }
        break;
    }

    case SRXL2_EVT_TIMEOUT:
        printf("[Master] Connection timeout\n");
        break;

    default:
        break;
    }

    (void)ctx;
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    const char *bus_name = "srxl2bus";

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--bus") == 0 || strcmp(argv[i], "-b") == 0) &&
            i + 1 < argc) {
            bus_name = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0 ||
                   strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [--bus <name>]\n", argv[0]);
            printf("  --bus, -b   Bus name (default: srxl2bus)\n");
            return 0;
        } else {
            bus_name = argv[i];  /* legacy positional */
        }
    }

    printf("=== SRXL2 New Master Simulator (libsrxl2) ===\n");
    printf("  Bus: %s\n", bus_name);
    printf("  Device ID: 0x10 (Remote Receiver / Master)\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (!fakeuart_init(bus_name, "NewMaster")) {
        fprintf(stderr, "Failed to init fakeuart\n");
        return 1;
    }

    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = {
            .device_id = 0x10,
            .priority  = 20,
            .info      = SRXL2_DEVINFO_TELEM_TX_ENABLED,
            .uid       = 0x12345678,
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

    printf("[Master] Running (Ctrl+C to stop)\n\n");

    /* Simulated channel data: 16 channels at center */
    uint16_t channels[32];
    memset(channels, 0, sizeof(channels));
    for (int i = 0; i < 16; i++)
        channels[i] = 32768;

    struct timespec last_chan;
    clock_gettime(CLOCK_MONOTONIC, &last_chan);

    uint32_t frame_count = 0;

    while (g_running) {
        /* Receive from bus (1ms timeout) */
        uint8_t buf[128];
        int n = fakeuart_receive(buf, sizeof(buf), 1, NULL, 0);
        if (n > 0)
            srxl2_feed(ctx, buf, (size_t)n);

        /* Tick state machine */
        srxl2_tick(ctx);

        /* Every 11ms, set channel data */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - last_chan.tv_sec) * 1000 +
                          (now.tv_nsec - last_chan.tv_nsec) / 1000000;

        if (elapsed_ms >= 11) {
            /* Slight throttle variation for demo */
            if (frame_count % 100 < 50)
                channels[0] = 32768 + (uint16_t)(frame_count % 100) * 100;
            else
                channels[0] = 32768 + (uint16_t)(100 - frame_count % 100) * 100;

            srxl2_set_channels(ctx, channels, 0x0000FFFF);
            last_chan = now;
            frame_count++;

            if (frame_count % 500 == 0) {
                printf("[Master] Frame %u, state=%s, peers=%u\n",
                       frame_count, srxl2_get_state(ctx),
                       srxl2_peer_count(ctx));
            }
        }
    }

    /* Stats */
    fakeuart_stats_t stats;
    fakeuart_get_stats(&stats);
    printf("\n[Master] Shutting down.\n");
    printf("  TX: %llu pkts, %llu bytes\n", stats.tx_packets, stats.tx_bytes);
    printf("  RX: %llu pkts, %llu bytes\n", stats.rx_packets, stats.rx_bytes);
    printf("  Frames: %u\n", frame_count);

    srxl2_destroy(ctx);
    fakeuart_close();
    return 0;
}
