/*
 * SRXL2 Master Simulator (libsrxl2)
 *
 * Bus master using the new context-based libsrxl2 stack.
 * Performs device discovery, sends channel data every 11ms,
 * polls slaves for telemetry.
 *
 * Supports both simulated (fakeuart) and real hardware (USB-to-serial).
 *
 * Usage: ./srxl2_new_master_sim [options]
 *   --device <name>   Device (bus name for fakeuart, /dev/tty* for serial)
 *   --serial          Use USB-to-serial instead of fakeuart
 *   --baud <rate>     Baud rate: 115200 or 400000 (default: 115200)
 *   --help
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
#include <getopt.h>

#include "srxl2.h"
#include "srxl2_telemetry.h"
#include "transport.h"

static volatile bool g_running = true;
static transport_handle_t *g_transport = NULL;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
}

/*---------------------------------------------------------------------------
 * HAL callbacks (wrapping transport)
 *---------------------------------------------------------------------------*/

static void hal_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    transport_send(g_transport, buf, len);
}

static void hal_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    (void)baud;  /* transport doesn't support runtime baud change */
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

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("  -d, --device <name>   Device (bus name or /dev/ttyUSB0)\n");
    printf("  -s, --serial          Use USB-to-serial instead of fakeuart\n");
    printf("  -B, --baud <rate>     Baud rate: 115200 or 400000 (default: 115200)\n");
    printf("  -l, --list-ports      List available serial ports and exit\n");
    printf("  -h, --help            Show this help\n");
}

int main(int argc, char *argv[])
{
    const char *device = "srxl2bus";
    bool use_serial = false;
    transport_baud_t baud = TRANSPORT_BAUD_115200;

    static const struct option long_opts[] = {
        {"device",     required_argument, NULL, 'd'},
        {"serial",     no_argument,       NULL, 's'},
        {"baud",       required_argument, NULL, 'B'},
        {"list-ports", no_argument,       NULL, 'l'},
        {"help",       no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:sB:lh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            device = optarg;
            break;
        case 's':
            use_serial = true;
            break;
        case 'B': {
            int rate = atoi(optarg);
            if (rate == 400000) baud = TRANSPORT_BAUD_400000;
            else if (rate != 115200) {
                fprintf(stderr, "Invalid baud rate (use 115200 or 400000)\n");
                return 1;
            }
            break;
        }
        case 'l':
            transport_list_serial_ports();
            return 0;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }
    if (optind < argc)
        device = argv[optind];

    transport_type_t type = use_serial ? TRANSPORT_TYPE_SERIAL : TRANSPORT_TYPE_FAKEUART;

    printf("=== SRXL2 Master (libsrxl2) ===\n");
    printf("  Transport: %s\n", use_serial ? "serial" : "fakeuart");
    printf("  Device: %s\n", device);
    printf("  Device ID: 0x10 (Remote Receiver / Master)\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_transport = transport_init(type, device, "NewMaster", baud);
    if (!g_transport) {
        fprintf(stderr, "Failed to init transport\n");
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
        transport_close(g_transport);
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
        int n = transport_receive(g_transport, buf, sizeof(buf), 1, NULL, 0);
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

    printf("\n[Master] Shutting down. Frames: %u\n", frame_count);

    srxl2_destroy(ctx);
    transport_close(g_transport);
    return 0;
}
