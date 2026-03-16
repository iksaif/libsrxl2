/*
 * SRXL2 Battery Simulator (libsrxl2)
 *
 * Simulates an SRXL2 battery telemetry sensor (device type 0xB0) using the
 * new context-based libsrxl2 stack. Responds to handshake, sends Flight Pack
 * MAH (0x34) telemetry when polled.
 *
 * Supports both simulated (fakeuart) and real hardware (USB-to-serial).
 *
 * Usage: ./srxl2_new_battery_sim [options]
 *   --device <name>   Device (bus name for fakeuart, /dev/tty* for serial)
 *   --serial          Use USB-to-serial instead of fakeuart
 *   --baud <rate>     Baud rate: 115200 or 400000 (default: 115200)
 *   --id <0xB0-0xBF>  Device ID (default: 0xB0)
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
#include <math.h>
#include <getopt.h>

#include "srxl2.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"
#include "transport.h"

static volatile bool g_running = true;
static transport_handle_t *g_transport = NULL;
static bool g_verbose = false;

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
 * Build FP_MAH telemetry payload using encoder API
 *---------------------------------------------------------------------------*/

static void build_fp_mah_payload(uint8_t payload[16])
{
    srxl2_telem_fp_mah_t data = {
        .current_a     = g_battery.current,
        .charge_used_a = g_battery.capacity_used,
        .temp_a        = g_battery.temperature,
        .current_b     = 0.0f,
        .charge_used_b = 0.0f,
        .temp_b        = NAN,
        .s_id          = 0x00,
    };
    srxl2_encode_fp_mah(payload, &data);
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
    if (g_verbose) {
        printf("[TX %3d] ", len);
        for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
        printf("\n");
    }
    transport_send(g_transport, buf, len);
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

    case SRXL2_EVT_TELEM_REQUEST:
        if (g_verbose)
            printf("[Battery] Telemetry requested by master\n");
        break;

    default:
        if (g_verbose)
            printf("[Battery] Event: %d\n", evt->type);
        break;
    }
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
    printf("  -i, --id <0xB0-0xBF>  Device ID (default: 0xB0)\n");
    printf("  -v, --verbose         Verbose debug output (RX bytes, state, TX)\n");
    printf("  -l, --list-ports      List available serial ports and exit\n");
    printf("  -h, --help            Show this help\n");
}

int main(int argc, char *argv[])
{
    const char *device = "srxl2bus";
    bool use_serial = false;
    transport_baud_t baud = TRANSPORT_BAUD_115200;
    uint8_t device_id = 0xB0;

    static const struct option long_opts[] = {
        {"device",     required_argument, NULL, 'd'},
        {"serial",     no_argument,       NULL, 's'},
        {"baud",       required_argument, NULL, 'B'},
        {"id",         required_argument, NULL, 'i'},
        {"verbose",    no_argument,       NULL, 'v'},
        {"list-ports", no_argument,       NULL, 'l'},
        {"help",       no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:sB:i:vlh", long_opts, NULL)) != -1) {
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
        case 'i': {
            unsigned long val = strtoul(optarg, NULL, 0);
            if (val > 0xFF) {
                fprintf(stderr, "Device ID must be in range 0x00-0xFF\n");
                return 1;
            }
            device_id = (uint8_t)val;
            break;
        }
        case 'v':
            g_verbose = true;
            break;
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

    printf("=== SRXL2 Battery Simulator (libsrxl2) ===\n");
    printf("  Transport: %s\n", use_serial ? "serial" : "fakeuart");
    printf("  Device: %s\n", device);
    printf("  Device ID: 0x%02X (Sensor, Unit %u)\n",
           device_id, device_id & 0x0F);
    printf("  Battery: 4S LiPo, %.0f mAh\n\n", g_battery.capacity_total);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_transport = transport_init(type, device, "NewBattery", baud);
    if (!g_transport) {
        fprintf(stderr, "Failed to init transport\n");
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
        transport_close(g_transport);
        return 1;
    }
    srxl2_on_event(ctx, on_event, NULL);

    printf("[Battery] Running (Ctrl+C to stop)\n\n");

    struct timespec last_batt_update;
    clock_gettime(CLOCK_MONOTONIC, &last_batt_update);
    const char *last_state = "";
    uint32_t rx_total = 0;

    while (g_running) {
        /* Receive from bus (1ms timeout) */
        uint8_t buf[128];
        int n = transport_receive(g_transport, buf, sizeof(buf), 1, NULL, 0);
        if (n > 0) {
            rx_total += (uint32_t)n;
            if (g_verbose) {
                printf("[RX %3d] ", n);
                for (int i = 0; i < n && i < 32; i++) printf("%02X ", buf[i]);
                if (n > 32) printf("...");
                printf("\n");
            }
            srxl2_feed(ctx, buf, (size_t)n);
        }

        /* Tick state machine */
        srxl2_tick(ctx);

        /* Log state changes */
        const char *cur_state = srxl2_get_state(ctx);
        if (cur_state != last_state) {
            printf("[Battery] State: %s (rx_total=%u bytes)\n", cur_state, rx_total);
            last_state = cur_state;
        }

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

    printf("\n[Battery] Shutting down.\n");
    printf("  Channel data received: %u\n", g_chan_recv);
    printf("  Telemetry updates: %u\n", g_telem_sent);
    printf("  Final: %.2fV, %.2fA, %.0f/%.0fmAh, %.1f°C\n",
           g_battery.voltage, g_battery.current,
           g_battery.capacity_used, g_battery.capacity_total,
           g_battery.temperature);

    srxl2_destroy(ctx);
    transport_close(g_transport);
    return 0;
}
