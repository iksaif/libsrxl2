/*
MIT License

SRXL2 Bus Sniffer

This program captures and decodes all SRXL2 traffic on the bus.
Supports both simulated (fakeuart) and real (USB-to-serial) transports.

Features:
- Multiple output formats (details, json, oneline, state)
- Real-time state view with ncurses
- Transport abstraction (fakeuart or serial)
- Statistics tracking

Usage: ./srxl2_sniffer [options]
  --device <name>      Device (bus name for fakeuart, /dev/tty* for serial)
  --serial             Use USB-to-serial instead of fakeuart
  --baud <rate>        Baud rate (115200 or 400000)
  --format <fmt>       Output format: details|json|oneline|state
  --hex                Show hex dump (details format only)
  --show-internal      Show internal heartbeat packets
  --no-color           Disable colors
  --list-ports         List available serial ports and exit
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <getopt.h>

// Include libsrxl2
#include "srxl2.h"
#include "srxl2_packet.h"

// Include transport library
#include "transport.h"

// ANSI Color codes (prefixed with ANSI_ to avoid ncurses macro conflicts)
#define ANSI_RESET   "\033[0m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_BOLD    "\033[1m"

// Output format types
typedef enum {
    FORMAT_DETAILS,
    FORMAT_JSON,
    FORMAT_ONELINE,
    FORMAT_STATE,
} output_format_t;

// Globals
static bool g_running = true;
static bool g_show_hex = false;
static bool g_use_colors = true;
static bool g_show_internal = false;
static uint64_t g_packet_count = 0;
static output_format_t g_format = FORMAT_DETAILS;
static transport_handle_t* g_transport = NULL;

// Serial framing buffer (reassembles byte stream into complete packets)
static struct {
    uint8_t buf[SRXL2_MAX_PACKET_SIZE];
    uint8_t len;
    bool synced;
} g_frame = {0};

// Statistics
static struct {
    uint64_t total_packets;
    uint64_t handshake_packets;
    uint64_t channel_packets;
    uint64_t telemetry_packets;
    uint64_t bind_packets;
    uint64_t rssi_packets;
    uint64_t param_packets;
    uint64_t internal_packets;
    uint64_t unknown_packets;
    uint64_t invalid_packets;
} g_stats = {0};

// Device state (for state format)
#define MAX_DEVICES 16
#define MAX_TELEMETRY_SENSORS 8

typedef struct {
    uint8_t sensor_id;
    uint8_t raw_data[16];
    uint8_t data_len;
    uint64_t last_seen;
} telemetry_sensor_t;

typedef struct {
    uint8_t device_id;
    bool active;
    uint32_t uid;
    uint8_t priority;
    int8_t rssi;
    uint16_t frame_losses;
    uint32_t channel_mask;
    uint16_t channels[32];
    uint64_t last_seen;
    uint64_t packet_count;

    // Telemetry sensors for this device
    telemetry_sensor_t telemetry[MAX_TELEMETRY_SENSORS];
    int telemetry_count;
} device_state_t;

static device_state_t g_devices[MAX_DEVICES] = {0};
static int g_device_count = 0;

///////////////////////////////////////////////////////////////////////////////
// Signal handler
///////////////////////////////////////////////////////////////////////////////

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
}

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

static uint64_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static device_state_t* find_or_create_device(uint8_t device_id)
{
    // Find existing device
    for (int i = 0; i < g_device_count; i++) {
        if (g_devices[i].device_id == device_id) {
            return &g_devices[i];
        }
    }

    // Create new device
    if (g_device_count < MAX_DEVICES) {
        device_state_t* dev = &g_devices[g_device_count++];
        dev->device_id = device_id;
        dev->active = true;
        return dev;
    }

    return NULL;
}

static void json_escape(const char* str)
{
    while (*str) {
        if (*str == '"' || *str == '\\') {
            putchar('\\');
        }
        putchar(*str);
        str++;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Format: JSON
///////////////////////////////////////////////////////////////////////////////

static void output_json_packet(const srxl2_decoded_pkt_t* pkt, uint64_t timestamp, const char* sender, size_t length)
{
    printf("{");
    printf("\"timestamp\":%llu,", (unsigned long long)timestamp);
    printf("\"packet_num\":%llu,", (unsigned long long)g_packet_count);
    printf("\"length\":%zu,", length);
    printf("\"sender\":\"");
    json_escape(sender);
    printf("\",");
    printf("\"type\":\"");
    json_escape(srxl2_packet_type_name(pkt->packet_type));
    printf("\",");
    printf("\"type_id\":\"0x%02X\",", pkt->packet_type);

    switch (pkt->packet_type) {
        case SRXL2_PKT_HANDSHAKE: {
            printf("\"data\":{");
            printf("\"src\":\"0x%02X\",", pkt->handshake.src_id);
            printf("\"dest\":\"0x%02X\",", pkt->handshake.dest_id);
            printf("\"priority\":%u,", pkt->handshake.priority);
            printf("\"baud\":%u,", pkt->handshake.baud_supported & SRXL2_BAUD_400000 ? 400000 : 115200);
            printf("\"uid\":\"0x%08X\"", pkt->handshake.uid);
            printf("}");
            break;
        }
        case SRXL2_PKT_CONTROL: {
            printf("\"data\":{");
            printf("\"cmd\":%u,", pkt->control.cmd);
            printf("\"reply_id\":\"0x%02X\",", pkt->control.reply_id);
            printf("\"rssi\":%d,", pkt->control.channel.rssi);
            printf("\"frame_losses\":%u,", pkt->control.channel.frame_losses);
            printf("\"channel_mask\":\"0x%08X\",", pkt->control.channel.mask);
            printf("\"num_channels\":%u", pkt->control.channel.num_channels);
            printf("}");
            break;
        }
        case SRXL2_PKT_TELEMETRY: {
            printf("\"data\":{");
            printf("\"dest\":\"0x%02X\",", pkt->telemetry.dest_id);
            printf("\"sensor_id\":\"0x%02X\"", pkt->telemetry.payload[0]);
            printf("}");
            break;
        }
        default:
            printf("\"data\":null");
            break;
    }

    printf("}\n");
    fflush(stdout);
}

///////////////////////////////////////////////////////////////////////////////
// Format: Oneline
///////////////////////////////////////////////////////////////////////////////

static void output_oneline_packet(const srxl2_decoded_pkt_t* pkt, uint64_t timestamp, const char* sender)
{
    // Timestamp
    if (g_use_colors) printf("%s", ANSI_CYAN);
    printf("[%llu.%06llu]",
           (unsigned long long)(timestamp / 1000000),
           (unsigned long long)(timestamp % 1000000));
    if (g_use_colors) printf("%s", ANSI_RESET);

    printf(" #%llu ", (unsigned long long)g_packet_count);

    // Sender
    if (g_use_colors) printf("%s", ANSI_BOLD);
    printf("%-15s", sender);
    if (g_use_colors) printf("%s", ANSI_RESET);

    // Packet type with emoji and color
    const char* emoji = "?";
    const char* color = ANSI_RESET;

    switch (pkt->packet_type) {
        case SRXL2_PKT_HANDSHAKE:
            emoji = "HS";
            color = ANSI_YELLOW;
            break;
        case SRXL2_PKT_CONTROL:
            emoji = "CD";
            color = ANSI_BLUE;
            break;
        case SRXL2_PKT_TELEMETRY:
            emoji = "TL";
            color = ANSI_GREEN;
            break;
        case SRXL2_PKT_BIND:
            emoji = "BD";
            color = ANSI_MAGENTA;
            break;
        case SRXL2_PKT_RSSI:
            emoji = "RS";
            color = ANSI_YELLOW;
            break;
        case 0x50: /* Parameter */
            emoji = "PM";
            color = ANSI_CYAN;
            break;
        case 0x99: /* Internal */
            emoji = "IN";
            color = ANSI_MAGENTA;
            break;
        default:
            emoji = "??";
            color = ANSI_RED;
            break;
    }

    if (g_use_colors) printf(" %s%s", color, emoji);
    else printf(" %s", emoji);

    printf(" %-12s", srxl2_packet_type_name(pkt->packet_type));
    if (g_use_colors) printf("%s", ANSI_RESET);

    // Packet-specific data
    switch (pkt->packet_type) {
        case SRXL2_PKT_HANDSHAKE:
            printf(" src=");
            if (g_use_colors) printf("%s", ANSI_YELLOW);
            printf("0x%02X", pkt->handshake.src_id);
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" dest=");
            if (g_use_colors) printf("%s", ANSI_YELLOW);
            printf("0x%02X", pkt->handshake.dest_id);
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" uid=");
            if (g_use_colors) printf("%s", ANSI_CYAN);
            printf("0x%08X", pkt->handshake.uid);
            if (g_use_colors) printf("%s", ANSI_RESET);
            break;

        case SRXL2_PKT_CONTROL: {
            int8_t rssi = pkt->control.channel.rssi;

            printf(" reply=");
            if (g_use_colors) printf("%s", ANSI_CYAN);
            printf("0x%02X", pkt->control.reply_id);
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" rssi=");
            // Color code RSSI: green=good, yellow=ok, red=poor
            if (g_use_colors) {
                if (rssi < 0) {
                    // dBm: > -70 is good, -70 to -85 is ok, < -85 is poor
                    if (rssi > -70) printf("%s", ANSI_GREEN);
                    else if (rssi > -85) printf("%s", ANSI_YELLOW);
                    else printf("%s", ANSI_RED);
                } else {
                    // Percent: > 70 is good, 40-70 is ok, < 40 is poor
                    if (rssi > 70) printf("%s", ANSI_GREEN);
                    else if (rssi > 40) printf("%s", ANSI_YELLOW);
                    else printf("%s", ANSI_RED);
                }
            }
            printf("%d%s", rssi, rssi < 0 ? "dBm" : "%");
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" ch=");
            if (g_use_colors) printf("%s", ANSI_BLUE);
            printf("%u", pkt->control.channel.num_channels);
            if (g_use_colors) printf("%s", ANSI_RESET);
            break;
        }

        case SRXL2_PKT_TELEMETRY:
            printf(" dest=");
            if (g_use_colors) printf("%s", ANSI_GREEN);
            printf("0x%02X", pkt->telemetry.dest_id);
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" sensor=");
            if (g_use_colors) printf("%s", ANSI_GREEN);
            printf("0x%02X", pkt->telemetry.payload[0]);
            if (g_use_colors) printf("%s", ANSI_RESET);
            break;

        case SRXL2_PKT_BIND:
            printf(" request=");
            if (g_use_colors) printf("%s", ANSI_MAGENTA);
            printf("0x%02X", pkt->bind.request);
            if (g_use_colors) printf("%s", ANSI_RESET);

            printf(" device=");
            if (g_use_colors) printf("%s", ANSI_MAGENTA);
            printf("0x%02X", pkt->bind.device_id);
            if (g_use_colors) printf("%s", ANSI_RESET);
            break;

        default:
            break;
    }

    printf("\n");
    fflush(stdout);
}

///////////////////////////////////////////////////////////////////////////////
// Format: Details (original format)
///////////////////////////////////////////////////////////////////////////////

static void output_details_packet(const srxl2_decoded_pkt_t* pkt, uint64_t timestamp, const char* sender, const uint8_t* data, size_t length)
{
    if (g_use_colors) printf("%s%s", ANSI_BOLD, ANSI_CYAN);
    printf("--------------------------------------------\n");
    printf("  [%llu.%06llu] Packet #%llu (%zu bytes)\n",
           (unsigned long long)(timestamp / 1000000),
           (unsigned long long)(timestamp % 1000000),
           (unsigned long long)g_packet_count,
           length);
    printf("  From: %s\n", sender);
    printf("  Packet Type: 0x%02X (%s)\n", pkt->packet_type,
           srxl2_packet_type_name(pkt->packet_type));
    if (g_use_colors) printf("%s", ANSI_RESET);

    // Decode packet details
    switch (pkt->packet_type) {
        case SRXL2_PKT_HANDSHAKE: {
            if (g_use_colors) printf("%s", ANSI_YELLOW);
            printf("    Source: 0x%02X (%s, Unit %u)\n",
                   pkt->handshake.src_id,
                   srxl2_device_type_name(srxl2_device_type(pkt->handshake.src_id)),
                   srxl2_unit_id(pkt->handshake.src_id));
            printf("    Dest: 0x%02X\n", pkt->handshake.dest_id);
            printf("    Priority: %u\n", pkt->handshake.priority);
            printf("    UID: 0x%08X\n", pkt->handshake.uid);
            if (g_use_colors) printf("%s", ANSI_RESET);
            g_stats.handshake_packets++;
            break;
        }
        case SRXL2_PKT_CONTROL: {
            if (g_use_colors) printf("%s", ANSI_BLUE);
            printf("    Reply ID: 0x%02X\n", pkt->control.reply_id);
            printf("    RSSI: %d %s\n", pkt->control.channel.rssi,
                   pkt->control.channel.rssi < 0 ? "dBm" : "%");
            printf("    Frame Losses: %u\n", pkt->control.channel.frame_losses);
            printf("    Active Channels: %u\n", pkt->control.channel.num_channels);
            if (g_use_colors) printf("%s", ANSI_RESET);
            g_stats.channel_packets++;
            break;
        }
        case SRXL2_PKT_TELEMETRY:
            if (g_use_colors) printf("%s", ANSI_GREEN);
            printf("    Dest: 0x%02X\n", pkt->telemetry.dest_id);
            printf("    Sensor ID: 0x%02X\n", pkt->telemetry.payload[0]);
            if (g_use_colors) printf("%s", ANSI_RESET);
            g_stats.telemetry_packets++;
            break;
        case SRXL2_PKT_BIND:
            g_stats.bind_packets++;
            break;
        case SRXL2_PKT_RSSI:
            g_stats.rssi_packets++;
            break;
        default:
            if (pkt->packet_type == 0x50)
                g_stats.param_packets++;
            else if (pkt->packet_type == 0x99)
                g_stats.internal_packets++;
            else
                g_stats.unknown_packets++;
            break;
    }

    if (g_show_hex) {
        printf("    Hex: ");
        for (size_t i = 0; i < length; i++) {
            printf("%02X ", data[i]);
            if ((i + 1) % 16 == 0 && i < length - 1)
                printf("\n         ");
        }
        printf("\n");
    }

    printf("\n");
    fflush(stdout);
}

///////////////////////////////////////////////////////////////////////////////
// Format: State (ncurses-based live view)
///////////////////////////////////////////////////////////////////////////////

#ifdef HAVE_NCURSES
#include <ncurses.h>

static const char* get_device_icon(uint8_t device_id)
{
    uint8_t type = srxl2_device_type(device_id);
    switch (type) {
        case 0x3: return "[FC]";
        case 0x4: return "[ESC]";
        case 0x2: return "[RX]";
        case 0x1: return "[RRX]";
        case 0x8: return "[VTX]";
        case 0xB: return "[SNS]";
        case 0x6:
        case 0x7: return "[SRV]";
        case 0x9: return "[RF]";
        case 0xA: return "[RID]";
        case 0xF: return "[BRD]";
        case 0x0:
        default:
            return "[???]";
    }
}

static void update_state_display(void)
{
    clear();

    // Header
    attron(A_BOLD | COLOR_PAIR(1));
    mvprintw(0, 0, "SRXL2 Bus Sniffer - Live State View");
    attroff(A_BOLD | COLOR_PAIR(1));

    mvprintw(1, 0, "Press Ctrl+C to exit");
    mvprintw(2, 0, "--------------------------------------------------------------------------------");

    // Statistics
    mvprintw(3, 0, "Total: %llu", (unsigned long long)g_stats.total_packets);
    mvprintw(3, 20, "Handshake: %llu", (unsigned long long)g_stats.handshake_packets);
    mvprintw(3, 40, "Control: %llu", (unsigned long long)g_stats.channel_packets);
    mvprintw(3, 60, "Telemetry: %llu", (unsigned long long)g_stats.telemetry_packets);

    mvprintw(4, 0, "--------------------------------------------------------------------------------");

    // Device list with tree structure
    uint64_t now = get_timestamp_us();
    int row = 5;

    for (int i = 0; i < g_device_count && row < LINES - 2; i++) {
        device_state_t* dev = &g_devices[i];
        uint64_t age_ms = (now - dev->last_seen) / 1000;

        // Device header line with icon
        attron(A_BOLD | COLOR_PAIR(2));
        mvprintw(row, 0, "%s 0x%02X", get_device_icon(dev->device_id), dev->device_id);
        attroff(A_BOLD | COLOR_PAIR(2));

        mvprintw(row, 14, "%-16s", srxl2_device_type_name(srxl2_device_type(dev->device_id)));

        // RSSI with color coding
        if (dev->rssi != 0) {
            if (dev->rssi < 0) {
                // dBm
                if (dev->rssi > -70) attron(COLOR_PAIR(2));  // Green
                else if (dev->rssi > -85) attron(COLOR_PAIR(3));  // Yellow
                else attron(COLOR_PAIR(4));  // Red
                mvprintw(row, 28, "RSSI:%4ddBm", dev->rssi);
            } else {
                // Percent
                if (dev->rssi > 70) attron(COLOR_PAIR(2));
                else if (dev->rssi > 40) attron(COLOR_PAIR(3));
                else attron(COLOR_PAIR(4));
                mvprintw(row, 28, "RSSI:%3d%%", dev->rssi);
            }
            attroff(COLOR_PAIR(2) | COLOR_PAIR(3) | COLOR_PAIR(4));
        }

        mvprintw(row, 42, "Pkts:%llu", (unsigned long long)dev->packet_count);
        mvprintw(row, 54, "Age:%llums", (unsigned long long)age_ms);
        row++;

        // Channel data (only show active channels, up to 10)
        int active_channels = __builtin_popcount(dev->channel_mask);
        if (active_channels > 0) {
            mvprintw(row, 2, "+- Channels (%d active):", active_channels);
            row++;

            int channels_shown = 0;
            for (int ch = 0; ch < 32 && channels_shown < 10; ch++) {
                if (dev->channel_mask & (1 << ch)) {
                    if (channels_shown % 5 == 0) {
                        if (channels_shown > 0) row++;
                        mvprintw(row, 2, "|  ");
                    }
                    mvprintw(row, 5 + (channels_shown % 5) * 14, "CH%02d:%4d",
                             ch + 1, dev->channels[ch]);
                    channels_shown++;
                }
            }
            row++;
        }

        // Telemetry sensors (tree structure)
        if (dev->telemetry_count > 0) {
            mvprintw(row, 2, "+- Telemetry (%d sensors):", dev->telemetry_count);
            row++;

            for (int t = 0; t < dev->telemetry_count && row < LINES - 2; t++) {
                telemetry_sensor_t* sensor = &dev->telemetry[t];
                uint64_t sensor_age_ms = (now - sensor->last_seen) / 1000;

                const char* connector = (t == dev->telemetry_count - 1) ? "  " : "| ";
                mvprintw(row, 5, "%s Sensor 0x%02X: ", connector, sensor->sensor_id);

                // Show first few bytes of telemetry data
                int col = 25;
                for (int b = 0; b < sensor->data_len && b < 8 && col < COLS - 15; b++) {
                    mvprintw(row, col, "%02X ", sensor->raw_data[b]);
                    col += 3;
                }

                mvprintw(row, col, "(%llums)", (unsigned long long)sensor_age_ms);
                row++;
            }
        }

        // Blank line between devices
        row++;
    }

    refresh();
}

static void output_state_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    curs_set(0);

    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_CYAN, COLOR_BLACK);    // Header
        init_pair(2, COLOR_GREEN, COLOR_BLACK);   // Good/Device name
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);  // Warning
        init_pair(4, COLOR_RED, COLOR_BLACK);     // Error/Poor
    }

    update_state_display();
}

static void output_state_cleanup(void)
{
    endwin();
}

// Helper to extract device ID from sender name (e.g., "Battery" -> 0xB1)
static uint8_t extract_device_id_from_sender(const char* sender)
{
    // For simulators, try to infer device ID from name
    if (strstr(sender, "Master") || strstr(sender, "master")) {
        return 0x10;  // Remote receiver master
    } else if (strstr(sender, "Battery") || strstr(sender, "battery")) {
        return 0xB1;  // Sensor (battery) device 1
    }
    // For serial, sender is just "serial", so we can't infer
    return 0;
}

static void output_state_packet_with_sender(const srxl2_decoded_pkt_t* pkt, const char* sender)
{
    uint64_t now = get_timestamp_us();

    // Try to infer sending device from sender name
    uint8_t sender_device_id = extract_device_id_from_sender(sender);

    // Update device state
    if (pkt->packet_type == SRXL2_PKT_HANDSHAKE) {
        device_state_t* dev = find_or_create_device(pkt->handshake.src_id);
        if (dev) {
            dev->uid = pkt->handshake.uid;
            dev->priority = pkt->handshake.priority;
            dev->last_seen = now;
            dev->packet_count++;
        }
    } else if (pkt->packet_type == SRXL2_PKT_CONTROL) {
        // Update from channel data - track the sender (usually master 0x10)

        // Use sender_device_id if available, otherwise assume master 0x10
        uint8_t control_sender = (sender_device_id != 0) ? sender_device_id : 0x10;
        device_state_t* dev = find_or_create_device(control_sender);
        if (dev) {
            dev->rssi = pkt->control.channel.rssi;
            dev->frame_losses = pkt->control.channel.frame_losses;
            dev->channel_mask = pkt->control.channel.mask;
            memcpy(dev->channels, pkt->control.channel.values, sizeof(dev->channels));
            dev->last_seen = now;
            dev->packet_count++;
        }
    } else if (pkt->packet_type == SRXL2_PKT_TELEMETRY) {
        // Track telemetry sender device (battery/sensor sending telemetry)
        if (sender_device_id != 0) {
            device_state_t* sender_dev = find_or_create_device(sender_device_id);
            if (sender_dev) {
                sender_dev->last_seen = now;
                sender_dev->packet_count++;
            }
        }

        // Also track telemetry data for destination device
        device_state_t* dest_dev = find_or_create_device(pkt->telemetry.dest_id);
        if (dest_dev) {
            // Telemetry data is 16 bytes, first byte is sensor ID
            uint8_t sensor_id = pkt->telemetry.payload[0];

            // Find or create sensor
            telemetry_sensor_t* sensor = NULL;
            for (int i = 0; i < dest_dev->telemetry_count; i++) {
                if (dest_dev->telemetry[i].sensor_id == sensor_id) {
                    sensor = &dest_dev->telemetry[i];
                    break;
                }
            }

            if (!sensor && dest_dev->telemetry_count < MAX_TELEMETRY_SENSORS) {
                sensor = &dest_dev->telemetry[dest_dev->telemetry_count++];
                sensor->sensor_id = sensor_id;
            }

            if (sensor) {
                // Copy all 16 bytes of telemetry data
                sensor->data_len = 16;
                memcpy(sensor->raw_data, pkt->telemetry.payload, 16);
                sensor->last_seen = now;
            }

            dest_dev->last_seen = now;
            dest_dev->packet_count++;
        }
    }

    // Update display every 100ms
    static uint64_t last_update = 0;
    if (now - last_update > 100000) {
        update_state_display();
        last_update = now;
    }
}
#else
static void output_state_init(void) {
    fprintf(stderr, "State format requires ncurses support (not compiled in)\n");
    exit(1);
}
static void output_state_cleanup(void) {}
static void output_state_packet_with_sender(const srxl2_decoded_pkt_t* pkt, const char* sender) {
    (void)pkt;
    (void)sender;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Packet Processing
///////////////////////////////////////////////////////////////////////////////

static void process_packet(const uint8_t* data, size_t length, const char* sender)
{
    // Filter internal packets by raw byte before parsing
    if (!g_show_internal && length >= 2 && data[1] == 0x99) {
        return;
    }

    srxl2_decoded_pkt_t pkt;
    srxl2_parse_result_t result = srxl2_pkt_parse(data, (uint8_t)length, &pkt);

    g_packet_count++;
    uint64_t timestamp = get_timestamp_us();

    if (result != SRXL2_PARSE_OK) {
        g_stats.invalid_packets++;
        if (g_format == FORMAT_DETAILS) {
            if (g_use_colors) printf("%s", ANSI_RED);
            printf("[INVALID] %s\n", srxl2_parse_result_name(result));
            if (g_use_colors) printf("%s", ANSI_RESET);
        }
        return;
    }

    g_stats.total_packets++;

    // Output based on format
    switch (g_format) {
        case FORMAT_DETAILS:
            output_details_packet(&pkt, timestamp, sender, data, length);
            break;
        case FORMAT_JSON:
            output_json_packet(&pkt, timestamp, sender, length);
            break;
        case FORMAT_ONELINE:
            output_oneline_packet(&pkt, timestamp, sender);
            break;
        case FORMAT_STATE:
            output_state_packet_with_sender(&pkt, sender);
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

static void print_usage(const char* prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\nTransport Options:\n");
    printf("  -d, --device <name>  Device (bus name or /dev/ttyUSB0)\n");
    printf("  -s, --serial         Use USB-to-serial instead of fakeuart\n");
    printf("  -B, --baud <rate>    Baud rate: 115200 or 400000 (default: 115200)\n");
    printf("  -l, --list-ports     List available serial ports and exit\n");
    printf("\nOutput Options:\n");
    printf("  -f, --format <fmt>   Output format (default: details)\n");
    printf("                       details  - Detailed view with packet contents\n");
    printf("                       json     - JSON output (one per line)\n");
    printf("                       oneline  - Compact one-line per packet\n");
    printf("                       state    - Live state view (ncurses)\n");
    printf("  -x, --hex            Show hex dump (details format only)\n");
    printf("  -n, --no-color       Disable colors\n");
    printf("  -I, --show-internal  Show internal heartbeat packets\n");
    printf("\nExamples:\n");
    printf("  %s -d srxl2bus\n", prog);
    printf("  %s --serial -d /dev/ttyUSB0 -B 115200\n", prog);
    printf("  %s -f json > capture.jsonl\n", prog);
    printf("  %s -f state\n", prog);
}

int main(int argc, char* argv[])
{
    const char* device = "srxl2bus";
    bool use_serial = false;
    transport_baud_t baud = TRANSPORT_BAUD_115200;

    static const struct option long_opts[] = {
        {"device",        required_argument, NULL, 'd'},
        {"serial",        no_argument,       NULL, 's'},
        {"baud",          required_argument, NULL, 'B'},
        {"list-ports",    no_argument,       NULL, 'l'},
        {"format",        required_argument, NULL, 'f'},
        {"hex",           no_argument,       NULL, 'x'},
        {"no-color",      no_argument,       NULL, 'n'},
        {"show-internal", no_argument,       NULL, 'I'},
        {"help",          no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:sB:lf:xnIh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            device = optarg;
            break;
        case 's':
            use_serial = true;
            break;
        case 'B': {
            int rate = atoi(optarg);
            if (rate == 115200) baud = TRANSPORT_BAUD_115200;
            else if (rate == 400000) baud = TRANSPORT_BAUD_400000;
            else {
                fprintf(stderr, "Invalid baud rate (use 115200 or 400000)\n");
                return 1;
            }
            break;
        }
        case 'l':
            transport_list_serial_ports();
            return 0;
        case 'f':
            if (strcmp(optarg, "details") == 0) g_format = FORMAT_DETAILS;
            else if (strcmp(optarg, "json") == 0) g_format = FORMAT_JSON;
            else if (strcmp(optarg, "oneline") == 0) g_format = FORMAT_ONELINE;
            else if (strcmp(optarg, "state") == 0) g_format = FORMAT_STATE;
            else {
                fprintf(stderr, "Invalid format (use: details, json, oneline, state)\n");
                return 1;
            }
            break;
        case 'x':
            g_show_hex = true;
            break;
        case 'n':
            g_use_colors = false;
            break;
        case 'I':
            g_show_internal = true;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    // Initialize transport
    transport_type_t type = use_serial ? TRANSPORT_TYPE_SERIAL : TRANSPORT_TYPE_FAKEUART;
    g_transport = transport_init(type, device, "Sniffer", baud);
    if (!g_transport) {
        fprintf(stderr, "Failed to initialize transport\n");
        return 1;
    }

    transport_set_promiscuous(g_transport, true);

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize output format
    if (g_format == FORMAT_STATE) {
        output_state_init();
    } else if (g_format == FORMAT_DETAILS) {
        printf("=== SRXL2 Bus Sniffer - Details View ===\n");
        printf("\nListening on %s (%s)... Press Ctrl+C to exit\n\n",
               device, use_serial ? "serial" : "fakeuart");
    }

    // Main loop
    uint8_t buffer[SRXL2_MAX_PACKET_SIZE];
    char sender[64];
    bool is_serial = (type == TRANSPORT_TYPE_SERIAL);

    while (g_running) {
        int bytes = transport_receive(g_transport, buffer, sizeof(buffer), 100, sender, sizeof(sender));
        if (bytes <= 0)
            continue;

        if (!is_serial) {
            // Fakeuart: each receive is one complete packet
            process_packet(buffer, bytes, sender);
        } else {
            // Serial: byte stream needs framing
            for (int i = 0; i < bytes; i++) {
                uint8_t b = buffer[i];

                if (!g_frame.synced) {
                    if (b == 0xA6) { // SRXL2_MAGIC
                        g_frame.buf[0] = b;
                        g_frame.len = 1;
                        g_frame.synced = true;
                    }
                    continue;
                }

                if (g_frame.len < SRXL2_MAX_PACKET_SIZE) {
                    g_frame.buf[g_frame.len++] = b;
                } else {
                    // Overflow, resync
                    g_frame.synced = false;
                    g_frame.len = 0;
                    continue;
                }

                // Once we have 3 bytes, we know expected length
                if (g_frame.len >= 3) {
                    uint8_t expected = g_frame.buf[2];
                    if (expected < 5 || expected > SRXL2_MAX_PACKET_SIZE) {
                        // Invalid length, resync
                        g_frame.synced = false;
                        g_frame.len = 0;
                        continue;
                    }
                    if (g_frame.len >= expected) {
                        // Complete packet
                        process_packet(g_frame.buf, expected, sender);
                        g_frame.synced = false;
                        g_frame.len = 0;
                    }
                }
            }
        }
    }

    // Cleanup
    if (g_format == FORMAT_STATE) {
        output_state_cleanup();
    }

    // Print statistics (except for state format)
    if (g_format != FORMAT_STATE && g_format != FORMAT_JSON) {
        printf("\n=== Statistics ===\n");
        printf("  Total packets: %llu\n", (unsigned long long)g_stats.total_packets);
        printf("  Handshake: %llu\n", (unsigned long long)g_stats.handshake_packets);
        printf("  Channel Data: %llu\n", (unsigned long long)g_stats.channel_packets);
        printf("  Telemetry: %llu\n", (unsigned long long)g_stats.telemetry_packets);
        printf("  Invalid: %llu\n", (unsigned long long)g_stats.invalid_packets);
    }

    transport_close(g_transport);
    return 0;
}
