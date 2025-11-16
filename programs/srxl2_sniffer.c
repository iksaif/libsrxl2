/*
MIT License

SRXL2 Bus Sniffer

This program captures and decodes all SRXL2 traffic on the bus.
It operates in promiscuous mode to see all packets including its own.

Features:
- Hex dump of all packets
- Decoded packet information
- Statistics tracking
- Color-coded output (optional)

Usage: ./srxl2_sniffer [bus_name] [--hex]
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

// Include SRXL2 library (for decoding only, not as a device)
#include "../SRXL2/Source/spm_srxl.h"

// Include fake UART
#include "../fakeuart/fakeuart.h"

// ANSI Color codes
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"

// Packet type emojis
#define EMOJI_HANDSHAKE  "🤝"
#define EMOJI_CHANNEL    "📡"
#define EMOJI_TELEMETRY  "📊"
#define EMOJI_BIND       "🔗"
#define EMOJI_RSSI       "📶"
#define EMOJI_PARAM      "⚙️"
#define EMOJI_UNKNOWN    "❓"
#define EMOJI_INVALID    "❌"

// Globals
static bool g_running = true;
static bool g_show_hex = false;
static bool g_use_colors = true;
static bool g_show_internal = false;  // By default, hide internal heartbeat packets
static uint64_t g_packet_count = 0;

// Statistics
static struct {
    uint64_t total_packets;
    uint64_t handshake_packets;
    uint64_t channel_packets;
    uint64_t telemetry_packets;
    uint64_t bind_packets;
    uint64_t rssi_packets;
    uint64_t param_packets;
    uint64_t unknown_packets;
    uint64_t invalid_packets;
} g_stats = {0};

///////////////////////////////////////////////////////////////////////////////
// Signal handler
///////////////////////////////////////////////////////////////////////////////

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
    printf("\n[Sniffer] Shutting down...\n");
}

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

static void print_hex(const uint8_t* data, uint8_t length)
{
    printf("    Hex: ");
    for (int i = 0; i < length; ++i)
    {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0 && i < length - 1)
            printf("\n         ");
    }
    printf("\n");
}

static const char* get_device_type_name(uint8_t deviceID)
{
    uint8_t devType = (deviceID >> 4) & 0x0F;
    switch (devType)
    {
        case 0x0: return "None";
        case 0x1: return "Remote Receiver";
        case 0x2: return "Receiver";
        case 0x3: return "Flight Controller";
        case 0x4: return "ESC";
        case 0x6: return "SRXL Servo 1";
        case 0x7: return "SRXL Servo 2";
        case 0x8: return "VTX";
        case 0x9: return "External RF";
        case 0xA: return "Remote ID";
        case 0xB: return "Sensor";
        case 0xF: return "Broadcast";
        default:  return "Unknown";
    }
}

static uint16_t get_crc_from_packet(const uint8_t* packet, uint8_t length)
{
    if (length < 2)
        return 0;
    return (packet[length - 2] << 8) | packet[length - 1];
}

///////////////////////////////////////////////////////////////////////////////
// Packet Decoding
///////////////////////////////////////////////////////////////////////////////

static void decode_handshake(const uint8_t* packet, uint8_t length)
{
    if (length < sizeof(SrxlHandshakePacket))
        return;

    SrxlHandshakePacket* pkt = (SrxlHandshakePacket*)packet;

    if (g_use_colors) printf("%s", COLOR_YELLOW);
    printf("    %s Type: HANDSHAKE\n", EMOJI_HANDSHAKE);
    printf("    Source: 0x%02X (%s)\n",
           pkt->payload.srcDevID, get_device_type_name(pkt->payload.srcDevID));
    printf("    Dest: 0x%02X %s\n",
           pkt->payload.destDevID,
           pkt->payload.destDevID == 0xFF ? "(Broadcast)" :
           pkt->payload.destDevID == 0x00 ? "(Discovery)" : "");
    printf("    Priority: %u\n", pkt->payload.priority);
    printf("    Baud: %s\n",
           pkt->payload.baudSupported & SRXL_BAUD_400000 ? "400000" : "115200");
    printf("    Info: 0x%02X", pkt->payload.info);
    if (pkt->payload.info & SRXL_DEVINFO_TELEM_TX_ENABLED)
        printf(" [TelemTX]");
    if (pkt->payload.info & SRXL_DEVINFO_TELEM_FULL_RANGE)
        printf(" [FullRange]");
    if (pkt->payload.info & SRXL_DEVINFO_FWD_PROG_SUPPORT)
        printf(" [FwdProg]");
    printf("\n");
    printf("    UID: 0x%08X\n", pkt->payload.uid);
    if (g_use_colors) printf("%s", COLOR_RESET);

    g_stats.handshake_packets++;
}

static void decode_channel_data(const uint8_t* packet, uint8_t length)
{
    if (length < SRXL_CTRL_BASE_LENGTH)
        return;

    SrxlControlPacket* pkt = (SrxlControlPacket*)packet;

    const char* cmd_name = "CHANNEL";
    if (pkt->payload.cmd == SRXL_CTRL_CMD_CHANNEL_FS)
        cmd_name = "CHANNEL_FAILSAFE";

    if (g_use_colors) printf("%s", COLOR_BLUE);
    printf("    %s Type: %s\n", EMOJI_CHANNEL, cmd_name);
    printf("    Reply ID: 0x%02X %s\n",
           pkt->payload.replyID,
           pkt->payload.replyID == 0 ? "(No telemetry request)" : "");
    printf("    RSSI: %d %s\n",
           pkt->payload.channelData.rssi,
           pkt->payload.channelData.rssi < 0 ? "dBm" : "%");
    printf("    Frame Losses: %u\n", pkt->payload.channelData.frameLosses);
    printf("    Channel Mask: 0x%08X\n", pkt->payload.channelData.mask);

    // Count active channels
    int channelCount = 0;
    for (int i = 0; i < 32; ++i)
    {
        if (pkt->payload.channelData.mask & (1u << i))
            channelCount++;
    }
    printf("    Active Channels: %d\n", channelCount);

    // Show first few channel values
    if (channelCount > 0 && channelCount <= 8)
    {
        printf("    Values: ");
        int idx = 0;
        for (int i = 0; i < 32 && idx < channelCount; ++i)
        {
            if (pkt->payload.channelData.mask & (1u << i))
            {
                printf("Ch%d=%u ", i, pkt->payload.channelData.values[idx] >> 4);
                idx++;
            }
        }
        printf("\n");
    }
    if (g_use_colors) printf("%s", COLOR_RESET);

    g_stats.channel_packets++;
}

static void decode_telemetry(const uint8_t* packet, uint8_t length)
{
    if (length < sizeof(SrxlTelemetryPacket))
        return;

    SrxlTelemetryPacket* pkt = (SrxlTelemetryPacket*)packet;

    if (g_use_colors) printf("%s", COLOR_GREEN);
    printf("    %s Type: TELEMETRY\n", EMOJI_TELEMETRY);
    printf("    Dest: 0x%02X\n", pkt->destDevID);
    printf("    Sensor ID: 0x%02X\n", pkt->payload.sensorID);

    // Try to decode common telemetry types
    switch (pkt->payload.sensorID)
    {
        case 0x34:  // Flight Pack MAH
        {
            int16_t current = pkt->payload.raw[1] | (pkt->payload.raw[2] << 8);
            int16_t capacity = pkt->payload.raw[3] | (pkt->payload.raw[4] << 8);
            uint16_t temp = pkt->payload.raw[5] | (pkt->payload.raw[6] << 8);

            printf("    🔋 Battery: %.1fA, %dmAh used",
                   current / 10.0f, capacity);
            if (temp != 0x7FFF)
                printf(", %.1f°C", temp / 10.0f);
            printf("\n");
            break;
        }
        default:
            printf("    Data: [");
            for (int i = 0; i < 14; ++i)
                printf("%02X ", pkt->payload.raw[i + 2]);
            printf("]\n");
            break;
    }
    if (g_use_colors) printf("%s", COLOR_RESET);

    g_stats.telemetry_packets++;
}

static void decode_bind(const uint8_t* packet, uint8_t length)
{
    if (length < sizeof(SrxlBindPacket))
        return;

    SrxlBindPacket* pkt = (SrxlBindPacket*)packet;

    if (g_use_colors) printf("%s", COLOR_MAGENTA);
    printf("    %s Type: BIND\n", EMOJI_BIND);
    printf("    Request: 0x%02X ", pkt->request);
    switch (pkt->request)
    {
        case SRXL_BIND_REQ_ENTER: printf("(Enter Bind)"); break;
        case SRXL_BIND_REQ_STATUS: printf("(Request Status)"); break;
        case SRXL_BIND_REQ_BOUND_DATA: printf("(Bound Data Report)"); break;
        case SRXL_BIND_REQ_SET_BIND: printf("(Set Bind)"); break;
        default: printf("(Unknown)"); break;
    }
    printf("\n");
    printf("    Device ID: 0x%02X\n", pkt->deviceID);
    if (g_use_colors) printf("%s", COLOR_RESET);

    g_stats.bind_packets++;
}

static void decode_packet(const uint8_t* packet, uint8_t length, const char* sender)
{
    if (length < 5)
    {
        printf("  %s [INVALID] Packet too short (%u bytes)\n", EMOJI_INVALID, length);
        g_stats.invalid_packets++;
        return;
    }

    // Validate SRXL ID
    if (packet[0] != SPEKTRUM_SRXL_ID)
    {
        printf("  %s [INVALID] Wrong SRXL ID: 0x%02X\n", EMOJI_INVALID, packet[0]);
        g_stats.invalid_packets++;
        return;
    }

    // Validate length
    if (packet[2] != length)
    {
        printf("  %s [INVALID] Length mismatch: header=%u, actual=%u\n",
               EMOJI_INVALID, packet[2], length);
        g_stats.invalid_packets++;
        return;
    }

    uint8_t packetType = packet[1];
    uint16_t crc = get_crc_from_packet(packet, length);

    // Filter out internal packets unless explicitly enabled
    if (!g_show_internal && packetType == SRXL_SPM_INTERNAL)
    {
        return;  // Skip internal packets silently
    }

    if (g_use_colors) printf("%s", COLOR_CYAN);
    printf("  📤 From: %s%s%s\n",
           g_use_colors ? COLOR_BOLD : "", sender, g_use_colors ? COLOR_RESET COLOR_CYAN : "");
    printf("  Packet Type: 0x%02X, Length: %u, CRC: 0x%04X\n",
           packetType, length, crc);
    if (g_use_colors) printf("%s", COLOR_RESET);

    // Decode based on packet type
    switch (packetType)
    {
        case SRXL_HANDSHAKE_ID:
            decode_handshake(packet, length);
            break;

        case SRXL_CTRL_ID:
            decode_channel_data(packet, length);
            break;

        case SRXL_TELEM_ID:
            decode_telemetry(packet, length);
            break;

        case SRXL_BIND_ID:
            decode_bind(packet, length);
            break;

        case SRXL_RSSI_ID:
            if (g_use_colors) printf("%s", COLOR_YELLOW);
            printf("    %s Type: RSSI\n", EMOJI_RSSI);
            if (g_use_colors) printf("%s", COLOR_RESET);
            g_stats.rssi_packets++;
            break;

        case SRXL_PARAM_ID:
            if (g_use_colors) printf("%s", COLOR_CYAN);
            printf("    %s Type: PARAMETER\n", EMOJI_PARAM);
            if (g_use_colors) printf("%s", COLOR_RESET);
            g_stats.param_packets++;
            break;

        case SRXL_SPM_INTERNAL:
            if (g_use_colors) printf("%s", COLOR_MAGENTA);
            printf("    💓 Type: INTERNAL (Heartbeat)\n");
            if (length >= 5 + sizeof(SrxlInternalData))
            {
                printf("    Source: 0x%02X\n", packet[3]);
                printf("    Dest: 0x%02X\n", packet[4]);
                printf("    Test: %u\n", packet[5]);
            }
            if (g_use_colors) printf("%s", COLOR_RESET);
            break;

        default:
            if (g_use_colors) printf("%s", COLOR_RED);
            printf("    %s Type: UNKNOWN (0x%02X)\n", EMOJI_UNKNOWN, packetType);
            if (g_use_colors) printf("%s", COLOR_RESET);
            g_stats.unknown_packets++;
            break;
    }

    if (g_show_hex)
    {
        print_hex(packet, length);
    }

    g_stats.total_packets++;
}

///////////////////////////////////////////////////////////////////////////////
// Main Program
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║       SRXL2 Bus Sniffer                       ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    // Parse arguments
    const char* busName = "srxl2bus";
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--hex") == 0)
        {
            g_show_hex = true;
        }
        else if (strcmp(argv[i], "--no-color") == 0)
        {
            g_use_colors = false;
        }
        else if (strcmp(argv[i], "--show-internal") == 0)
        {
            g_show_internal = true;
        }
        else
        {
            busName = argv[i];
        }
    }

    printf("Configuration:\n");
    printf("  Bus name: %s\n", busName);
    printf("  Hex dump: %s\n", g_show_hex ? "enabled" : "disabled");
    printf("  Colors: %s\n", g_use_colors ? "enabled" : "disabled");
    printf("  Show internal: %s\n", g_show_internal ? "yes" : "no (use --show-internal to enable)");
    printf("\n");

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize fake UART in promiscuous mode
    if (!fakeuart_init(busName, "Sniffer"))
    {
        fprintf(stderr, "Failed to initialize fake UART\n");
        return 1;
    }

    fakeuart_set_promiscuous(true);

    printf("[Sniffer] Listening on bus (press Ctrl+C to exit)\n");
    printf("═══════════════════════════════════════════════\n");
    printf("\n");

    // Receive buffer
    uint8_t buffer[SRXL_MAX_BUFFER_SIZE];
    char sender[64];

    // Main loop
    while (g_running)
    {
        int bytesReceived = fakeuart_receive(buffer, SRXL_MAX_BUFFER_SIZE, 100,
                                            sender, sizeof(sender));

        if (bytesReceived > 0)
        {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);

            if (g_use_colors) printf("%s%s", COLOR_BOLD, COLOR_CYAN);
            printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
            printf("⏱️  [%ld.%06ld] Packet #%llu (%d bytes)\n",
                   ts.tv_sec, ts.tv_nsec / 1000,
                   ++g_packet_count, bytesReceived);
            if (g_use_colors) printf("%s", COLOR_RESET);

            decode_packet(buffer, bytesReceived, sender);
            printf("\n");
        }
    }

    // Print statistics
    printf("═══════════════════════════════════════════════\n");
    printf("Statistics:\n");
    printf("  Total packets: %llu\n", g_stats.total_packets);
    printf("  Handshake: %llu\n", g_stats.handshake_packets);
    printf("  Channel Data: %llu\n", g_stats.channel_packets);
    printf("  Telemetry: %llu\n", g_stats.telemetry_packets);
    printf("  Bind: %llu\n", g_stats.bind_packets);
    printf("  RSSI: %llu\n", g_stats.rssi_packets);
    printf("  Parameter: %llu\n", g_stats.param_packets);
    printf("  Unknown: %llu\n", g_stats.unknown_packets);
    printf("  Invalid: %llu\n", g_stats.invalid_packets);

    fakeuart_close();

    return 0;
}
