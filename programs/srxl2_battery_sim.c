/*
MIT License

SRXL2 Battery Device Simulator

This program simulates an SRXL2 battery telemetry device that:
- Responds to handshake requests
- Sends battery telemetry when requested
- Simulates realistic battery voltage and current

Usage: ./srxl2_battery_sim [bus_name]
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>

// Windows-style type definitions for Spektrum telemetry headers
typedef uint8_t  UINT8;
typedef int8_t   INT8;
typedef uint16_t UINT16;
typedef int16_t  INT16;
typedef uint32_t UINT32;
typedef int32_t  INT32;
typedef uint64_t UINT64;
typedef int64_t  INT64;
typedef float    FP32;
typedef double   FP64;

// Include SRXL2 library
#include "../SRXL2/Source/spm_srxl.h"

// Include fake UART
#include "../fakeuart/fakeuart.h"

// Include Spektrum telemetry structures
#include "../SpektrumDocumentation/Telemetry/spektrumTelemetrySensors.h"

// Globals
static bool g_running = true;
static uint8_t g_rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
static uint8_t g_rxBufferIndex = 0;
static uint32_t g_telemSentCount = 0;
static uint32_t g_handshakeCount = 0;
static uint32_t g_channelDataCount = 0;
static bool g_masterFound = false;

// Battery simulation state
static struct {
    float voltage;           // Volts
    float current;           // Amps
    float capacity_used;     // mAh
    float capacity_total;    // mAh
    float temperature;       // Celsius
    uint32_t time_ms;        // Time since start
} g_battery = {
    .voltage = 16.8,        // 4S LiPo fully charged
    .current = 5.0,         // 5A draw
    .capacity_used = 0.0,
    .capacity_total = 5000.0,  // 5000mAh battery
    .temperature = 25.0,
    .time_ms = 0
};

///////////////////////////////////////////////////////////////////////////////
// Signal handler
///////////////////////////////////////////////////////////////////////////////

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
    printf("\n[Battery] Shutting down...\n");
}

///////////////////////////////////////////////////////////////////////////////
// Battery Simulation
///////////////////////////////////////////////////////////////////////////////

static void update_battery(uint32_t deltaMs)
{
    g_battery.time_ms += deltaMs;

    // Discharge based on current draw
    float hours = deltaMs / (1000.0f * 3600.0f);
    g_battery.capacity_used += g_battery.current * hours * 1000.0f;

    // Update voltage based on discharge (simple linear model)
    float remaining_pct = 1.0f - (g_battery.capacity_used / g_battery.capacity_total);
    if (remaining_pct < 0.0f) remaining_pct = 0.0f;

    // LiPo voltage range: 3.0V to 4.2V per cell (4S = 12.0V to 16.8V)
    g_battery.voltage = 12.0f + (remaining_pct * 4.8f);

    // Add some realistic variation
    g_battery.voltage += (sinf(g_battery.time_ms / 1000.0f) * 0.1f);

    // Current varies slightly
    g_battery.current = 5.0f + (sinf(g_battery.time_ms / 500.0f) * 1.0f);
    if (g_battery.current < 0.0f) g_battery.current = 0.0f;

    // Temperature rises slightly with use
    g_battery.temperature = 25.0f + (g_battery.capacity_used / g_battery.capacity_total) * 15.0f;
}

///////////////////////////////////////////////////////////////////////////////
// SRXL2 User Callbacks
///////////////////////////////////////////////////////////////////////////////

void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry)
{
    // Create battery telemetry packet using Flight Pack MAH format
    STRU_TELE_FP_MAH* pBatt = (STRU_TELE_FP_MAH*)pTelemetry->raw;

    memset(pTelemetry->raw, 0, 16);

    pBatt->identifier = TELE_DEVICE_FP_MAH;  // 0x34
    pBatt->sID = 0;  // Secondary ID

    // Convert current to telemetry format (0.1A resolution)
    pBatt->current_A = (int16_t)(g_battery.current * 10.0f);

    // Capacity used (1mAh resolution)
    pBatt->chargeUsed_A = (int16_t)(g_battery.capacity_used);

    // Temperature (0.1°C resolution, 0x7FFF = not populated)
    pBatt->temp_A = (uint16_t)(g_battery.temperature * 10.0f);

    // Battery B not used
    pBatt->current_B = 0x7FFF;  // No data
    pBatt->chargeUsed_B = 0x7FFF;  // No data
    pBatt->temp_B = 0x7FFF;  // No data
    pBatt->spare = 0;

    g_telemSentCount++;

    // Print status every 50 telemetry packets (~5.5 seconds at 11ms frame rate)
    if (g_telemSentCount % 50 == 0)
    {
        float remaining_pct = 100.0f * (1.0f - g_battery.capacity_used / g_battery.capacity_total);
        printf("[Battery] Telem #%u: %.2fV, %.2fA, %.0f/%.0fmAh (%.1f%% left), %.1f°C\n",
               g_telemSentCount, g_battery.voltage, g_battery.current,
               g_battery.capacity_used, g_battery.capacity_total,
               remaining_pct, g_battery.temperature);
    }
}

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafeData)
{
    printf("userProvidedReceivedChannelData: pChannelData = %p, isFailsafeData = %d\n", pChannelData, isFailsafeData);
    printf("pChannelData->chData[0] = %x\n", pChannelData->values[0]);
    printf("pChannelData->chData[1] = %x\n", pChannelData->values[1]);
}

void userProvidedHandleVtxData(SrxlVtxData* pVtxData)
{
    (void)pVtxData;
}

///////////////////////////////////////////////////////////////////////////////
// Main Program
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║       SRXL2 Battery Simulator                 ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    // Parse arguments
    const char* busName = (argc > 1) ? argv[1] : "srxl2bus";

    printf("Configuration:\n");
    printf("  Bus name: %s\n", busName);
    printf("  Device ID: 0xB1 (Sensor, Unit 1)\n");
    printf("  Role: Telemetry Device (Slave)\n");
    printf("  Battery: 4S LiPo, %.0f mAh\n", g_battery.capacity_total);
    printf("\n");

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize fake UART
    if (!fakeuart_init(busName, "Battery"))
    {
        fprintf(stderr, "Failed to initialize fake UART\n");
        return 1;
    }

    // Initialize SRXL2 as sensor device (device ID 0xB1 = slave device)
    // Unit ID must be non-zero (1-15) to act as slave, not master
    uint32_t uniqueID = 0xAABBCCDD;
    if (!srxlInitDevice(0xB1, 30, SRXL_DEVINFO_NO_RF, uniqueID))
    {
        fprintf(stderr, "Failed to initialize SRXL2 device\n");
        fakeuart_close();
        return 1;
    }

    if (!srxlInitBus(0, 0, SRXL_BAUD_115200))
    {
        fprintf(stderr, "Failed to initialize SRXL2 bus\n");
        fakeuart_close();
        return 1;
    }

    printf("[Battery] Initialized successfully\n");
    printf("[Battery] Waiting for bus master (press Ctrl+C to exit)\n");
    printf("\n");

    // Main loop
    struct timespec lastUpdate;
    clock_gettime(CLOCK_MONOTONIC, &lastUpdate);

    while (g_running)
    {
        // Try to receive UART data
        int bytesReceived = fakeuart_receive(&g_rxBuffer[g_rxBufferIndex],
                                            SRXL_MAX_BUFFER_SIZE, 5, NULL, 0);  // 5ms timeout

        if (bytesReceived > 0)
        {
            g_rxBufferIndex += bytesReceived;

            // Check if we have a complete packet
            if (g_rxBufferIndex >= 5 && g_rxBuffer[0] == SPEKTRUM_SRXL_ID)
            {
                uint8_t packetLength = g_rxBuffer[2];

                if (g_rxBufferIndex >= packetLength)
                {
                    // Check packet type for debug output
                    uint8_t packetType = g_rxBuffer[1];

                    // Try to parse packet (this will call srxlRun internally)
                    if (srxlParsePacket(0, g_rxBuffer, packetLength))
                    {
                        // Packet parsed successfully
                        if (packetType == SRXL_HANDSHAKE_ID)  // 0x21 = Handshake
                        {
                            g_handshakeCount++;
                            if (!g_masterFound)
                            {
                                printf("[Battery] Master discovered! Handshake received from device 0x%02X\n",
                                       g_rxBuffer[3]);
                                g_masterFound = true;
                            }
                        }
                        else if (packetType == SRXL_CTRL_ID)  // 0xCD = Control/Channel Data
                        {
                            g_channelDataCount++;
                            if (g_channelDataCount == 1)
                            {
                                printf("[Battery] Receiving channel data from master\n");
                            }
                            else if (g_channelDataCount % 500 == 0)
                            {
                                printf("[Battery] Channel data count: %u\n", g_channelDataCount);
                            }
                        }
                    } else {
                        printf("[Battery] Failed to parse packet\n");
                    }

                    // Remove packet from buffer
                    g_rxBufferIndex -= packetLength;
                    if (g_rxBufferIndex > 0)
                    {
                        memmove(g_rxBuffer, &g_rxBuffer[packetLength], g_rxBufferIndex);
                    }
                }
            }
            else if (g_rxBufferIndex > 0 && g_rxBuffer[0] != SPEKTRUM_SRXL_ID)
            {
                // Invalid data, reset
                g_rxBufferIndex = 0;
            }
        }
        else
        {
            // Assume 5ms because it's our read timeout.
            srxlRun(0, 5);
        }

        // Update battery simulation
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);

        long elapsedMs = (currentTime.tv_sec - lastUpdate.tv_sec) * 1000 +
                        (currentTime.tv_nsec - lastUpdate.tv_nsec) / 1000000;

        if (elapsedMs >= 100)  // Update every 100ms
        {
            update_battery(elapsedMs);
            lastUpdate = currentTime;
        }
    }

    // Cleanup
    fakeuart_stats_t stats;
    fakeuart_get_stats(&stats);

    printf("\n");
    printf("Statistics:\n");
    printf("  TX: %llu packets, %llu bytes\n", stats.tx_packets, stats.tx_bytes);
    printf("  RX: %llu packets, %llu bytes\n", stats.rx_packets, stats.rx_bytes);
    printf("  Handshakes received: %u\n", g_handshakeCount);
    printf("  Channel data received: %u\n", g_channelDataCount);
    printf("  Telemetry sent: %u\n", g_telemSentCount);
    printf("\n");
    printf("Final battery state:\n");
    printf("  Voltage: %.2f V\n", g_battery.voltage);
    printf("  Current: %.2f A\n", g_battery.current);
    printf("  Capacity used: %.0f / %.0f mAh (%.1f%%)\n",
           g_battery.capacity_used, g_battery.capacity_total,
           (g_battery.capacity_used / g_battery.capacity_total) * 100.0f);
    printf("  Temperature: %.1f °C\n", g_battery.temperature);

    fakeuart_close();

    return 0;
}
