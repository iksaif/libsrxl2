/*
MIT License

SRXL2 Master Simulator

This program simulates an SRXL2 bus master (remote receiver) that:
- Performs device discovery
- Sends channel data every 11ms
- Polls devices for telemetry
- Responds to bind requests

Usage: ./srxl2_master_sim [bus_name]
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

// Include SRXL2 library (SRXL_INCLUDE_MASTER_CODE is defined by CMake)
#include "../SRXL2/Source/spm_srxl.h"
#include "../SRXL2_Master/spm_srxl_master.h"

// Include fake UART
#include "../fakeuart/fakeuart.h"

// Globals
static bool g_running = true;
static uint8_t g_rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
static uint8_t g_rxBufferIndex = 0;
static uint16_t g_frameCount = 0;

// Channel data (simulated RC inputs)
static uint16_t g_channels[16] = {
    32768, 32768, 16384, 32768,  // Throttle, Aileron, Elevator, Rudder
    32768, 32768, 32768, 32768,
    32768, 32768, 32768, 32768,
    32768, 32768, 32768, 32768
};

///////////////////////////////////////////////////////////////////////////////
// Signal handler
///////////////////////////////////////////////////////////////////////////////

static void signal_handler(int sig)
{
    (void)sig;
    g_running = false;
    printf("\n[Master] Shutting down...\n");
}

///////////////////////////////////////////////////////////////////////////////
// SRXL2 User Callbacks
///////////////////////////////////////////////////////////////////////////////

void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry)
{
    // Masters typically don't send telemetry, but we can implement if needed
    (void)pTelemetry;
}

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafeData)
{
    // Masters don't receive channel data
    (void)pChannelData;
    (void)isFailsafeData;
}

// Track telemetry reception
static uint32_t g_telemRecvCount = 0;

void userProvidedHandleVtxData(SrxlVtxData* pVtxData)
{
    (void)pVtxData;
}

///////////////////////////////////////////////////////////////////////////////
// SRXL2 Master Hooks
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    printf("[Master] Starting device discovery on bus %d...\n", busIndex);
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("[Master] *** DISCOVERY COMPLETE: %d device(s) found on bus %d ***\n",
           deviceCount, busIndex);
    printf("[Master] Transitioning to normal operation...\n");
}

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    (void)busIndex;
    (void)frameCount;

    // Update simulated channel data (slight movement for demonstration)
    static int direction = 1;
    if (frameCount % 50 == 0)
    {
        g_channels[1] += 100 * direction;  // Aileron
        if (g_channels[1] > 40000 || g_channels[1] < 25000)
            direction = -direction;
    }

    return false;  // Use normal behavior
}

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    (void)busIndex;

    // Print status every 500 frames (~5.5 seconds)
    if (g_frameCount % 500 == 0 && g_frameCount > 0)
    {
        printf("[Master] Frame %u: Ch data sent, telem device=0x%02X, telem recv=%u\n",
               g_frameCount, telemDeviceID, g_telemRecvCount);
    }
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    (void)busIndex;
    return defaultDeviceID;  // Use default selection
}

void srxlOnMasterRunStart(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterRunEnd(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterSetTelemTx(bool enabled) { (void)enabled; }
void srxlOnMasterTelemSent(void) { }
void srxlOnMasterSuppressTelem(void* pTelemetryData) { (void)pTelemetryData; }
bool srxlOnMasterBind(void* pBindInfo) { (void)pBindInfo; return false; }
bool srxlOnMasterParseInternal(void* pInternal) { (void)pInternal; return false; }

uint8_t srxlOnMasterFillInternal(void* pInternal)
{
    // Fill internal packet for heartbeat
    SrxlInternalData* pData = (SrxlInternalData*)pInternal;

    // Set key to 0 for simple heartbeat
    pData->key = 0;

    // Return packet length (header + payload + CRC)
    return sizeof(SrxlHeader) + sizeof(SrxlInternalData) + 2;
}

///////////////////////////////////////////////////////////////////////////////
// Main Program
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║       SRXL2 Master Simulator                  ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    // Parse arguments
    const char* busName = (argc > 1) ? argv[1] : "srxl2bus";

    printf("Configuration:\n");
    printf("  Bus name: %s\n", busName);
    printf("  Device ID: 0x10 (Remote Receiver)\n");
    printf("  Role: Bus Master\n");
    printf("\n");

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize fake UART
    if (!fakeuart_init(busName, "Master"))
    {
        fprintf(stderr, "Failed to initialize fake UART\n");
        return 1;
    }

    // Initialize SRXL2 as master (device ID 0x10 = remote receiver)
    uint32_t uniqueID = 0x12345678;  // Random unique ID
    if (!srxlInitDevice(0x10, 20, SRXL_DEVINFO_TELEM_TX_ENABLED, uniqueID))
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

    printf("[Master] Initialized successfully\n");
    printf("[Master] Starting main loop (press Ctrl+C to exit)\n");
    printf("\n");

    // Initialize channel data
    for (int i = 0; i < 16; ++i)
    {
        srxlChData.values[i] = g_channels[i];
    }
    srxlChData.mask = 0x0000FFFF;  // First 16 channels active
    srxlChData.rssi = -50;  // Good signal
    srxlChData.frameLosses = 0;

    // Main loop
    struct timespec lastFrameTime;
    clock_gettime(CLOCK_MONOTONIC, &lastFrameTime);
    while (g_running)
    {
        // Try to receive UART data, don't block too long to allow the state machine to run and to keep the channel active.
        // According to the spec timing requirements:
        // - 115200 baud: wait ~2.5-3 ms for telemetry response
        // - 400000 baud: wait ~0.8-1 ms for telemetry response
        // So we should wait for 1ms.
        int bytesReceived = fakeuart_receive(&g_rxBuffer[g_rxBufferIndex],
                                            SRXL_MAX_BUFFER_SIZE, 1, NULL, 0);  // 1ms timeout

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

                    // Try to parse packet
                    if (srxlParsePacket(0, g_rxBuffer, packetLength))
                    {
                        // Packet parsed successfully
                        if (packetType == SRXL_TELEM_ID)  // 0x80 = Telemetry
                        {
                            g_telemRecvCount++;
                            if (g_telemRecvCount % 20 == 0)
                            {
                                printf("[Master] Received telemetry packet #%u (type=0x%02X)\n",
                                       g_telemRecvCount, g_rxBuffer[4]);  // Telemetry device ID
                            }
                        }
                        else if (packetType == SRXL_HANDSHAKE_ID)  // 0x21 = Handshake
                        {
                            static uint32_t handshake_count = 0;
                            handshake_count++;
                            printf("[Master] Received handshake #%u from device 0x%02X (src=0x%02X, dst=0x%02X)\n",
                                   handshake_count, g_rxBuffer[3], g_rxBuffer[3], g_rxBuffer[4]);
                        }

                        // Run the state machine to handle the packet. Reset timeout.
                        srxlRun(0, 0);
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

        // Check if it's time for next frame (11ms)
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC, &currentTime);

        long elapsedMs = (currentTime.tv_sec - lastFrameTime.tv_sec) * 1000 +
                        (currentTime.tv_nsec - lastFrameTime.tv_nsec) / 1000000;

        // Every 10 seconds update the channel data and send it.
        if (elapsedMs >= 10000)
        {
            // Update channel data with latest values every frame
            for (int i = 0; i < 16; ++i)
            {
                srxlChData.values[i] = g_channels[i];
            }
            // Mark the channels that have changed as outgoing
            printf("[Master] Frame %u: Sending channel data (every 10 seconds)\n", g_frameCount);
            srxlSetOutgoingChannelMask(0xFFFFFFFF);

            lastFrameTime = currentTime;
            
            g_frameCount++;
        }

        // Run SRXL state machine with the elapsed time
        // This allows the state machine to timeout properly during handshake, and to send channel data if needed.
        srxlRun(0, 1);
    }

    // Cleanup
    fakeuart_stats_t stats;
    fakeuart_get_stats(&stats);

    printf("\n");
    printf("Statistics:\n");
    printf("  TX: %llu packets, %llu bytes\n", stats.tx_packets, stats.tx_bytes);
    printf("  RX: %llu packets, %llu bytes\n", stats.rx_packets, stats.rx_bytes);
    printf("  Frames sent: %u\n", g_frameCount);
    printf("  Telemetry received: %u\n", g_telemRecvCount);

    fakeuart_close();

    return 0;
}
