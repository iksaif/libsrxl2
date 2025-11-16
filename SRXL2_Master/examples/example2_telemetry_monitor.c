/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Example 2: Telemetry Monitor

This example demonstrates telemetry monitoring and statistics tracking.
It tracks which devices are being polled for telemetry and provides
statistics on telemetry flow.

Use case:
- Monitoring telemetry balance across devices
- Validating priority-based scheduling
- Debugging telemetry issues
- Performance optimization
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// TELEMETRY STATISTICS
///////////////////////////////////////////////////////////////////////////////

#define MAX_DEVICES 16

typedef struct
{
    uint8_t  deviceID;
    uint32_t requestCount;    // How many times this device was requested
    uint32_t responseCount;   // How many times this device responded
    uint32_t lastFrameRequested;
} DeviceTelemStats;

typedef struct
{
    DeviceTelemStats devices[MAX_DEVICES];
    uint8_t deviceCount;
    uint32_t totalRequests;
    uint32_t totalResponses;
    uint32_t currentFrame;
    uint32_t statsReportInterval; // Frames between statistics reports
} TelemMonitor;

static TelemMonitor g_monitor[SRXL_NUM_OF_BUSES];

///////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static DeviceTelemStats* getDeviceStats(uint8_t busIndex, uint8_t deviceID)
{
    TelemMonitor* pMon = &g_monitor[busIndex];

    // Find existing entry
    for (uint8_t i = 0; i < pMon->deviceCount; ++i)
    {
        if (pMon->devices[i].deviceID == deviceID)
        {
            return &pMon->devices[i];
        }
    }

    // Add new entry
    if (pMon->deviceCount < MAX_DEVICES)
    {
        DeviceTelemStats* pStats = &pMon->devices[pMon->deviceCount++];
        pStats->deviceID = deviceID;
        pStats->requestCount = 0;
        pStats->responseCount = 0;
        pStats->lastFrameRequested = 0;
        return pStats;
    }

    return NULL;
}

static void printStatistics(uint8_t busIndex)
{
    TelemMonitor* pMon = &g_monitor[busIndex];

    printf("\n");
    printf("========================================\n");
    printf("Telemetry Statistics - Bus %d\n", busIndex);
    printf("========================================\n");
    printf("Total Frames: %u\n", pMon->currentFrame);
    printf("Total Requests: %u\n", pMon->totalRequests);
    printf("Total Responses: %u\n", pMon->totalResponses);
    printf("\n");
    printf("Per-Device Statistics:\n");
    printf("----------------------------------------\n");
    printf("Device ID | Requests | Responses | Rate    | Last Frame\n");
    printf("----------|----------|-----------|---------|------------\n");

    for (uint8_t i = 0; i < pMon->deviceCount; ++i)
    {
        DeviceTelemStats* pStats = &pMon->devices[i];
        float rate = pMon->totalRequests > 0 ?
            (100.0f * pStats->requestCount / pMon->totalRequests) : 0.0f;

        printf("  0x%02X    | %8u | %9u | %6.2f%% | %10u\n",
               pStats->deviceID,
               pStats->requestCount,
               pStats->responseCount,
               rate,
               pStats->lastFrameRequested);
    }

    printf("----------------------------------------\n");
    printf("\n");
}

///////////////////////////////////////////////////////////////////////////////
// HOOK IMPLEMENTATIONS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    printf("[Bus %d] Handshake starting - resetting statistics\n", busIndex);
    memset(&g_monitor[busIndex], 0, sizeof(TelemMonitor));
    g_monitor[busIndex].statsReportInterval = 500; // Report every 500 frames (~5.5 seconds)
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("[Bus %d] Handshake complete - %d devices found\n", busIndex, deviceCount);
    printf("[Bus %d] Starting telemetry monitoring...\n", busIndex);
}

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    TelemMonitor* pMon = &g_monitor[busIndex];
    pMon->currentFrame = frameCount;

    // Print statistics periodically
    if (frameCount > 0 && (frameCount % pMon->statsReportInterval) == 0)
    {
        printStatistics(busIndex);
    }

    return false; // Don't override normal behavior
}

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    if (telemDeviceID == 0 || telemDeviceID == 0xFF)
        return; // No telemetry requested

    TelemMonitor* pMon = &g_monitor[busIndex];
    DeviceTelemStats* pStats = getDeviceStats(busIndex, telemDeviceID);

    if (pStats)
    {
        pStats->requestCount++;
        pStats->lastFrameRequested = pMon->currentFrame;
        pMon->totalRequests++;
    }
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    // Just track the selection, don't modify it
    return defaultDeviceID;
}

void srxlOnMasterTelemSent(void)
{
    // Note: In a real implementation, you'd need to know which device sent telemetry
    // For now, just count it globally
    printf("[Telemetry] Packet transmitted over RF\n");
}

void srxlOnMasterSuppressTelem(void* pTelemetryData)
{
    // Could track received telemetry here
    (void)pTelemetryData;
}

///////////////////////////////////////////////////////////////////////////////
// PLACEHOLDER HOOKS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterRunStart(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterRunEnd(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterSetTelemTx(bool enabled) { (void)enabled; }
bool srxlOnMasterBind(void* pBindInfo) { (void)pBindInfo; return false; }
bool srxlOnMasterParseInternal(void* pInternal) { (void)pInternal; return false; }
uint8_t srxlOnMasterFillInternal(void* pInternal) { (void)pInternal; return 0; }

///////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAM
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("===============================================\n");
    printf("SRXL2 Master Example 2: Telemetry Monitor\n");
    printf("===============================================\n");
    printf("\n");
    printf("This example tracks telemetry requests and\n");
    printf("provides statistics on telemetry flow.\n");
    printf("\n");

    // TODO: Initialize SRXL2 library
    // TODO: Initialize as bus master
    // TODO: Main loop calling srxlRun()

    // Print final statistics
    printf("\nFinal Statistics:\n");
    for (uint8_t bus = 0; bus < SRXL_NUM_OF_BUSES; ++bus)
    {
        if (g_monitor[bus].deviceCount > 0)
        {
            printStatistics(bus);
        }
    }

    return 0;
}
