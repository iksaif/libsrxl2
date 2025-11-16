/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Example 1: Simple Logger

This example demonstrates the basic lifecycle hooks to log all master events.
It's useful for understanding the master state machine flow and debugging.

Use case:
- Development and debugging
- Understanding SRXL2 master timing
- Monitoring bus activity
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// Get high-resolution timestamp for profiling
static uint64_t getMicros(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Track timing for each bus
static uint64_t g_lastRunStart[SRXL_NUM_OF_BUSES] = {0};
static uint64_t g_runStartTime[SRXL_NUM_OF_BUSES] = {0};

///////////////////////////////////////////////////////////////////////////////
// LIFECYCLE HOOKS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterRunStart(uint8_t busIndex)
{
    g_runStartTime[busIndex] = getMicros();

    uint64_t delta = g_lastRunStart[busIndex] ?
        (g_runStartTime[busIndex] - g_lastRunStart[busIndex]) : 0;

    printf("[Bus %d] Master cycle start (dt=%.2f ms)\n",
           busIndex, delta / 1000.0);

    g_lastRunStart[busIndex] = g_runStartTime[busIndex];
}

void srxlOnMasterRunEnd(uint8_t busIndex)
{
    uint64_t endTime = getMicros();
    uint64_t duration = endTime - g_runStartTime[busIndex];

    printf("[Bus %d] Master cycle end (duration=%.2f us)\n",
           busIndex, (double)duration);
}

///////////////////////////////////////////////////////////////////////////////
// HANDSHAKE HOOKS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    printf("[Bus %d] ===== HANDSHAKE STARTED =====\n", busIndex);
    printf("[Bus %d] Discovering devices on SRXL bus...\n", busIndex);
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("[Bus %d] ===== HANDSHAKE COMPLETE =====\n", busIndex);
    printf("[Bus %d] Discovered %d device(s) on the bus\n", busIndex, deviceCount);
    printf("[Bus %d] Transitioning to normal operation\n", busIndex);
}

///////////////////////////////////////////////////////////////////////////////
// FRAME TIMING HOOKS
///////////////////////////////////////////////////////////////////////////////

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    // Log every 100th frame to avoid flooding
    if (frameCount % 100 == 0)
    {
        printf("[Bus %d] Frame %u (every 100 frames)\n", busIndex, frameCount);
    }

    // Don't override normal behavior
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// CHANNEL DATA HOOKS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    if (telemDeviceID != 0)
    {
        printf("[Bus %d] Channel data sent, telemetry requested from device 0x%02X\n",
               busIndex, telemDeviceID);
    }
    else
    {
        printf("[Bus %d] Channel data sent (no telemetry requested)\n", busIndex);
    }
}

///////////////////////////////////////////////////////////////////////////////
// TELEMETRY HOOKS
///////////////////////////////////////////////////////////////////////////////

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    if (defaultDeviceID != 0xFF)
    {
        printf("[Bus %d] Selecting telemetry device: 0x%02X\n",
               busIndex, defaultDeviceID);
    }

    // Use default selection
    return defaultDeviceID;
}

void srxlOnMasterSetTelemTx(bool enabled)
{
    printf("[Master] Telemetry TX %s\n", enabled ? "ENABLED" : "DISABLED");
}

void srxlOnMasterTelemSent(void)
{
    printf("[Master] Telemetry packet transmitted over RF\n");
}

void srxlOnMasterSuppressTelem(void* pTelemetryData)
{
    printf("[Master] Suppressing internal telemetry generation\n");
}

///////////////////////////////////////////////////////////////////////////////
// BIND HOOKS
///////////////////////////////////////////////////////////////////////////////

bool srxlOnMasterBind(void* pBindInfo)
{
    printf("[Master] Bind requested (not implemented in this example)\n");
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAM
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("===============================================\n");
    printf("SRXL2 Master Example 1: Simple Logger\n");
    printf("===============================================\n");
    printf("\n");
    printf("This example logs all master events to help you\n");
    printf("understand the SRXL2 master state machine.\n");
    printf("\n");

    // TODO: Initialize SRXL2 library
    // TODO: Initialize as bus master
    // TODO: Main loop calling srxlRun()

    printf("\n");
    printf("Example complete.\n");

    return 0;
}
