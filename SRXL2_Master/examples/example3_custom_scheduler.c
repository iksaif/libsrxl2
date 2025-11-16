/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Example 3: Custom Telemetry Scheduler

This example demonstrates how to implement custom telemetry scheduling
logic by overriding the device selection hook. It shows three different
scheduling strategies:

1. Round-robin: Poll devices in sequence
2. Fixed priority: Always prioritize certain devices
3. Adaptive: Increase priority for devices that haven't responded recently

Use case:
- Custom telemetry requirements
- Prioritizing critical sensors
- Implementing application-specific scheduling
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
///////////////////////////////////////////////////////////////////////////////

// Choose scheduling strategy
typedef enum
{
    SCHED_ROUND_ROBIN,      // Simple round-robin
    SCHED_FIXED_PRIORITY,   // Fixed device priorities
    SCHED_ADAPTIVE,         // Adaptive based on response rates
} SchedulingStrategy;

static SchedulingStrategy g_strategy = SCHED_ROUND_ROBIN;

///////////////////////////////////////////////////////////////////////////////
// SCHEDULER STATE
///////////////////////////////////////////////////////////////////////////////

#define MAX_DEVICES 16

typedef struct
{
    uint8_t deviceID;
    uint8_t fixedPriority;      // User-defined priority (0-100)
    uint32_t requestCount;
    uint32_t noResponseCount;   // Consecutive frames with no response
    uint32_t adaptivePriority;  // Calculated priority for adaptive mode
} DeviceInfo;

typedef struct
{
    DeviceInfo devices[MAX_DEVICES];
    uint8_t deviceCount;
    uint8_t lastSelectedIndex;
    uint32_t frameCount;
} Scheduler;

static Scheduler g_sched[SRXL_NUM_OF_BUSES];

///////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static DeviceInfo* findDevice(uint8_t busIndex, uint8_t deviceID)
{
    Scheduler* pSched = &g_sched[busIndex];

    for (uint8_t i = 0; i < pSched->deviceCount; ++i)
    {
        if (pSched->devices[i].deviceID == deviceID)
        {
            return &pSched->devices[i];
        }
    }

    return NULL;
}

static void addDevice(uint8_t busIndex, uint8_t deviceID)
{
    Scheduler* pSched = &g_sched[busIndex];

    if (findDevice(busIndex, deviceID))
        return; // Already exists

    if (pSched->deviceCount >= MAX_DEVICES)
        return; // No room

    DeviceInfo* pDev = &pSched->devices[pSched->deviceCount++];
    pDev->deviceID = deviceID;
    pDev->requestCount = 0;
    pDev->noResponseCount = 0;
    pDev->adaptivePriority = 50; // Default medium priority

    // Assign fixed priorities based on device type
    uint8_t devType = (deviceID >> 4) & 0x0F;
    switch (devType)
    {
        case 0x0: // Remote receiver - high priority
        case 0x1:
        case 0x2:
            pDev->fixedPriority = 80;
            break;
        case 0x3: // Flight controller - medium priority
            pDev->fixedPriority = 50;
            break;
        case 0x4: // ESC - medium-low priority
            pDev->fixedPriority = 30;
            break;
        case 0xB: // Sensor - low priority
            pDev->fixedPriority = 20;
            break;
        default:
            pDev->fixedPriority = 50;
            break;
    }

    printf("[Scheduler] Added device 0x%02X with fixed priority %d\n",
           deviceID, pDev->fixedPriority);
}

///////////////////////////////////////////////////////////////////////////////
// SCHEDULING ALGORITHMS
///////////////////////////////////////////////////////////////////////////////

static uint8_t scheduleRoundRobin(uint8_t busIndex)
{
    Scheduler* pSched = &g_sched[busIndex];

    if (pSched->deviceCount == 0)
        return 0xFF;

    // Select next device in sequence
    pSched->lastSelectedIndex = (pSched->lastSelectedIndex + 1) % pSched->deviceCount;
    return pSched->devices[pSched->lastSelectedIndex].deviceID;
}

static uint8_t scheduleFixedPriority(uint8_t busIndex)
{
    Scheduler* pSched = &g_sched[busIndex];

    if (pSched->deviceCount == 0)
        return 0xFF;

    // Find device with highest fixed priority that hasn't been selected recently
    uint8_t bestIndex = 0;
    uint32_t bestScore = 0;

    for (uint8_t i = 0; i < pSched->deviceCount; ++i)
    {
        // Score = priority * (frames since last request + 1)
        uint32_t framesSince = pSched->frameCount - pSched->devices[i].requestCount;
        uint32_t score = pSched->devices[i].fixedPriority * (framesSince + 1);

        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = i;
        }
    }

    return pSched->devices[bestIndex].deviceID;
}

static uint8_t scheduleAdaptive(uint8_t busIndex)
{
    Scheduler* pSched = &g_sched[busIndex];

    if (pSched->deviceCount == 0)
        return 0xFF;

    // Update adaptive priorities based on response behavior
    for (uint8_t i = 0; i < pSched->deviceCount; ++i)
    {
        DeviceInfo* pDev = &pSched->devices[i];

        // Increase priority if device hasn't responded recently
        if (pDev->noResponseCount > 10)
        {
            // Significantly boost priority for non-responsive devices
            pDev->adaptivePriority = pDev->fixedPriority +
                (pDev->noResponseCount - 10) * 2;
            if (pDev->adaptivePriority > 100)
                pDev->adaptivePriority = 100;
        }
        else
        {
            // Gradually return to fixed priority
            if (pDev->adaptivePriority > pDev->fixedPriority)
                pDev->adaptivePriority--;
        }
    }

    // Select device with highest adaptive priority
    uint8_t bestIndex = 0;
    uint32_t bestPriority = 0;

    for (uint8_t i = 0; i < pSched->deviceCount; ++i)
    {
        if (pSched->devices[i].adaptivePriority > bestPriority)
        {
            bestPriority = pSched->devices[i].adaptivePriority;
            bestIndex = i;
        }
    }

    return pSched->devices[bestIndex].deviceID;
}

///////////////////////////////////////////////////////////////////////////////
// HOOK IMPLEMENTATIONS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    printf("[Bus %d] Handshake started - clearing device list\n", busIndex);
    memset(&g_sched[busIndex], 0, sizeof(Scheduler));
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("[Bus %d] Handshake complete - %d devices\n", busIndex, deviceCount);

    const char* strategyName[] = {
        "Round-Robin",
        "Fixed Priority",
        "Adaptive"
    };
    printf("[Bus %d] Using %s scheduling\n", busIndex, strategyName[g_strategy]);
}

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    g_sched[busIndex].frameCount = frameCount;

    // Print scheduler status every 100 frames
    if (frameCount % 100 == 0 && frameCount > 0)
    {
        Scheduler* pSched = &g_sched[busIndex];
        printf("[Frame %u] Scheduler status: %d devices\n",
               frameCount, pSched->deviceCount);
    }

    return false;
}

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    if (telemDeviceID == 0 || telemDeviceID == 0xFF)
        return;

    // Track the request
    DeviceInfo* pDev = findDevice(busIndex, telemDeviceID);
    if (!pDev)
    {
        // First time seeing this device, add it
        addDevice(busIndex, telemDeviceID);
        pDev = findDevice(busIndex, telemDeviceID);
    }

    if (pDev)
    {
        pDev->requestCount++;
        pDev->noResponseCount++;
    }
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    // Override default selection with custom strategy
    uint8_t selectedID = defaultDeviceID;

    switch (g_strategy)
    {
        case SCHED_ROUND_ROBIN:
            selectedID = scheduleRoundRobin(busIndex);
            break;

        case SCHED_FIXED_PRIORITY:
            selectedID = scheduleFixedPriority(busIndex);
            break;

        case SCHED_ADAPTIVE:
            selectedID = scheduleAdaptive(busIndex);
            break;
    }

    return selectedID;
}

void srxlOnMasterSuppressTelem(void* pTelemetryData)
{
    // When telemetry is received, reset no-response counter
    // (In a real implementation, you'd need to know which device sent it)
    (void)pTelemetryData;
}

///////////////////////////////////////////////////////////////////////////////
// PLACEHOLDER HOOKS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterRunStart(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterRunEnd(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterSetTelemTx(bool enabled) { (void)enabled; }
void srxlOnMasterTelemSent(void) { }
bool srxlOnMasterBind(void* pBindInfo) { (void)pBindInfo; return false; }
bool srxlOnMasterParseInternal(void* pInternal) { (void)pInternal; return false; }
uint8_t srxlOnMasterFillInternal(void* pInternal) { (void)pInternal; return 0; }

///////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAM
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    printf("===============================================\n");
    printf("SRXL2 Master Example 3: Custom Scheduler\n");
    printf("===============================================\n");
    printf("\n");

    // Parse command line for strategy selection
    if (argc > 1)
    {
        if (strcmp(argv[1], "rr") == 0)
            g_strategy = SCHED_ROUND_ROBIN;
        else if (strcmp(argv[1], "fixed") == 0)
            g_strategy = SCHED_FIXED_PRIORITY;
        else if (strcmp(argv[1], "adaptive") == 0)
            g_strategy = SCHED_ADAPTIVE;
    }

    const char* strategyName[] = {
        "Round-Robin",
        "Fixed Priority",
        "Adaptive"
    };
    printf("Scheduling strategy: %s\n", strategyName[g_strategy]);
    printf("(Use: %s [rr|fixed|adaptive])\n", argv[0]);
    printf("\n");

    // TODO: Initialize SRXL2 library
    // TODO: Initialize as bus master
    // TODO: Main loop calling srxlRun()

    return 0;
}
