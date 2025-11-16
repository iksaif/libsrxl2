/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Example 4: Device Manager

This example demonstrates device discovery, management, and monitoring.
It tracks all devices on the bus, validates expected devices are present,
and provides a rich device information display.

Use case:
- System validation and diagnostics
- Device inventory management
- Health monitoring
- User interface for device status
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

///////////////////////////////////////////////////////////////////////////////
// DEVICE DATABASE
///////////////////////////////////////////////////////////////////////////////

#define MAX_DEVICES 16

typedef struct
{
    uint8_t  deviceID;
    uint8_t  devType;           // Upper nibble of device ID
    char     typeName[32];
    uint8_t  priority;
    uint8_t  info;              // Device info bits
    bool     present;           // Discovered during handshake
    uint32_t lastSeen;          // Frame number when last seen
    uint32_t telemCount;        // Telemetry packets received
    time_t   discoveredTime;    // When device was first discovered
} DeviceEntry;

typedef struct
{
    DeviceEntry devices[MAX_DEVICES];
    uint8_t deviceCount;
    bool handshakeComplete;
    uint32_t currentFrame;
} DeviceManager;

static DeviceManager g_devMgr[SRXL_NUM_OF_BUSES];

///////////////////////////////////////////////////////////////////////////////
// DEVICE TYPE NAMES
///////////////////////////////////////////////////////////////////////////////

static const char* getDeviceTypeName(uint8_t devType)
{
    switch (devType)
    {
        case 0x0: return "None";
        case 0x1: return "Remote Receiver";
        case 0x2: return "Receiver";
        case 0x3: return "Flight Controller";
        case 0x4: return "ESC";
        case 0x6: return "SRXL Servo (Type 1)";
        case 0x7: return "SRXL Servo (Type 2)";
        case 0x8: return "VTX";
        case 0x9: return "External RF";
        case 0xA: return "Remote ID";
        case 0xB: return "Sensor";
        case 0xF: return "Broadcast";
        default:  return "Unknown";
    }
}

///////////////////////////////////////////////////////////////////////////////
// DEVICE MANAGEMENT FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static DeviceEntry* findOrCreateDevice(uint8_t busIndex, uint8_t deviceID)
{
    DeviceManager* pMgr = &g_devMgr[busIndex];

    // Find existing
    for (uint8_t i = 0; i < pMgr->deviceCount; ++i)
    {
        if (pMgr->devices[i].deviceID == deviceID)
        {
            return &pMgr->devices[i];
        }
    }

    // Create new
    if (pMgr->deviceCount >= MAX_DEVICES)
        return NULL;

    DeviceEntry* pDev = &pMgr->devices[pMgr->deviceCount++];
    memset(pDev, 0, sizeof(DeviceEntry));
    pDev->deviceID = deviceID;
    pDev->devType = (deviceID >> 4) & 0x0F;
    strncpy(pDev->typeName, getDeviceTypeName(pDev->devType), sizeof(pDev->typeName) - 1);
    pDev->discoveredTime = time(NULL);
    pDev->present = false;

    return pDev;
}

static void printDeviceList(uint8_t busIndex)
{
    DeviceManager* pMgr = &g_devMgr[busIndex];

    printf("\n");
    printf("========================================================================\n");
    printf("Device List - Bus %d\n", busIndex);
    printf("========================================================================\n");
    printf("ID   | Type                  | Present | Priority | Telem | Last Seen\n");
    printf("-----|----------------------|---------|----------|-------|----------\n");

    for (uint8_t i = 0; i < pMgr->deviceCount; ++i)
    {
        DeviceEntry* pDev = &pMgr->devices[i];

        printf("0x%02X | %-20s | %-7s | %8d | %5u | Frame %u\n",
               pDev->deviceID,
               pDev->typeName,
               pDev->present ? "YES" : "NO",
               pDev->priority,
               pDev->telemCount,
               pDev->lastSeen);

        // Show device capabilities
        if (pDev->info != 0)
        {
            printf("     |   Capabilities: ");
            if (pDev->info & SRXL_DEVINFO_TELEM_TX_ENABLED)
                printf("TelemTX ");
            if (pDev->info & SRXL_DEVINFO_TELEM_FULL_RANGE)
                printf("FullRange ");
            if (pDev->info & SRXL_DEVINFO_FWD_PROG_SUPPORT)
                printf("FwdProg ");
            printf("\n");
        }
    }

    printf("-----|----------------------|---------|----------|-------|----------\n");
    printf("Total: %d device(s)\n", pMgr->deviceCount);
    printf("========================================================================\n");
    printf("\n");
}

static void validateExpectedDevices(uint8_t busIndex)
{
    // Example: Check for expected devices and warn if missing
    DeviceManager* pMgr = &g_devMgr[busIndex];

    printf("[Device Manager] Validating device configuration...\n");

    // Check for receiver (required)
    bool hasReceiver = false;
    for (uint8_t i = 0; i < pMgr->deviceCount; ++i)
    {
        uint8_t type = pMgr->devices[i].devType;
        if ((type >= 0x1 && type <= 0x2) && pMgr->devices[i].present)
        {
            hasReceiver = true;
            break;
        }
    }

    if (!hasReceiver)
    {
        printf("[WARNING] No receiver found on bus %d!\n", busIndex);
    }
    else
    {
        printf("[OK] Receiver present on bus %d\n", busIndex);
    }

    // Check for telemetry capable devices
    bool hasTelemDevice = false;
    for (uint8_t i = 0; i < pMgr->deviceCount; ++i)
    {
        if ((pMgr->devices[i].info & SRXL_DEVINFO_TELEM_TX_ENABLED) &&
            pMgr->devices[i].present)
        {
            hasTelemDevice = true;
            break;
        }
    }

    if (!hasTelemDevice)
    {
        printf("[WARNING] No telemetry-capable device found on bus %d\n", busIndex);
    }
    else
    {
        printf("[OK] Telemetry device present on bus %d\n", busIndex);
    }
}

///////////////////////////////////////////////////////////////////////////////
// HOOK IMPLEMENTATIONS
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║   SRXL2 Device Discovery Started (Bus %d)     ║\n", busIndex);
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    // Clear present flags
    DeviceManager* pMgr = &g_devMgr[busIndex];
    for (uint8_t i = 0; i < pMgr->deviceCount; ++i)
    {
        pMgr->devices[i].present = false;
    }
    pMgr->handshakeComplete = false;
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    DeviceManager* pMgr = &g_devMgr[busIndex];
    pMgr->handshakeComplete = true;

    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  SRXL2 Device Discovery Complete (Bus %d)    ║\n", busIndex);
    printf("╚═══════════════════════════════════════════════╝\n");

    // Print device list
    printDeviceList(busIndex);

    // Validate configuration
    validateExpectedDevices(busIndex);
}

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    g_devMgr[busIndex].currentFrame = frameCount;

    // Periodically print device status
    if (frameCount % 1000 == 0 && frameCount > 0)
    {
        printf("\n[Frame %u] Periodic device status:\n", frameCount);
        printDeviceList(busIndex);
    }

    return false;
}

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    if (telemDeviceID == 0 || telemDeviceID == 0xFF)
        return;

    DeviceManager* pMgr = &g_devMgr[busIndex];
    DeviceEntry* pDev = findOrCreateDevice(busIndex, telemDeviceID);

    if (pDev)
    {
        pDev->present = true;
        pDev->lastSeen = pMgr->currentFrame;

        // First time seeing this device during operation
        if (!pMgr->handshakeComplete)
        {
            printf("[Discovery] Found device 0x%02X (%s)\n",
                   telemDeviceID, pDev->typeName);
        }
    }
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    // Use default selection
    return defaultDeviceID;
}

void srxlOnMasterSuppressTelem(void* pTelemetryData)
{
    // When telemetry is received, increment counter
    // (In real implementation, would need to identify source device)
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

int main(void)
{
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║  SRXL2 Master Example 4: Device Manager      ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");
    printf("This example demonstrates comprehensive device\n");
    printf("discovery, management, and monitoring.\n");
    printf("\n");

    // TODO: Initialize SRXL2 library
    // TODO: Initialize as bus master with device ID 0x10 (remote receiver)
    // TODO: Main loop calling srxlRun()

    printf("\n");
    printf("Example complete.\n");

    return 0;
}
