/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "../../SRXL2/Source/spm_srxl.h"
#include "spm_srxl_master.h"
#include "spm_srxl_master_config.h"
#include <string.h>
#include <stdio.h>

// Declare symbols that are not in spm_srxl.h but are defined in spm_srxl.c and that are needed by this file
extern void srxlSend(SrxlBus* pBus, SRXL_CMD srxlCmd, uint8_t replyID);
extern bool srxlChDataIsFailsafe;

/// LOCAL VARIABLES ///

// Master data for each bus
static SrxlMasterData srxlMasterData[SRXL_NUM_OF_BUSES];

// Telemetry age tracking (incremented when telemetry not sent for a device)
#define TELEM_AGE_MAX (65535)

// Telemetry age threshold before trying to refresh
// Assuming 11ms per frame, 100 frames is 1 second.
#define TELEM_AGE_REFRESH_THRESHOLD (100)

/// HELPER FUNCTIONS ///

/**
 * @brief Get master data for a bus
 */
static inline SrxlMasterData* getMasterData(SrxlBus* pBus)
{
    if (!pBus || pBus->fullID.busIndex >= SRXL_NUM_OF_BUSES)
        return NULL;
    return &srxlMasterData[pBus->fullID.busIndex];
}

/**
 * @brief Age telemetry counters for all devices
 */
static void ageTelemCounters(SrxlBus* pBus)
{
    for (uint8_t i = 0; i < pBus->rxDevCount; ++i)
    {
        if (pBus->rxDevAge[i] < TELEM_AGE_MAX)
            pBus->rxDevAge[i]++;
    }
}

/**
 * @brief Check if telemetry is too old
 */
static bool telemetryIsTooOld(SrxlBus* pBus)
{
    for (uint8_t i = 0; i < pBus->rxDevCount; ++i)
    {
        if (pBus->rxDevAge[i] >= TELEM_AGE_REFRESH_THRESHOLD)
            return true;
    }
    return false;
}

/**
 * @brief Select next device to request telemetry from based on priority
 *
 * This implements a weighted round-robin selection based on device priority.
 * Devices with higher priority will be polled more frequently.
 */
static uint8_t selectNextTelemDevice(SrxlBus* pBus, SrxlMasterData* pMaster)
{
    if (pBus->rxDevCount == 0 || pBus->rxDevPrioritySum == 0)
        return 0xFF;

    // Find device with highest priority/age ratio
    uint8_t bestDevice = 0xFF;
    uint32_t bestScore = 0;

    for (uint8_t i = 0; i < pBus->rxDevCount; ++i)
    {
        // Score = priority * age (devices that haven't been polled in a while get higher score)
        uint32_t score = (uint32_t)pBus->rxDev[i].priority * (pBus->rxDevAge[i] + 1);

        if (score > bestScore)
        {
            bestScore = score;
            bestDevice = i;
        }
    }

    return bestDevice;
}

/**
 * @brief Start handshake sequence
 */
static void startHandshake(SrxlBus* pBus, SrxlMasterData* pMaster)
{
    // Reset device count
    pBus->rxDevCount = 0;
    pBus->rxDevPrioritySum = 0;

    // Start by polling our own ID to establish master
    pBus->requestID = pBus->fullID.deviceID;

    // Hook: User notification that handshake is starting
    srxlOnMasterHandshakeStart(pBus->fullID.busIndex);
}

/**
 * @brief Continue handshake sequence to next device ID
 */
static void continueHandshake(SrxlBus* pBus)
{
    // Scan through all possible device IDs (0x10 to 0xFE)
    while (pBus->requestID < 0xFF)
    {
        // Skip IDs below 0x10 (invalid)
        if (pBus->requestID < 0x10)
        {
            pBus->requestID = 0x10;
            continue;
        }

        pBus->requestID++;

        // Don't poll our own device ID
        if (pBus->requestID == pBus->fullID.deviceID)
        {
            pBus->requestID++;
            continue;
        }
        return;
    }

    // Here we should send the broadcast handshake.
    // requestID should be 0xFF at this point.
    
    // Hook: User notification that handshake is complete
    srxlOnMasterHandshakeComplete(pBus->fullID.busIndex, pBus->rxDevCount);
}

/**
 * @brief Send channel data packet
 */
static void sendChannelData(SrxlBus* pBus, SrxlMasterData* pMaster)
{
    // Select device to request telemetry from
    uint8_t telemDevIndex = selectNextTelemDevice(pBus, pMaster);
    uint8_t telemDevID = 0;

    if (telemDevIndex != 0xFF && telemDevIndex < pBus->rxDevCount)
    {
        telemDevID = pBus->rxDev[telemDevIndex].deviceID;
        pMaster->telemDevIndex = telemDevIndex;
    }

    // Hook: Allow user to modify which device gets telemetry request
    telemDevID = srxlOnMasterSelectTelemDevice(pBus->fullID.busIndex, telemDevID);
   
    // Send failsafe data if in failsafe mode
    if (srxlChDataIsFailsafe)
    {
        pBus->state = SrxlState_Running;
        srxlSend(pBus, SRXL_CMD_CHANNEL_FS, 0);
    } else {
        pBus->state = SrxlState_Running;
        srxlSend(pBus, SRXL_CMD_CHANNEL, telemDevID);
    }

    // Age all telemetry counters
    ageTelemCounters(pBus);

    // Reset age for device we're requesting from
    if (telemDevIndex != 0xFF && telemDevIndex < pBus->rxDevCount)
    {
        pBus->rxDevAge[telemDevIndex] = 0;
    }

    // Hook: User notification that channel data was sent
    srxlOnMasterChannelDataSent(pBus->fullID.busIndex, telemDevID);
}

/// PUBLIC FUNCTIONS ///

/**
 * @brief Main master state machine
 */
void srxlRunMaster(SrxlBus* pBus)
{
    if (!pBus || !pBus->master || pBus->state == SrxlState_Disabled)
        return;

    // If we're polling once more, we should not send anything.
    if (pBus->pollOnceMore)
    {
        pBus->pollOnceMore = false;
        return;
    }

    SrxlMasterData* pMaster = getMasterData(pBus);
    if (!pMaster)
        return;


    // Hook: Called at the start of each master cycle
    srxlOnMasterRunStart(pBus->fullID.busIndex);


    // Handle special commands from bus state machine
    switch (pBus->state)
    {
    case SrxlState_ListenOnStartup:
    {
        // If we're waiting for the handshake to complete, we should not send anything.
        break;
    }

    case SrxlState_SendHandshake:
    {
        // Here we always restart from pBus->requestID as set by the state machine.
        if (pBus->requestID == 0 || pBus->requestID == pBus->fullID.deviceID) {
            // If we're starting a new series of handhsakes, we should reset the handshake state.
            startHandshake(pBus, pMaster);
        }
        
        srxlSend(pBus, SRXL_CMD_HANDSHAKE, pBus->requestID);

        if (pBus->requestID == 0xFF)
        {
            pBus->state = SrxlState_Running;
        }
        else
        {
            continueHandshake(pBus);
        }
        
        break;
    }

    case SrxlState_Running:
    {       
        if (srxlOnMasterFrame(pBus->fullID.busIndex, pMaster->frameCount))
        {
            break;
        }

        // If we're in a timeout, we should not send anything too early
        // and let some more time to get an answer from the device.
        if (pBus->timeoutCount_ms > 0) {
            int timeoutMs = pBus->baudRate == SRXL_BAUD_115200 ? 4 : 2;
            if (pBus->timeoutCount_ms < timeoutMs) {
                break;
            }
        }

        if (pBus->timeoutCount_ms >= 10 || telemetryIsTooOld(pBus) || pBus->channelOutMask || srxlChDataIsFailsafe)
        {
            sendChannelData(pBus, pMaster);
            pMaster->frameCount++;
            pBus->timeoutCount_ms = 0;
        }

        break;
    }

    case SrxlState_SendEnterBind:
    {
        srxlSend(pBus, SRXL_CMD_ENTER_BIND, pBus->requestID);
        pBus->state = SrxlState_Running;
        break;
    }

    case SrxlState_SendSetBindInfo:
    {
        srxlSend(pBus, SRXL_CMD_SET_BIND, pBus->requestID);
        pBus->state = SrxlState_Running;
        break;
    }

    case SrxlState_RequestBindInfo:
    {
        srxlSend(pBus, SRXL_CMD_REQ_BIND_INFO, pBus->requestID);
        pBus->state = SrxlState_Running;
        break;
    }

    case SrxlState_SendBoundDataReport:
    {
        srxlSend(pBus, SRXL_CMD_BIND_INFO, pBus->fullID.deviceID);
        pBus->state = SrxlState_Running;
        break;
    }

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
    case SrxlState_SendInternal:
    {
        srxlSend(pBus, SRXL_CMD_INTERNAL, pBus->requestID);
        pBus->state = SrxlState_Running;
        break;
    }
#endif

    default:
        break;
    }

    // Hook: Called at the end of each master cycle
    srxlOnMasterRunEnd(pBus->fullID.busIndex);
}

/**
 * @brief Initialize receiver entry for master
 */
void* srxlInitReceiver(uint8_t deviceID, uint8_t info)
{
    extern SrxlDevice srxlThisDev;
    extern SrxlReceiverStats srxlRx;

    if (srxlRx.rcvrCount >= SRXL_MAX_RCVRS)
        return NULL;

    // Add receiver entry
    uint8_t i = srxlRx.rcvrCount++;
    SrxlRcvrEntry* pRcvr = &srxlRx.rcvr[i];
    pRcvr->deviceID = deviceID;
    pRcvr->busBits = 0x01; // Start with bus 0
    pRcvr->info = info;
    pRcvr->rssiRcvd = 0;
    pRcvr->rssi_dBm = -1;
    pRcvr->rssi_Pct = 0;
    pRcvr->fades = 0;
    pRcvr->channelMask = 0;

    // Set as device receiver pointer
    srxlThisDev.pRcvr = pRcvr;

    // Add to sorted list
    srxlRx.rcvrSorted[i] = pRcvr;

    // If full-range telemetry, adjust insert point
    if (info & SRXL_DEVINFO_TELEM_FULL_RANGE)
    {
        srxlRx.rcvrSortInsert = 1;
    }

    return pRcvr;
}

/**
 * @brief Set outgoing channel mask
 */
void srxlSetOutgoingChannelMask(uint32_t channelMask)
{
    extern SrxlBus srxlBus[SRXL_NUM_OF_BUSES];

    for (uint8_t i = 0; i < SRXL_NUM_OF_BUSES; ++i)
    {
        if (srxlBus[i].master)
        {
            srxlBus[i].channelOutMask |= channelMask;
        }
    }
}

/**
 * @brief Enable/disable telemetry TX
 */
void srxlSetTelemetryTxEnable(bool enabled)
{
    // Hook for user implementation
    srxlOnMasterSetTelemTx(enabled);
}

/**
 * @brief Mark telemetry as sent
 */
void srxlTelemetrySent(void)
{
    // Hook for user implementation
    srxlOnMasterTelemSent();
}

/**
 * @brief Suppress internal telemetry
 */
void srxlSuppressInternalTelemetry(void* pTelemetryData)
{
    // Hook for user implementation
    srxlOnMasterSuppressTelem(pTelemetryData);
}

/**
 * @brief Try to bind receiver
 */
bool srxlTryToBind(SrxlBindData bindInfo)
{
    // Hook for user implementation
    return srxlOnMasterBind(&bindInfo);
}

/**
 * @brief Parse internal packet
 */
bool srxlParseInternal(void* pInternal)
{
    // Hook for user implementation
    return srxlOnMasterParseInternal(pInternal);
}

/**
 * @brief Fill internal packet
 */
uint8_t srxlFillInternal(void* pInternal)
{
    // Hook for user implementation
    return srxlOnMasterFillInternal(pInternal);
}
