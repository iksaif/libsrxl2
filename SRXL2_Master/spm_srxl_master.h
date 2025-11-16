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

#ifndef __SPM_SRXL_MASTER_H__
#define __SPM_SRXL_MASTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

// Forward declare SrxlBus if not already included
#ifndef __SRXL_H__
typedef struct SrxlBus SrxlBus;
#endif

/**
 * @file spm_srxl_master.h
 * @brief SRXL2 Bus Master Implementation
 *
 * This library provides an open-source implementation of SRXL2 bus master functionality,
 * intended for use with the official Spektrum SRXL2 library. It implements the srxlRunMaster()
 * function and provides hooks for user customization.
 *
 * The bus master is responsible for:
 * - Performing device discovery via handshake
 * - Broadcasting channel data to slave devices
 * - Polling devices for telemetry based on priority
 * - Managing baud rate negotiation
 */


// Master Runtime Data (per bus)
typedef struct SrxlMasterData
{
    uint8_t         telemDevIndex;      // Current device being polled for telemetry
    uint16_t        telemPriorityCount; // Running priority counter for telemetry scheduling
    uint16_t        frameCount;         // Frame counter for timing
} SrxlMasterData;

/**
 * @brief Main master state machine, called from srxlRun() when device is bus master
 *
 * This function implements the SRXL2 bus master protocol:
 * 1. Initial handshake with all devices on the bus
 * 2. Regular channel data transmission
 * 3. Priority-based telemetry polling
 *
 * @param pBus Pointer to the SRXL bus structure
 */
void srxlRunMaster(SrxlBus* pBus);

/**
 * @brief Initialize master-specific data for a bus
 *
 * @param busIndex Index of the bus to initialize
 * @param deviceID The device ID for the receiver (typically 0x10 for remote receiver)
 * @param info Device info bits (should include SRXL_DEVINFO_TELEM_TX_ENABLED)
 * @return Pointer to receiver entry if successful, NULL otherwise
 */
void* srxlInitReceiver(uint8_t deviceID, uint8_t info);

/**
 * @brief Set the outgoing channel mask for the master
 *
 * Called when channel data is received to mark which channels should be sent
 *
 * @param channelMask Bitmask of channels to send
 */
void srxlSetOutgoingChannelMask(uint32_t channelMask);

/**
 * @brief Enable or disable telemetry transmission over RF
 *
 * @param enabled True to enable telemetry TX, false to disable
 */
void srxlSetTelemetryTxEnable(bool enabled);

/**
 * @brief Callback when telemetry has been sent over RF
 *
 * Used to age telemetry data and manage telemetry flow
 */
void srxlTelemetrySent(void);

/**
 * @brief Suppress internal telemetry generation
 *
 * Called when external telemetry is received to prevent duplicate data
 *
 * @param pTelemetryData Pointer to received telemetry data
 */
void srxlSuppressInternalTelemetry(void* pTelemetryData);

/**
 * @brief Attempt to bind the receiver
 *
 * @param bindInfo Bind information including type, GUID, options
 * @return True if bind was initiated, false otherwise
 */
bool srxlTryToBind(SrxlBindData bindInfo);

/**
 * @brief Parse internal Spektrum test packet
 *
 * @param pInternal Pointer to internal packet structure
 * @return True if packet was valid and handled, false otherwise
 */
bool srxlParseInternal(void* pInternal);

/**
 * @brief Fill internal Spektrum test packet
 *
 * @param pInternal Pointer to internal packet structure to fill
 * @return Length of the filled packet
 */
uint8_t srxlFillInternal(void* pInternal);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __SPM_SRXL_MASTER_H__
