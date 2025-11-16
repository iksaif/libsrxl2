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

/**
 * @file spm_srxl_master_config.h
 * @brief Configuration and Hooks for SRXL2 Master
 *
 * This file defines hook functions that allow users to customize the behavior
 * of the SRXL2 bus master. Users should implement these functions in their
 * application to handle events and customize master behavior.
 *
 * All hooks are optional - default implementations are provided that do nothing.
 */

#ifndef __SPM_SRXL_MASTER_CONFIG_H__
#define __SPM_SRXL_MASTER_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// LIFECYCLE HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called at the very start of each srxlRunMaster() cycle
 *
 * This hook is called before any state machine logic runs. Use it for:
 * - Timing/profiling measurements
 * - Pre-processing before master logic
 * - Debugging/logging master state
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 */
static inline void srxlOnMasterRunStart(uint8_t busIndex)
{
    (void)busIndex;
    // Default: Do nothing
}

/**
 * @brief Called at the very end of each srxlRunMaster() cycle
 *
 * This hook is called after all state machine logic has run. Use it for:
 * - Timing/profiling measurements
 * - Post-processing after master logic
 * - Cleanup or state validation
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 */
static inline void srxlOnMasterRunEnd(uint8_t busIndex)
{
    (void)busIndex;
    // Default: Do nothing
}

///////////////////////////////////////////////////////////////////////////////
// HANDSHAKE HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called when master handshake sequence is starting
 *
 * The handshake sequence discovers all devices on the bus and negotiates
 * baud rate. Use this hook for:
 * - Logging/debugging
 * - Resetting device discovery state
 * - Notifying UI that discovery is in progress
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 */
static inline void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    (void)busIndex;
    // Default: Do nothing
}

/**
 * @brief Called when master handshake sequence is complete
 *
 * At this point all devices have been discovered and the master will
 * transition to normal operation. Use this hook for:
 * - Logging discovered devices
 * - Validating expected devices are present
 * - Notifying UI that discovery is complete
 * - Setting up device-specific configurations
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 * @param deviceCount Number of devices discovered on the bus
 */
static inline void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    (void)busIndex;
    (void)deviceCount;
    // Default: Do nothing
}

///////////////////////////////////////////////////////////////////////////////
// FRAME TIMING HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called once per frame before channel data is sent
 *
 * This is called every ~11ms frame and allows you to inject custom behavior.
 * Use this hook for:
 * - Generating periodic telemetry data
 * - Implementing custom timing/scheduling
 * - Monitoring frame timing
 * - Injecting custom packets between channel data frames
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 * @param frameCount Frame counter (increments each frame)
 * @return true to skip normal channel data sending (you handle this frame),
 *         false to let master send channel data normally
 */
static inline bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    (void)busIndex;
    (void)frameCount;
    // Default: Allow normal channel data sending
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// CHANNEL DATA HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called after channel data packet has been sent
 *
 * Use this hook for:
 * - Logging channel data transmission
 * - Tracking which devices are receiving telemetry requests
 * - Monitoring bus activity
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 * @param telemDeviceID The device ID that was requested to send telemetry (0 if none)
 */
static inline void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    (void)busIndex;
    (void)telemDeviceID;
    // Default: Do nothing
}

///////////////////////////////////////////////////////////////////////////////
// TELEMETRY HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called to select which device should send telemetry this frame
 *
 * The master uses a priority-based weighted round-robin algorithm by default.
 * This hook allows you to override the selection. Use this hook for:
 * - Implementing custom telemetry scheduling
 * - Prioritizing specific devices temporarily
 * - Debugging telemetry flow
 *
 * @param busIndex The bus index (0 to SRXL_NUM_OF_BUSES-1)
 * @param defaultDeviceID The device ID selected by default algorithm
 * @return The device ID to request telemetry from (return defaultDeviceID for no change)
 */
static inline uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    (void)busIndex;
    // Default: Use the default selection
    return defaultDeviceID;
}

/**
 * @brief Called when telemetry TX is enabled/disabled
 *
 * For master receivers that transmit telemetry over RF, this indicates
 * whether the receiver should be actively transmitting. Use this hook for:
 * - Controlling RF telemetry transmission
 * - Managing power consumption
 * - Coordinating with multiple receivers
 *
 * @param enabled true to enable telemetry TX, false to disable
 */
static inline void srxlOnMasterSetTelemTx(bool enabled)
{
    (void)enabled;
    // Default: Do nothing
}

/**
 * @brief Called when telemetry packet has been sent over RF
 *
 * This is called after a telemetry packet has been transmitted over the
 * RF link (for receivers). Use this hook for:
 * - Tracking telemetry transmission statistics
 * - Aging telemetry data
 * - Generating new telemetry
 *
 */
static inline void srxlOnMasterTelemSent(void)
{
    // Default: Do nothing
}

/**
 * @brief Called when internal telemetry should be suppressed
 *
 * When external telemetry is received, internal generation may need to be
 * suppressed to avoid duplicates. Use this hook for:
 * - Managing telemetry data sources
 * - Avoiding telemetry conflicts
 *
 * @param pTelemetryData Pointer to received telemetry data (SrxlTelemetryData*)
 */
static inline void srxlOnMasterSuppressTelem(void* pTelemetryData)
{
    (void)pTelemetryData;
    // Default: Do nothing
}

///////////////////////////////////////////////////////////////////////////////
// BIND HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called when bind is initiated
 *
 * Use this hook for:
 * - Implementing actual bind procedure for receiver hardware
 * - Storing bind information to non-volatile memory
 * - Triggering LED patterns or other user feedback
 *
 * @param pBindInfo Pointer to bind information structure (SrxlBindData*)
 * @return true if bind was successfully initiated, false otherwise
 */
static inline bool srxlOnMasterBind(void* pBindInfo)
{
    (void)pBindInfo;
    // Default: Bind not supported
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// INTERNAL/TEST HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called to parse Spektrum internal test packets
 *
 * These are used for internal Spektrum testing and diagnostics.
 * Most users can ignore this hook.
 *
 * @param pInternal Pointer to internal packet structure (SrxlInternalPacket*)
 * @return true if packet was handled, false otherwise
 */
static inline bool srxlOnMasterParseInternal(void* pInternal)
{
    (void)pInternal;
    // Default: Not handled
    return false;
}

/**
 * @brief Called to fill Spektrum internal test packets
 *
 * These are used for internal Spektrum testing and diagnostics.
 * Most users can ignore this hook.
 *
 * @param pInternal Pointer to internal packet structure (SrxlInternalPacket*)
 * @return Length of filled packet data
 */
static inline uint8_t srxlOnMasterFillInternal(void* pInternal)
{
    (void)pInternal;
    // Default: No data
    return 0;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __SPM_SRXL_MASTER_CONFIG_H__
