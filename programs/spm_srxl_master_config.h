/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Custom configuration for srxl2_master_sim program
This file overrides the default hooks with extern declarations,
allowing the simulator to provide its own implementations.
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
 */
extern void srxlOnMasterRunStart(uint8_t busIndex);

/**
 * @brief Called at the very end of each srxlRunMaster() cycle
 */
extern void srxlOnMasterRunEnd(uint8_t busIndex);

///////////////////////////////////////////////////////////////////////////////
// HANDSHAKE HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called when master handshake sequence is starting
 */
extern void srxlOnMasterHandshakeStart(uint8_t busIndex);

/**
 * @brief Called when master handshake sequence is complete
 */
extern void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount);

///////////////////////////////////////////////////////////////////////////////
// FRAME TIMING HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called once per frame before channel data is sent
 *
 * @return true to skip normal channel data sending, false for normal operation
 */
extern bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount);

///////////////////////////////////////////////////////////////////////////////
// CHANNEL DATA HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called after channel data packet has been sent
 */
extern void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID);

///////////////////////////////////////////////////////////////////////////////
// TELEMETRY HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called to select which device should send telemetry this frame
 *
 * @return The device ID to request telemetry from
 */
extern uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID);

/**
 * @brief Called when telemetry TX is enabled/disabled
 */
extern void srxlOnMasterSetTelemTx(bool enabled);

/**
 * @brief Called when telemetry packet has been sent over RF
 */
extern void srxlOnMasterTelemSent(void);

/**
 * @brief Called when internal telemetry should be suppressed
 */
extern void srxlOnMasterSuppressTelem(void* pTelemetryData);

///////////////////////////////////////////////////////////////////////////////
// BIND HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called when bind is initiated
 *
 * @return true if bind was successfully initiated, false otherwise
 */
extern bool srxlOnMasterBind(void* pBindInfo);

///////////////////////////////////////////////////////////////////////////////
// INTERNAL/TEST HOOKS
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called to parse Spektrum internal test packets
 *
 * @return true if packet was handled, false otherwise
 */
extern bool srxlOnMasterParseInternal(void* pInternal);

/**
 * @brief Called to fill Spektrum internal test packets
 *
 * @return Length of filled packet data
 */
extern uint8_t srxlOnMasterFillInternal(void* pInternal);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __SPM_SRXL_MASTER_CONFIG_H__
