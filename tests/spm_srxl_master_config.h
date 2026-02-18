/*
 * SRXL2 Master Hooks Configuration for Tests
 *
 * Same as programs/spm_srxl_master_config.h — all 13 hooks as extern.
 * Test files provide implementations that track calls.
 *
 * MIT License
 */

#ifndef __SPM_SRXL_MASTER_CONFIG_H__
#define __SPM_SRXL_MASTER_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

// Lifecycle hooks
extern void srxlOnMasterRunStart(uint8_t busIndex);
extern void srxlOnMasterRunEnd(uint8_t busIndex);

// Handshake hooks
extern void srxlOnMasterHandshakeStart(uint8_t busIndex);
extern void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount);

// Frame timing hooks
extern bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount);

// Channel data hooks
extern void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID);

// Telemetry hooks
extern uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID);
extern void srxlOnMasterSetTelemTx(bool enabled);
extern void srxlOnMasterTelemSent(void);
extern void srxlOnMasterSuppressTelem(void *pTelemetryData);

// Bind hooks
extern bool srxlOnMasterBind(void *pBindInfo);

// Internal/test hooks
extern bool srxlOnMasterParseInternal(void *pInternal);
extern uint8_t srxlOnMasterFillInternal(void *pInternal);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __SPM_SRXL_MASTER_CONFIG_H__
