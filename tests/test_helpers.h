/*
 * Shared Test Infrastructure for SRXL2 State Machine Tests
 *
 * MIT License
 */

#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdint.h>
#include <stdbool.h>

// Access SRXL2 internals (these may already be defined via -D flags from CMake)
#ifndef SRXL_INCLUDE_MASTER_CODE
#define SRXL_INCLUDE_MASTER_CODE
#endif
#ifndef SRXL_ENABLE_INTERNAL
#define SRXL_ENABLE_INTERNAL
#endif
#include "spm_srxl.h"

// Extern globals from spm_srxl.c that we need to reset/inspect
extern SrxlDevice srxlThisDev;
extern SrxlBus srxlBus[];
extern bool srxlChDataIsFailsafe;
extern bool srxlTelemetryPhase;
extern uint32_t srxlFailsafeChMask;
extern SrxlReceiverStats srxlRx;

// Hook call counters (test_helpers.c provides all 13 hooks)
extern int g_hook_run_start_count;
extern int g_hook_run_end_count;
extern int g_hook_handshake_start_count;
extern int g_hook_handshake_complete_count;
extern int g_hook_handshake_complete_dev_count;
extern int g_hook_frame_count;
extern int g_hook_channel_sent_count;
extern uint8_t g_hook_channel_sent_telem_id;
extern int g_hook_select_telem_count;
extern uint8_t g_hook_select_telem_default_id;
extern int g_hook_set_telem_tx_count;
extern bool g_hook_set_telem_tx_enabled;
extern int g_hook_telem_sent_count;
extern int g_hook_suppress_telem_count;
extern int g_hook_bind_count;
extern int g_hook_parse_internal_count;
extern int g_hook_fill_internal_count;

// Callback tracking for user callbacks
extern int g_cb_fill_telem_count;
extern int g_cb_recv_channel_count;
extern bool g_cb_recv_channel_is_failsafe;
extern int g_cb_vtx_count;

// Whether srxlOnMasterFrame should return true (skip)
extern bool g_hook_frame_skip;

// Whether srxlOnMasterSelectTelemDevice should pass through or override
extern bool g_hook_select_telem_passthrough;
extern uint8_t g_hook_select_telem_override;

// Initialize as master device (e.g., 0x10)
void test_init_master(uint8_t deviceID);

// Initialize as slave device (e.g., 0xB1)
void test_init_slave(uint8_t deviceID);

// Reset all globals for test isolation
void test_reset_all(void);

// Reset only capture buffer and hook counters
void test_reset_capture(void);

// Build a valid handshake packet with correct CRC
// Returns packet length
uint8_t test_build_handshake(uint8_t *buf, uint8_t src, uint8_t dest,
                              uint8_t priority, uint8_t baud,
                              uint8_t info, uint32_t uid);

// Build a valid 0xCD channel data packet with CRC
// Returns packet length
uint8_t test_build_channel_data(uint8_t *buf, uint8_t cmd, uint8_t reply_id,
                                 uint32_t mask, const uint16_t *values,
                                 int8_t rssi);

// Build a valid 0x80 telemetry packet with CRC
// Returns packet length
uint8_t test_build_telemetry(uint8_t *buf, uint8_t dest_id,
                              const uint8_t payload[16]);

// Wrapper for srxlParsePacket
bool test_inject_packet(uint8_t bus, uint8_t *packet, uint8_t len);

// Wrapper for srxlRun
void test_advance_ms(uint8_t bus, int16_t ms);

// Compute SRXL2 CRC-16 (XMODEM, poly 0x1021, seed 0)
uint16_t test_crc16(const uint8_t *data, size_t len);

#endif // TEST_HELPERS_H
