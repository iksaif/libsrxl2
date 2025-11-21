/*
 * SRXL2 Parser Library - Modern C Implementation
 *
 * A standalone, dependency-free SRXL2 packet parser library.
 * This library provides read-only parsing of SRXL2 protocol packets
 * with no side effects or external dependencies.
 *
 * Features:
 * - Zero dependencies (only standard C library)
 * - Pure parsing functions with no state modification
 * - Modern C99/C11 coding style
 * - CRC validation
 * - Comprehensive packet type support
 *
 * MIT License
 */

#ifndef SRXL2_PARSER_H
#define SRXL2_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// SRXL2 Protocol Constants
#define SRXL2_MAGIC_ID              0xA6
#define SRXL2_MAX_PACKET_SIZE       80
#define SRXL2_MIN_PACKET_SIZE       5
#define SRXL2_HEADER_SIZE           3
#define SRXL2_CRC_SIZE              2

// Packet Type IDs
typedef enum {
    SRXL2_PKT_HANDSHAKE         = 0x21,
    SRXL2_PKT_BIND              = 0x41,
    SRXL2_PKT_PARAMETER         = 0x50,
    SRXL2_PKT_RSSI              = 0x55,
    SRXL2_PKT_TELEMETRY         = 0x80,
    SRXL2_PKT_SPM_INTERNAL      = 0x99,
    SRXL2_PKT_CONTROL           = 0xCD,
} srxl2_packet_type_t;

// Control Data Subcommands
typedef enum {
    SRXL2_CTRL_CHANNEL          = 0x00,
    SRXL2_CTRL_CHANNEL_FS       = 0x01,
    SRXL2_CTRL_VTX              = 0x02,
    SRXL2_CTRL_FWD_PGM          = 0x03,
} srxl2_control_cmd_t;

// Bind Request Types
typedef enum {
    SRXL2_BIND_ENTER            = 0xEB,
    SRXL2_BIND_REQ_STATUS       = 0xB5,
    SRXL2_BIND_BOUND_DATA       = 0xDB,
    SRXL2_BIND_SET_INFO         = 0x5B,
} srxl2_bind_request_t;

// Device Type (upper nibble of device ID)
typedef enum {
    SRXL2_DEV_NONE              = 0x0,
    SRXL2_DEV_REMOTE_RECEIVER   = 0x1,
    SRXL2_DEV_RECEIVER          = 0x2,
    SRXL2_DEV_FLIGHT_CONTROLLER = 0x3,
    SRXL2_DEV_ESC               = 0x4,
    SRXL2_DEV_SERVO_1           = 0x6,
    SRXL2_DEV_SERVO_2           = 0x7,
    SRXL2_DEV_VTX               = 0x8,
    SRXL2_DEV_EXTERNAL_RF       = 0x9,
    SRXL2_DEV_REMOTE_ID         = 0xA,
    SRXL2_DEV_SENSOR            = 0xB,
    SRXL2_DEV_BROADCAST         = 0xF,
} srxl2_device_type_t;

// Baud Rate Support
typedef enum {
    SRXL2_BAUD_115200           = 0x00,
    SRXL2_BAUD_400000           = 0x01,
} srxl2_baud_t;

// Device Info Bits
typedef enum {
    SRXL2_INFO_TELEM_TX         = 0x01,
    SRXL2_INFO_FULL_RANGE       = 0x02,
    SRXL2_INFO_FWD_PROG         = 0x04,
} srxl2_device_info_t;

// Parse Result
typedef enum {
    SRXL2_PARSE_OK              = 0,
    SRXL2_PARSE_ERR_NULL_PTR    = -1,
    SRXL2_PARSE_ERR_TOO_SHORT   = -2,
    SRXL2_PARSE_ERR_TOO_LONG    = -3,
    SRXL2_PARSE_ERR_BAD_MAGIC   = -4,
    SRXL2_PARSE_ERR_BAD_LENGTH  = -5,
    SRXL2_PARSE_ERR_BAD_CRC     = -6,
    SRXL2_PARSE_ERR_UNKNOWN_TYPE = -7,
} srxl2_parse_result_t;

// Common Packet Header (present in all packets)
typedef struct {
    uint8_t magic_id;           // Always 0xA6
    uint8_t packet_type;        // See srxl2_packet_type_t
    uint8_t length;             // Total packet length including header and CRC
} srxl2_header_t;

// Handshake Packet Payload
typedef struct {
    uint8_t src_device_id;      // Source device ID
    uint8_t dest_device_id;     // Destination device ID (0=discovery, 0xFF=broadcast)
    uint8_t priority;           // Telemetry priority (1-100)
    uint8_t baud_supported;     // See srxl2_baud_t
    uint8_t info;               // Device info bits (see srxl2_device_info_t)
    uint32_t uid;               // Unique device identifier
} srxl2_handshake_t;

// Channel Data Payload
typedef struct {
    int8_t rssi;                // Signal quality (negative = dBm, positive = %)
    uint16_t frame_losses;      // Total frame loss count
    uint32_t channel_mask;      // Bitmask of active channels
    uint16_t values[32];        // Channel values (0-65532, center=32768)
    uint8_t num_channels;       // Number of channels in values array
} srxl2_channel_data_t;

// Control Packet Payload
typedef struct {
    uint8_t cmd;                // Control command (see srxl2_control_cmd_t)
    uint8_t reply_id;           // Device ID to reply with telemetry
    srxl2_channel_data_t channel_data;
} srxl2_control_t;

// Telemetry Packet Payload
typedef struct {
    uint8_t dest_device_id;     // Destination device for RF transmission
    uint8_t raw[16];            // Raw 16-byte telemetry data
} srxl2_telemetry_t;

// VTX Data
typedef struct {
    uint8_t band;               // VTX band
    uint8_t channel;            // VTX channel (0-7)
    uint8_t pit_mode;           // 0=Race, 1=Pit
    uint8_t power;              // Power level
    uint16_t power_mw;          // Power in milliwatts
    uint8_t region;             // 0=USA, 1=EU
} srxl2_vtx_data_t;

// Bind Packet Payload
typedef struct {
    uint8_t request;            // Bind request type (see srxl2_bind_request_t)
    uint8_t device_id;          // Target device ID
    uint8_t bind_type;          // Bind type
    uint8_t options;            // Bind options
    uint64_t guid;              // Transmitter GUID
    uint32_t uid;               // Device UID
} srxl2_bind_t;

// Internal Packet Payload (Spektrum internal use)
typedef struct {
    uint8_t src_device_id;      // Source device ID
    uint8_t dest_device_id;     // Destination device ID
    uint8_t test;               // Test/command byte
    uint32_t key;               // Key/data field
} srxl2_internal_t;

// Parsed Packet (union of all possible payloads)
typedef struct {
    srxl2_header_t header;
    union {
        srxl2_handshake_t handshake;
        srxl2_control_t control;
        srxl2_telemetry_t telemetry;
        srxl2_bind_t bind;
        srxl2_internal_t internal;
        uint8_t raw[SRXL2_MAX_PACKET_SIZE - SRXL2_HEADER_SIZE - SRXL2_CRC_SIZE];
    } payload;
} srxl2_packet_t;

//=============================================================================
// Public API Functions
//=============================================================================

/**
 * @brief Parse an SRXL2 packet from raw bytes
 *
 * This function validates and parses a complete SRXL2 packet, checking:
 * - Magic ID
 * - Length field
 * - CRC checksum
 * - Packet type
 *
 * @param data      Pointer to raw packet data
 * @param length    Length of data buffer
 * @param packet    Output parsed packet structure
 * @return          SRXL2_PARSE_OK on success, error code otherwise
 */
srxl2_parse_result_t srxl2_parse_packet(
    const uint8_t *data,
    size_t length,
    srxl2_packet_t *packet
);

/**
 * @brief Compute SRXL2 CRC-16 checksum
 *
 * Uses XMODEM/ZMODEM CRC with polynomial 0x1021
 *
 * @param data      Data to compute CRC over
 * @param length    Length of data (excluding CRC bytes)
 * @return          16-bit CRC value
 */
uint16_t srxl2_compute_crc(const uint8_t *data, size_t length);

/**
 * @brief Validate packet CRC
 *
 * @param data      Complete packet data including CRC
 * @param length    Total packet length
 * @return          true if CRC is valid, false otherwise
 */
bool srxl2_validate_crc(const uint8_t *data, size_t length);

/**
 * @brief Get device type from device ID
 *
 * @param device_id Device ID byte
 * @return          Device type (upper nibble)
 */
static inline srxl2_device_type_t srxl2_get_device_type(uint8_t device_id) {
    return (srxl2_device_type_t)((device_id >> 4) & 0x0F);
}

/**
 * @brief Get unit ID from device ID
 *
 * @param device_id Device ID byte
 * @return          Unit ID (lower nibble, 0=master)
 */
static inline uint8_t srxl2_get_unit_id(uint8_t device_id) {
    return device_id & 0x0F;
}

/**
 * @brief Check if device is a bus master
 *
 * @param device_id Device ID byte
 * @return          true if unit ID is 0 (master)
 */
static inline bool srxl2_is_master(uint8_t device_id) {
    return (device_id & 0x0F) == 0;
}

/**
 * @brief Get human-readable device type name
 *
 * @param device_type Device type enum
 * @return            String name of device type
 */
const char* srxl2_device_type_name(srxl2_device_type_t device_type);

/**
 * @brief Get human-readable packet type name
 *
 * @param packet_type Packet type enum
 * @return            String name of packet type
 */
const char* srxl2_packet_type_name(srxl2_packet_type_t packet_type);

/**
 * @brief Get human-readable parse result message
 *
 * @param result Parse result code
 * @return       String description of result
 */
const char* srxl2_parse_result_str(srxl2_parse_result_t result);

#ifdef __cplusplus
}
#endif

#endif // SRXL2_PARSER_H
