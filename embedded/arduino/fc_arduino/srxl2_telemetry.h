/*
 * SRXL2 Telemetry Payload Decoder
 *
 * Decodes the 16-byte X-Bus telemetry payloads carried inside SRXL2
 * telemetry packets (0x80). Handles big-endian byte swapping and
 * "no data" sentinel detection.
 *
 * Supported sensor types:
 *   0x20 - ESC (STRU_TELE_ESC)
 *   0x34 - Flight Pack Capacity (STRU_TELE_FP_MAH)
 *   0x42 - Smart Battery (reverse-engineered from MSRC)
 *   0x3A - LiPo Cell Monitor 6S (STRU_TELE_LIPOMON)
 *   0x7E - RPM/Volts/Temp (STRU_TELE_RPM)
 *
 * MIT License
 */

#ifndef SRXL2_TELEMETRY_H
#define SRXL2_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Sensor IDs (first byte of the 16-byte telemetry payload)
//=============================================================================

#define SRXL2_TELEM_SENSOR_ESC          0x20
#define SRXL2_TELEM_SENSOR_RX_MAH       0x18
#define SRXL2_TELEM_SENSOR_FP_MAH       0x34
#define SRXL2_TELEM_SENSOR_LIPOMON      0x3A
#define SRXL2_TELEM_SENSOR_SMART_BAT    0x42
#define SRXL2_TELEM_SENSOR_RPM          0x7E

//=============================================================================
// Smart Battery sub-types (byte 2 of the payload when sensor ID is 0x42)
//=============================================================================

#define SRXL2_SMART_BAT_REALTIME        0x00
#define SRXL2_SMART_BAT_CELLS_1         0x10
#define SRXL2_SMART_BAT_CELLS_2         0x20
#define SRXL2_SMART_BAT_CELLS_3         0x30
#define SRXL2_SMART_BAT_ID             0x80
#define SRXL2_SMART_BAT_LIMITS          0x90

//=============================================================================
// "No data" sentinel value
//=============================================================================

#define SRXL2_NO_DATA_16    0xFFFF
#define SRXL2_NO_DATA_8     0xFF

// NAN is used in decoded structs to indicate "no data"
#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif

//=============================================================================
// Wire-format structs (packed, big-endian 16-bit fields)
// These match the exact byte layout on the wire.
//=============================================================================

typedef struct {
    uint8_t  identifier;     // 0x20
    uint8_t  s_id;           // Secondary ID
    uint16_t rpm;            // 10 RPM/count, big-endian
    uint16_t volts_input;    // 0.01V/count
    uint16_t temp_fet;       // 0.1C/count
    uint16_t current_motor;  // 10mA/count
    uint16_t temp_bec;       // 0.1C/count
    uint8_t  current_bec;    // 100mA/count
    uint8_t  voltage_bec;    // 0.05V/count
    uint8_t  throttle;       // 0.5%/count
    uint8_t  power_out;      // 0.5%/count
} __attribute__((packed)) srxl2_wire_esc_t;

typedef struct {
    uint8_t  identifier;     // 0x34
    uint8_t  s_id;
    int16_t  current_a;      // 0.1A/count
    int16_t  charge_used_a;  // 1mAh/count
    uint16_t temp_a;         // 0.1C/count, 0x7FFF = not populated
    int16_t  current_b;
    int16_t  charge_used_b;
    uint16_t temp_b;
    uint16_t spare;
} __attribute__((packed)) srxl2_wire_fp_mah_t;

typedef struct {
    uint8_t  identifier;     // 0x3A
    uint8_t  s_id;
    uint16_t cell[6];        // 0.01V/count, 0x7FFF = not present
    uint16_t temp;           // 0.1C/count
} __attribute__((packed)) srxl2_wire_lipomon_t;

typedef struct {
    uint8_t  identifier;     // 0x7E
    uint8_t  s_id;
    uint16_t microseconds;   // Pulse period (RPM sensor)
    uint16_t volts;          // 0.01V/count
    int16_t  temperature;    // Degrees F, 0x7FFF = no data
    int8_t   dbm_a;          // RSSI antenna A
    int8_t   dbm_b;          // RSSI antenna B
    uint16_t spare[2];
    uint16_t uptime;         // bit 15 = fastboot, bits 0-14 = seconds
} __attribute__((packed)) srxl2_wire_rpm_t;

// Smart Battery wire structs (sensor ID 0x42)
typedef struct {
    uint8_t  identifier;     // 0x42
    uint8_t  s_id;
    uint8_t  type;           // 0x00
    int8_t   temp;           // Degrees C
    uint32_t current;        // mA
    uint16_t consumption;    // 0.1 mAh
    uint16_t min_cell;       // mV
    uint16_t max_cell;       // mV
} __attribute__((packed)) srxl2_wire_smart_bat_realtime_t;

typedef struct {
    uint8_t  identifier;     // 0x42
    uint8_t  s_id;
    uint8_t  type;           // 0x10, 0x20, or 0x30
    int8_t   temp;           // Degrees C
    uint16_t cell[6];        // mV
} __attribute__((packed)) srxl2_wire_smart_bat_cells_t;

typedef struct {
    uint8_t  identifier;     // 0x42
    uint8_t  s_id;
    uint8_t  type;           // 0x80
    uint8_t  chemistry;
    uint8_t  cells;          // Cell count
    uint8_t  mfg_code;
    uint16_t cycles;         // Charge cycles
    uint8_t  uid;
} __attribute__((packed)) srxl2_wire_smart_bat_id_t;

typedef struct {
    uint8_t  identifier;     // 0x42
    uint8_t  s_id;
    uint8_t  type;           // 0x90
    uint8_t  rfu;
    uint16_t capacity;       // mAh
    uint16_t discharge_rate;
    uint16_t overdischarge;
    uint16_t zero_capacity;
    uint16_t fully_charged;
    uint8_t  min_temp;       // Degrees C
    uint8_t  max_temp;       // Degrees C
} __attribute__((packed)) srxl2_wire_smart_bat_limits_t;

//=============================================================================
// Decoded structs (host byte order, human-readable units, NAN = no data)
//=============================================================================

typedef struct {
    float    rpm;            // RPM
    float    voltage;        // Volts
    float    current;        // Amps
    float    temp_fet;       // Degrees C
    float    temp_bec;       // Degrees C
    float    current_bec;    // Amps
    float    voltage_bec;    // Volts
    float    throttle;       // Percent (0-100)
    float    power_out;      // Percent (0-100)
    uint8_t  s_id;
} srxl2_telem_esc_t;

typedef struct {
    float    current_a;      // Amps
    float    charge_used_a;  // mAh
    float    temp_a;         // Degrees C (NAN if not populated)
    float    current_b;      // Amps
    float    charge_used_b;  // mAh
    float    temp_b;         // Degrees C (NAN if not populated)
    uint8_t  s_id;
} srxl2_telem_fp_mah_t;

typedef struct {
    float    cell[6];        // Volts per cell (NAN if not present)
    float    temp;           // Degrees C
    uint8_t  s_id;
} srxl2_telem_lipomon_t;

typedef struct {
    float    rpm;            // RPM (computed from microseconds)
    float    voltage;        // Volts
    float    temperature;    // Degrees C (NAN if no data)
    int8_t   rssi_a;         // dBm or % (0 = no data)
    int8_t   rssi_b;         // dBm or % (0 = no data)
    uint8_t  s_id;
} srxl2_telem_rpm_t;

typedef struct {
    float    temp;           // Degrees C
    float    current;        // Amps
    float    consumption;    // mAh
    float    min_cell;       // Volts
    float    max_cell;       // Volts
} srxl2_telem_smart_bat_realtime_t;

typedef struct {
    float    temp;           // Degrees C
    float    cell[6];        // Volts per cell
} srxl2_telem_smart_bat_cells_t;

typedef struct {
    uint8_t  chemistry;
    uint8_t  cells;          // Cell count
    uint8_t  mfg_code;
    uint16_t cycles;         // Charge cycles
    uint8_t  uid;
} srxl2_telem_smart_bat_id_t;

typedef struct {
    uint16_t capacity;       // mAh
    uint16_t discharge_rate;
    uint16_t overdischarge;
    uint16_t zero_capacity;
    uint16_t fully_charged;
    uint8_t  min_temp;       // Degrees C
    uint8_t  max_temp;       // Degrees C
} srxl2_telem_smart_bat_limits_t;

//=============================================================================
// Telemetry result (tagged union)
//=============================================================================

typedef enum {
    SRXL2_TELEM_TYPE_UNKNOWN = 0,
    SRXL2_TELEM_TYPE_ESC,
    SRXL2_TELEM_TYPE_FP_MAH,
    SRXL2_TELEM_TYPE_LIPOMON,
    SRXL2_TELEM_TYPE_RPM,
    SRXL2_TELEM_TYPE_SMART_BAT_REALTIME,
    SRXL2_TELEM_TYPE_SMART_BAT_CELLS,
    SRXL2_TELEM_TYPE_SMART_BAT_ID,
    SRXL2_TELEM_TYPE_SMART_BAT_LIMITS,
} srxl2_telem_type_t;

typedef struct {
    srxl2_telem_type_t type;
    uint8_t sensor_id;       // Raw sensor ID byte (0x20, 0x34, 0x42, etc.)
    uint8_t smart_bat_sub;   // Smart battery sub-type (only for 0x42)
    union {
        srxl2_telem_esc_t               esc;
        srxl2_telem_fp_mah_t            fp_mah;
        srxl2_telem_lipomon_t           lipomon;
        srxl2_telem_rpm_t               rpm;
        srxl2_telem_smart_bat_realtime_t smart_bat_realtime;
        srxl2_telem_smart_bat_cells_t    smart_bat_cells;
        srxl2_telem_smart_bat_id_t       smart_bat_id;
        srxl2_telem_smart_bat_limits_t   smart_bat_limits;
    };
} srxl2_telem_decoded_t;

//=============================================================================
// Public API
//=============================================================================

/**
 * @brief Decode a 16-byte telemetry payload into human-readable values
 *
 * @param raw       Pointer to the 16-byte X-Bus telemetry payload
 *                  (bytes 4-19 of the SRXL2 telemetry packet)
 * @param decoded   Output decoded telemetry structure
 * @return          true if the sensor type was recognized and decoded
 */
bool srxl2_decode_telemetry(const uint8_t raw[16], srxl2_telem_decoded_t *decoded);

/**
 * @brief Get human-readable name for a telemetry sensor type
 *
 * @param sensor_id Raw sensor ID byte
 * @return          String name of sensor type
 */
const char *srxl2_telem_sensor_name(uint8_t sensor_id);

/**
 * @brief Get human-readable name for a decoded telemetry type
 *
 * @param type Decoded telemetry type enum
 * @return     String name
 */
const char *srxl2_telem_type_name(srxl2_telem_type_t type);

#ifdef __cplusplus
}
#endif

#endif // SRXL2_TELEMETRY_H
