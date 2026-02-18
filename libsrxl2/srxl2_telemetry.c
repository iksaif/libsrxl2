/*
 * SRXL2 Telemetry Payload Decoder
 *
 * MIT License
 */

#include "srxl2_telemetry.h"
#include <string.h>

//=============================================================================
// Byte-swap helpers
//=============================================================================

static inline uint16_t swap16(uint16_t v)
{
    return (v >> 8) | (v << 8);
}

static inline uint32_t swap32(uint32_t v)
{
    return ((v >> 24) & 0x000000FF) |
           ((v >>  8) & 0x0000FF00) |
           ((v <<  8) & 0x00FF0000) |
           ((v << 24) & 0xFF000000);
}

//=============================================================================
// ESC telemetry (0x20) — big-endian wire format
//=============================================================================

static void decode_esc(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_esc_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_esc_t *esc = &out->esc;
    out->type = SRXL2_TELEM_TYPE_ESC;
    esc->s_id = wire.s_id;

    esc->rpm         = (wire.rpm == SRXL2_NO_DATA_16)
                       ? NAN : (float)swap16(wire.rpm) * 10.0f;
    esc->voltage     = (wire.volts_input == SRXL2_NO_DATA_16)
                       ? NAN : (float)swap16(wire.volts_input) / 100.0f;
    esc->temp_fet    = (wire.temp_fet == SRXL2_NO_DATA_16)
                       ? NAN : (float)swap16(wire.temp_fet) / 10.0f;
    esc->current     = (wire.current_motor == SRXL2_NO_DATA_16)
                       ? NAN : (float)swap16(wire.current_motor) / 100.0f;
    esc->temp_bec    = (wire.temp_bec == SRXL2_NO_DATA_16)
                       ? NAN : (float)swap16(wire.temp_bec) / 10.0f;
    esc->current_bec = (wire.current_bec == SRXL2_NO_DATA_8)
                       ? NAN : (float)wire.current_bec / 10.0f;
    esc->voltage_bec = (wire.voltage_bec == SRXL2_NO_DATA_8)
                       ? NAN : (float)wire.voltage_bec / 20.0f;
    esc->throttle    = (wire.throttle == SRXL2_NO_DATA_8)
                       ? NAN : (float)wire.throttle * 0.5f;
    esc->power_out   = (wire.power_out == SRXL2_NO_DATA_8)
                       ? NAN : (float)wire.power_out * 0.5f;
}

//=============================================================================
// Flight Pack Capacity (0x34)
//=============================================================================

static void decode_fp_mah(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_fp_mah_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_fp_mah_t *fp = &out->fp_mah;
    out->type = SRXL2_TELEM_TYPE_FP_MAH;
    fp->s_id = wire.s_id;

    fp->current_a    = (float)(int16_t)swap16((uint16_t)wire.current_a) / 10.0f;
    fp->charge_used_a = (float)(int16_t)swap16((uint16_t)wire.charge_used_a);
    fp->temp_a       = (swap16(wire.temp_a) == 0x7FFF)
                       ? NAN : (float)swap16(wire.temp_a) / 10.0f;

    fp->current_b    = (float)(int16_t)swap16((uint16_t)wire.current_b) / 10.0f;
    fp->charge_used_b = (float)(int16_t)swap16((uint16_t)wire.charge_used_b);
    fp->temp_b       = (swap16(wire.temp_b) == 0x7FFF)
                       ? NAN : (float)swap16(wire.temp_b) / 10.0f;
}

//=============================================================================
// LiPo Cell Monitor 6S (0x3A)
//=============================================================================

static void decode_lipomon(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_lipomon_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_lipomon_t *lipo = &out->lipomon;
    out->type = SRXL2_TELEM_TYPE_LIPOMON;
    lipo->s_id = wire.s_id;

    for (int i = 0; i < 6; i++)
    {
        uint16_t v = swap16(wire.cell[i]);
        lipo->cell[i] = (v == 0x7FFF) ? NAN : (float)v / 100.0f;
    }

    lipo->temp = (float)swap16(wire.temp) / 10.0f;
}

//=============================================================================
// RPM/Volts/Temp (0x7E) — big-endian wire format
//=============================================================================

static void decode_rpm(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_rpm_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_rpm_t *rpm = &out->rpm;
    out->type = SRXL2_TELEM_TYPE_RPM;
    rpm->s_id = wire.s_id;

    uint16_t us = swap16(wire.microseconds);
    rpm->rpm = (us == 0 || us == SRXL2_NO_DATA_16)
               ? NAN : 60000000.0f / (float)us;

    rpm->voltage = (float)swap16(wire.volts) / 100.0f;

    int16_t temp_f = (int16_t)swap16((uint16_t)wire.temperature);
    rpm->temperature = (temp_f == 0x7FFF)
                       ? NAN : ((float)temp_f - 32.0f) * 5.0f / 9.0f;

    rpm->rssi_a = wire.dbm_a;
    rpm->rssi_b = wire.dbm_b;
}

//=============================================================================
// Smart Battery (0x42) — sub-type dispatch
//=============================================================================

static void decode_smart_bat_realtime(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_smart_bat_realtime_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_smart_bat_realtime_t *bat = &out->smart_bat_realtime;
    out->type = SRXL2_TELEM_TYPE_SMART_BAT_REALTIME;

    bat->temp        = (float)wire.temp;
    bat->current     = (float)swap32(wire.current) / 1000.0f;
    bat->consumption = (float)swap16(wire.consumption) / 10.0f;
    bat->min_cell    = (float)swap16(wire.min_cell) / 1000.0f;
    bat->max_cell    = (float)swap16(wire.max_cell) / 1000.0f;
}

static void decode_smart_bat_cells(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_smart_bat_cells_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_smart_bat_cells_t *cells = &out->smart_bat_cells;
    out->type = SRXL2_TELEM_TYPE_SMART_BAT_CELLS;

    cells->temp = (float)wire.temp;
    for (int i = 0; i < 6; i++)
        cells->cell[i] = (float)swap16(wire.cell[i]) / 1000.0f;
}

static void decode_smart_bat_id(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_smart_bat_id_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_smart_bat_id_t *id = &out->smart_bat_id;
    out->type = SRXL2_TELEM_TYPE_SMART_BAT_ID;

    id->chemistry = wire.chemistry;
    id->cells     = wire.cells;
    id->mfg_code  = wire.mfg_code;
    id->cycles    = swap16(wire.cycles);
    id->uid       = wire.uid;
}

static void decode_smart_bat_limits(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    srxl2_wire_smart_bat_limits_t wire;
    memcpy(&wire, raw, sizeof(wire));

    srxl2_telem_smart_bat_limits_t *lim = &out->smart_bat_limits;
    out->type = SRXL2_TELEM_TYPE_SMART_BAT_LIMITS;

    lim->capacity       = swap16(wire.capacity);
    lim->discharge_rate = swap16(wire.discharge_rate);
    lim->overdischarge  = swap16(wire.overdischarge);
    lim->zero_capacity  = swap16(wire.zero_capacity);
    lim->fully_charged  = swap16(wire.fully_charged);
    lim->min_temp       = wire.min_temp;
    lim->max_temp       = wire.max_temp;
}

static bool decode_smart_bat(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    uint8_t sub_type = raw[2];
    out->smart_bat_sub = sub_type;

    switch (sub_type)
    {
    case SRXL2_SMART_BAT_REALTIME:
        decode_smart_bat_realtime(raw, out);
        return true;
    case SRXL2_SMART_BAT_CELLS_1:
    case SRXL2_SMART_BAT_CELLS_2:
    case SRXL2_SMART_BAT_CELLS_3:
        decode_smart_bat_cells(raw, out);
        return true;
    case SRXL2_SMART_BAT_ID:
        decode_smart_bat_id(raw, out);
        return true;
    case SRXL2_SMART_BAT_LIMITS:
        decode_smart_bat_limits(raw, out);
        return true;
    default:
        out->type = SRXL2_TELEM_TYPE_UNKNOWN;
        return false;
    }
}

//=============================================================================
// Main decode entry point
//=============================================================================

bool srxl2_decode_telemetry(const uint8_t raw[16], srxl2_telem_decoded_t *decoded)
{
    if (!raw || !decoded)
        return false;

    memset(decoded, 0, sizeof(*decoded));
    decoded->sensor_id = raw[0];
    decoded->smart_bat_sub = 0;
    decoded->type = SRXL2_TELEM_TYPE_UNKNOWN;

    switch (raw[0])
    {
    case SRXL2_TELEM_SENSOR_ESC:
        decode_esc(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_FP_MAH:
        decode_fp_mah(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_LIPOMON:
        decode_lipomon(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_RPM:
        decode_rpm(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_SMART_BAT:
        return decode_smart_bat(raw, decoded);

    default:
        return false;
    }
}

//=============================================================================
// String helpers
//=============================================================================

const char *srxl2_telem_sensor_name(uint8_t sensor_id)
{
    switch (sensor_id)
    {
    case 0x00: return "No Data";
    case 0x01: return "Voltage";
    case 0x02: return "Temperature";
    case 0x03: return "High Current";
    case 0x0A: return "PowerBox";
    case 0x11: return "Airspeed";
    case 0x12: return "Altitude";
    case 0x14: return "G-Meter";
    case 0x15: return "JetCat";
    case 0x16: return "GPS Location";
    case 0x17: return "GPS Status";
    case 0x18: return "Rx Pack Capacity";
    case 0x1A: return "Gyro";
    case 0x1B: return "Attitude/Compass";
    case 0x1E: return "AS6X Gain";
    case 0x20: return "ESC";
    case 0x22: return "Fuel Flow";
    case 0x26: return "GPS Binary";
    case 0x34: return "Flight Pack Capacity";
    case 0x36: return "Digital Air";
    case 0x38: return "Strain Gauge";
    case 0x3A: return "LiPo Monitor 6S";
    case 0x3F: return "LiPo Monitor 14S";
    case 0x40: return "Vario";
    case 0x42: return "Smart Battery";
    case 0x7E: return "RPM/Volts/Temp";
    case 0x7F: return "QoS";
    default:   return "Unknown";
    }
}

const char *srxl2_telem_type_name(srxl2_telem_type_t type)
{
    switch (type)
    {
    case SRXL2_TELEM_TYPE_ESC:                  return "ESC";
    case SRXL2_TELEM_TYPE_FP_MAH:               return "Flight Pack Capacity";
    case SRXL2_TELEM_TYPE_LIPOMON:               return "LiPo Monitor";
    case SRXL2_TELEM_TYPE_RPM:                   return "RPM/Volts/Temp";
    case SRXL2_TELEM_TYPE_SMART_BAT_REALTIME:    return "Smart Battery Realtime";
    case SRXL2_TELEM_TYPE_SMART_BAT_CELLS:       return "Smart Battery Cells";
    case SRXL2_TELEM_TYPE_SMART_BAT_ID:          return "Smart Battery ID";
    case SRXL2_TELEM_TYPE_SMART_BAT_LIMITS:      return "Smart Battery Limits";
    case SRXL2_TELEM_TYPE_UNKNOWN:
    default:                                     return "Unknown";
    }
}
