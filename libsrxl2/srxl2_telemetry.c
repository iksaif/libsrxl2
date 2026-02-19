/*
 * SRXL2 Telemetry Payload Decoder
 *
 * MIT License
 */

#include "srxl2_telemetry.h"
#include "srxl2_packet.h"
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
// Flight Controller Status (0x05)
//=============================================================================

static void decode_flitectrl(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_FLITECTRL;
    out->flitectrl.s_id = raw[1];
    out->flitectrl.flight_mode = raw[2] & 0x0F;
}

//=============================================================================
// Airspeed (0x11)
//=============================================================================

static void decode_airspeed(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_AIRSPEED;
    out->airspeed.s_id = raw[1];
    out->airspeed.speed     = (float)srxl2_rd_be16(&raw[2]);
    out->airspeed.max_speed = (float)srxl2_rd_be16(&raw[4]);
}

//=============================================================================
// G-Meter (0x14)
//=============================================================================

static void decode_gmeter(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_GMETER;
    out->gmeter.s_id = raw[1];
    out->gmeter.x     = (float)(int16_t)srxl2_rd_be16(&raw[2])  / 100.0f;
    out->gmeter.y     = (float)(int16_t)srxl2_rd_be16(&raw[4])  / 100.0f;
    out->gmeter.z     = (float)(int16_t)srxl2_rd_be16(&raw[6])  / 100.0f;
    out->gmeter.max_x = (float)(int16_t)srxl2_rd_be16(&raw[8])  / 100.0f;
    out->gmeter.max_y = (float)(int16_t)srxl2_rd_be16(&raw[10]) / 100.0f;
    out->gmeter.max_z = (float)(int16_t)srxl2_rd_be16(&raw[12]) / 100.0f;
    out->gmeter.min_z = (float)(int16_t)srxl2_rd_be16(&raw[14]) / 100.0f;
}

//=============================================================================
// Gyro (0x1A)
//=============================================================================

static void decode_gyro(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_GYRO;
    out->gyro.s_id = raw[1];
    out->gyro.x     = (float)(int16_t)srxl2_rd_be16(&raw[2])  / 10.0f;
    out->gyro.y     = (float)(int16_t)srxl2_rd_be16(&raw[4])  / 10.0f;
    out->gyro.z     = (float)(int16_t)srxl2_rd_be16(&raw[6])  / 10.0f;
    out->gyro.max_x = (float)(int16_t)srxl2_rd_be16(&raw[8])  / 10.0f;
    out->gyro.max_y = (float)(int16_t)srxl2_rd_be16(&raw[10]) / 10.0f;
    out->gyro.max_z = (float)(int16_t)srxl2_rd_be16(&raw[12]) / 10.0f;
}

//=============================================================================
// Attitude & Magnetic Compass (0x1B)
//=============================================================================

static void decode_attmag(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_ATTMAG;
    out->attmag.s_id = raw[1];
    out->attmag.roll    = (float)(int16_t)srxl2_rd_be16(&raw[2])  / 10.0f;
    out->attmag.pitch   = (float)(int16_t)srxl2_rd_be16(&raw[4])  / 10.0f;
    out->attmag.yaw     = (float)(int16_t)srxl2_rd_be16(&raw[6])  / 10.0f;
    out->attmag.mag_x   = (float)(int16_t)srxl2_rd_be16(&raw[8])  / 10.0f;
    out->attmag.mag_y   = (float)(int16_t)srxl2_rd_be16(&raw[10]) / 10.0f;
    out->attmag.mag_z   = (float)(int16_t)srxl2_rd_be16(&raw[12]) / 10.0f;
    out->attmag.heading = (float)srxl2_rd_be16(&raw[14]) / 10.0f;
}

//=============================================================================
// GPS Binary (0x26)
//=============================================================================

static void decode_gps_binary(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_GPS_BINARY;
    out->gps_binary.s_id = raw[1];
    /* altitude: uint16 at [2], 1m resolution, 1000m offset */
    out->gps_binary.altitude = (float)srxl2_rd_be16(&raw[2]) - 1000.0f;
    /* latitude: int32 at [4], degree / 10,000,000 */
    out->gps_binary.latitude  = (float)(int32_t)srxl2_rd_be32(&raw[4])  / 10000000.0f;
    /* longitude: int32 at [8], degree / 10,000,000 */
    out->gps_binary.longitude = (float)(int32_t)srxl2_rd_be32(&raw[8])  / 10000000.0f;
    /* heading: uint16 at [12], degree / 10 */
    out->gps_binary.heading = (float)srxl2_rd_be16(&raw[12]) / 10.0f;
    /* ground speed: uint8 at [14], km/h */
    out->gps_binary.ground_speed = (float)raw[14];
    /* num_sats: uint8 at [15] */
    out->gps_binary.num_sats = raw[15];
}

//=============================================================================
// Vario (0x40)
//=============================================================================

static void decode_vario(const uint8_t raw[16], srxl2_telem_decoded_t *out)
{
    out->type = SRXL2_TELEM_TYPE_VARIO;
    out->vario.s_id = raw[1];
    out->vario.altitude     = (int16_t)srxl2_rd_be16(&raw[2]);
    out->vario.delta_0250ms = (float)(int16_t)srxl2_rd_be16(&raw[4])  / 10.0f;
    out->vario.delta_0500ms = (float)(int16_t)srxl2_rd_be16(&raw[6])  / 10.0f;
    out->vario.delta_1000ms = (float)(int16_t)srxl2_rd_be16(&raw[8])  / 10.0f;
    out->vario.delta_1500ms = (float)(int16_t)srxl2_rd_be16(&raw[10]) / 10.0f;
    out->vario.delta_2000ms = (float)(int16_t)srxl2_rd_be16(&raw[12]) / 10.0f;
    out->vario.delta_3000ms = (float)(int16_t)srxl2_rd_be16(&raw[14]) / 10.0f;
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
    case SRXL2_TELEM_SENSOR_FLITECTRL:
        decode_flitectrl(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_AIRSPEED:
        decode_airspeed(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_GMETER:
        decode_gmeter(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_GYRO:
        decode_gyro(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_ATTMAG:
        decode_attmag(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_ESC:
        decode_esc(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_GPS_BINARY:
        decode_gps_binary(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_FP_MAH:
        decode_fp_mah(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_LIPOMON:
        decode_lipomon(raw, decoded);
        return true;

    case SRXL2_TELEM_SENSOR_VARIO:
        decode_vario(raw, decoded);
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
    case 0x05: return "Flight Controller";
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
    case SRXL2_TELEM_TYPE_FLITECTRL:             return "Flight Controller";
    case SRXL2_TELEM_TYPE_AIRSPEED:              return "Airspeed";
    case SRXL2_TELEM_TYPE_GMETER:                return "G-Meter";
    case SRXL2_TELEM_TYPE_GYRO:                  return "Gyro";
    case SRXL2_TELEM_TYPE_ATTMAG:                return "Attitude/Compass";
    case SRXL2_TELEM_TYPE_GPS_BINARY:            return "GPS Binary";
    case SRXL2_TELEM_TYPE_VARIO:                 return "Vario";
    case SRXL2_TELEM_TYPE_UNKNOWN:
    default:                                     return "Unknown";
    }
}

//=============================================================================
// Encoders
//=============================================================================

void srxl2_encode_esc(srxl2_telem_raw_t payload, const srxl2_telem_esc_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_ESC;
    payload[1] = data->s_id;

    srxl2_wr_be16(&payload[2],  isnan(data->rpm)         ? SRXL2_NO_DATA_16 : (uint16_t)(data->rpm / 10.0f));
    srxl2_wr_be16(&payload[4],  isnan(data->voltage)     ? SRXL2_NO_DATA_16 : (uint16_t)(data->voltage * 100.0f));
    srxl2_wr_be16(&payload[6],  isnan(data->temp_fet)    ? SRXL2_NO_DATA_16 : (uint16_t)(data->temp_fet * 10.0f));
    srxl2_wr_be16(&payload[8],  isnan(data->current)     ? SRXL2_NO_DATA_16 : (uint16_t)(data->current * 100.0f));
    srxl2_wr_be16(&payload[10], isnan(data->temp_bec)    ? SRXL2_NO_DATA_16 : (uint16_t)(data->temp_bec * 10.0f));
    payload[12] = isnan(data->current_bec) ? SRXL2_NO_DATA_8 : (uint8_t)(data->current_bec * 10.0f);
    payload[13] = isnan(data->voltage_bec) ? SRXL2_NO_DATA_8 : (uint8_t)(data->voltage_bec * 20.0f);
    payload[14] = isnan(data->throttle)    ? SRXL2_NO_DATA_8 : (uint8_t)(data->throttle / 0.5f);
    payload[15] = isnan(data->power_out)   ? SRXL2_NO_DATA_8 : (uint8_t)(data->power_out / 0.5f);
}

void srxl2_encode_fp_mah(srxl2_telem_raw_t payload, const srxl2_telem_fp_mah_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_FP_MAH;
    payload[1] = data->s_id;

    srxl2_wr_be16(&payload[2],  (uint16_t)(int16_t)(data->current_a * 10.0f));
    srxl2_wr_be16(&payload[4],  (uint16_t)(int16_t)(data->charge_used_a));
    srxl2_wr_be16(&payload[6],  isnan(data->temp_a) ? 0x7FFF : (uint16_t)(data->temp_a * 10.0f));
    srxl2_wr_be16(&payload[8],  (uint16_t)(int16_t)(data->current_b * 10.0f));
    srxl2_wr_be16(&payload[10], (uint16_t)(int16_t)(data->charge_used_b));
    srxl2_wr_be16(&payload[12], isnan(data->temp_b) ? 0x7FFF : (uint16_t)(data->temp_b * 10.0f));
    srxl2_wr_be16(&payload[14], 0x0000); /* spare */
}

void srxl2_encode_rpm(srxl2_telem_raw_t payload, const srxl2_telem_rpm_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_RPM;
    payload[1] = data->s_id;

    /* RPM -> microseconds per revolution */
    uint16_t us_per_rev = 0;
    if (!isnan(data->rpm) && data->rpm > 0.0f)
        us_per_rev = (uint16_t)(60000000.0f / data->rpm);
    srxl2_wr_be16(&payload[2], us_per_rev);

    srxl2_wr_be16(&payload[4], (uint16_t)(data->voltage * 100.0f));

    /* Temperature: C -> F, 0x7FFF = no data */
    if (isnan(data->temperature)) {
        srxl2_wr_be16(&payload[6], 0x7FFF);
    } else {
        int16_t temp_f = (int16_t)(data->temperature * 9.0f / 5.0f + 32.0f);
        srxl2_wr_be16(&payload[6], (uint16_t)temp_f);
    }

    payload[8] = (uint8_t)data->rssi_a;
    payload[9] = (uint8_t)data->rssi_b;
}

void srxl2_encode_lipomon(srxl2_telem_raw_t payload, const srxl2_telem_lipomon_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_LIPOMON;
    payload[1] = data->s_id;

    for (int i = 0; i < 6; i++)
        srxl2_wr_be16(&payload[2 + i * 2],
                       isnan(data->cell[i]) ? 0x7FFF : (uint16_t)(data->cell[i] * 100.0f));

    srxl2_wr_be16(&payload[14], (uint16_t)(data->temp * 10.0f));
}

void srxl2_encode_flitectrl(srxl2_telem_raw_t payload, const srxl2_telem_flitectrl_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_FLITECTRL;
    payload[1] = data->s_id;
    payload[2] = data->flight_mode & 0x0F;
}

void srxl2_encode_airspeed(srxl2_telem_raw_t payload, const srxl2_telem_airspeed_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_AIRSPEED;
    payload[1] = data->s_id;
    srxl2_wr_be16(&payload[2], (uint16_t)data->speed);
    srxl2_wr_be16(&payload[4], (uint16_t)data->max_speed);
}

void srxl2_encode_gmeter(srxl2_telem_raw_t payload, const srxl2_telem_gmeter_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_GMETER;
    payload[1] = data->s_id;
    srxl2_wr_be16(&payload[2],  (uint16_t)(int16_t)(data->x     * 100.0f));
    srxl2_wr_be16(&payload[4],  (uint16_t)(int16_t)(data->y     * 100.0f));
    srxl2_wr_be16(&payload[6],  (uint16_t)(int16_t)(data->z     * 100.0f));
    srxl2_wr_be16(&payload[8],  (uint16_t)(int16_t)(data->max_x * 100.0f));
    srxl2_wr_be16(&payload[10], (uint16_t)(int16_t)(data->max_y * 100.0f));
    srxl2_wr_be16(&payload[12], (uint16_t)(int16_t)(data->max_z * 100.0f));
    srxl2_wr_be16(&payload[14], (uint16_t)(int16_t)(data->min_z * 100.0f));
}

void srxl2_encode_gyro(srxl2_telem_raw_t payload, const srxl2_telem_gyro_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_GYRO;
    payload[1] = data->s_id;
    srxl2_wr_be16(&payload[2],  (uint16_t)(int16_t)(data->x     * 10.0f));
    srxl2_wr_be16(&payload[4],  (uint16_t)(int16_t)(data->y     * 10.0f));
    srxl2_wr_be16(&payload[6],  (uint16_t)(int16_t)(data->z     * 10.0f));
    srxl2_wr_be16(&payload[8],  (uint16_t)(int16_t)(data->max_x * 10.0f));
    srxl2_wr_be16(&payload[10], (uint16_t)(int16_t)(data->max_y * 10.0f));
    srxl2_wr_be16(&payload[12], (uint16_t)(int16_t)(data->max_z * 10.0f));
}

void srxl2_encode_attmag(srxl2_telem_raw_t payload, const srxl2_telem_attmag_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_ATTMAG;
    payload[1] = data->s_id;
    srxl2_wr_be16(&payload[2],  (uint16_t)(int16_t)(data->roll    * 10.0f));
    srxl2_wr_be16(&payload[4],  (uint16_t)(int16_t)(data->pitch   * 10.0f));
    srxl2_wr_be16(&payload[6],  (uint16_t)(int16_t)(data->yaw     * 10.0f));
    srxl2_wr_be16(&payload[8],  (uint16_t)(int16_t)(data->mag_x   * 10.0f));
    srxl2_wr_be16(&payload[10], (uint16_t)(int16_t)(data->mag_y   * 10.0f));
    srxl2_wr_be16(&payload[12], (uint16_t)(int16_t)(data->mag_z   * 10.0f));
    /* heading doesn't fit - 16 bytes used up by the 7 fields above (2 + 6*2 = 14).
       Actually: identifier(1) + sID(1) + roll(2) + pitch(2) + yaw(2) + magX(2) + magY(2) + magZ(2) = 14 bytes.
       2 bytes remaining for heading at offset 14. */
    srxl2_wr_be16(&payload[14], (uint16_t)(data->heading * 10.0f));
}

void srxl2_encode_gps_binary(srxl2_telem_raw_t payload, const srxl2_telem_gps_binary_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_GPS_BINARY;
    payload[1] = data->s_id;
    /* altitude: 1m + 1000m offset */
    srxl2_wr_be16(&payload[2], (uint16_t)(data->altitude + 1000.0f));
    /* latitude: degree * 10,000,000 */
    srxl2_wr_be32(&payload[4], (uint32_t)(int32_t)(data->latitude * 10000000.0f));
    /* longitude: degree * 10,000,000 */
    srxl2_wr_be32(&payload[8], (uint32_t)(int32_t)(data->longitude * 10000000.0f));
    /* heading: degree * 10 */
    srxl2_wr_be16(&payload[12], (uint16_t)(data->heading * 10.0f));
    /* ground speed: km/h */
    payload[14] = (uint8_t)data->ground_speed;
    /* num_sats */
    payload[15] = data->num_sats;
}

void srxl2_encode_vario(srxl2_telem_raw_t payload, const srxl2_telem_vario_t *data)
{
    memset(payload, 0, 16);
    payload[0] = SRXL2_TELEM_SENSOR_VARIO;
    payload[1] = data->s_id;
    srxl2_wr_be16(&payload[2],  (uint16_t)(int16_t)data->altitude);
    srxl2_wr_be16(&payload[4],  (uint16_t)(int16_t)(data->delta_0250ms * 10.0f));
    srxl2_wr_be16(&payload[6],  (uint16_t)(int16_t)(data->delta_0500ms * 10.0f));
    srxl2_wr_be16(&payload[8],  (uint16_t)(int16_t)(data->delta_1000ms * 10.0f));
    srxl2_wr_be16(&payload[10], (uint16_t)(int16_t)(data->delta_1500ms * 10.0f));
    srxl2_wr_be16(&payload[12], (uint16_t)(int16_t)(data->delta_2000ms * 10.0f));
    srxl2_wr_be16(&payload[14], (uint16_t)(int16_t)(data->delta_3000ms * 10.0f));
}
