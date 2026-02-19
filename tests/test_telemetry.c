/*
 * Telemetry Decoder & Encoder Unit Tests
 *
 * Tests the standalone telemetry payload decoder for ESC (0x20),
 * Smart Battery (0x42), and edge cases. Also tests round-trip
 * encode/decode for all 11 supported sensor types.
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2_telemetry.h"

///////////////////////////////////////////////////////////////////////////////
// ESC Telemetry (0x20)
///////////////////////////////////////////////////////////////////////////////

static void test_esc_normal_values(void)
{
    TEST_BEGIN(test_esc_normal_values);
    // Build a big-endian ESC payload
    // RPM: 1000 (0x03E8) → 10000 RPM
    // Voltage: 2500 (0x09C4) → 25.00V
    // TempFET: 500 (0x01F4) → 50.0C
    // Current: 5000 (0x1388) → 50.00A
    // TempBEC: 350 (0x015E) → 35.0C
    // CurrentBEC: 20 → 2.0A
    // VoltageBEC: 100 → 5.0V
    // Throttle: 100 → 50.0%
    // PowerOut: 160 → 80.0%
    uint8_t raw[16] = {
        0x20, 0x00,             // identifier, s_id
        0x03, 0xE8,             // RPM (BE)
        0x09, 0xC4,             // voltage (BE)
        0x01, 0xF4,             // temp_fet (BE)
        0x13, 0x88,             // current_motor (BE)
        0x01, 0x5E,             // temp_bec (BE)
        20,                     // current_bec
        100,                    // voltage_bec
        100,                    // throttle
        160,                    // power_out
    };

    srxl2_telem_decoded_t decoded;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_ESC, decoded.type);
    ASSERT_EQ_U(0x20, decoded.sensor_id);

    ASSERT_FLOAT_EQ(10000.0f, decoded.esc.rpm, 1.0f);
    ASSERT_FLOAT_EQ(25.0f, decoded.esc.voltage, 0.01f);
    ASSERT_FLOAT_EQ(50.0f, decoded.esc.temp_fet, 0.1f);
    ASSERT_FLOAT_EQ(50.0f, decoded.esc.current, 0.01f);
    ASSERT_FLOAT_EQ(35.0f, decoded.esc.temp_bec, 0.1f);
    ASSERT_FLOAT_EQ(2.0f, decoded.esc.current_bec, 0.1f);
    ASSERT_FLOAT_EQ(5.0f, decoded.esc.voltage_bec, 0.01f);
    ASSERT_FLOAT_EQ(50.0f, decoded.esc.throttle, 0.5f);
    ASSERT_FLOAT_EQ(80.0f, decoded.esc.power_out, 0.5f);
    TEST_END();
}

static void test_esc_all_nodata(void)
{
    TEST_BEGIN(test_esc_all_nodata);
    uint8_t raw[16];
    memset(raw, 0xFF, 16);
    raw[0] = 0x20; // identifier
    raw[1] = 0x00; // s_id

    srxl2_telem_decoded_t decoded;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_ESC, decoded.type);

    ASSERT_NAN(decoded.esc.rpm);
    ASSERT_NAN(decoded.esc.voltage);
    ASSERT_NAN(decoded.esc.temp_fet);
    ASSERT_NAN(decoded.esc.current);
    ASSERT_NAN(decoded.esc.temp_bec);
    ASSERT_NAN(decoded.esc.current_bec);
    ASSERT_NAN(decoded.esc.voltage_bec);
    ASSERT_NAN(decoded.esc.throttle);
    ASSERT_NAN(decoded.esc.power_out);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Smart Battery (0x42)
///////////////////////////////////////////////////////////////////////////////

static void test_smart_bat_realtime(void)
{
    TEST_BEGIN(test_smart_bat_realtime);
    // sub=0x00 (realtime)
    // temp=25C, current=2000mA (BE: 0x000007D0), consumption=500 (0x01F4 BE → 50.0 mAh)
    // min_cell=3700mV (0x0E74 BE), max_cell=4200mV (0x1068 BE)
    uint8_t raw[16] = {
        0x42, 0x00,             // identifier, s_id
        0x00,                   // type = realtime
        25,                     // temp = 25C
        0x00, 0x00, 0x07, 0xD0, // current 2000mA (BE)
        0x01, 0xF4,             // consumption (BE)
        0x0E, 0x74,             // min_cell (BE)
        0x10, 0x68,             // max_cell (BE)
        0x00, 0x00,             // padding
    };

    srxl2_telem_decoded_t decoded;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_SMART_BAT_REALTIME, decoded.type);

    ASSERT_FLOAT_EQ(25.0f, decoded.smart_bat_realtime.temp, 0.1f);
    ASSERT_FLOAT_EQ(2.0f, decoded.smart_bat_realtime.current, 0.01f);
    ASSERT_FLOAT_EQ(50.0f, decoded.smart_bat_realtime.consumption, 0.1f);
    ASSERT_FLOAT_EQ(3.7f, decoded.smart_bat_realtime.min_cell, 0.001f);
    ASSERT_FLOAT_EQ(4.2f, decoded.smart_bat_realtime.max_cell, 0.001f);
    TEST_END();
}

static void test_smart_bat_cells(void)
{
    TEST_BEGIN(test_smart_bat_cells);
    // sub=0x10 (cells block 1)
    // 6 cells at various voltages (mV, big-endian)
    uint8_t raw[16] = {
        0x42, 0x00,             // identifier, s_id
        0x10,                   // type = cells_1
        30,                     // temp = 30C
        0x0E, 0x74,             // cell 0: 3700mV (BE)
        0x0E, 0xD8,             // cell 1: 3800mV (BE)
        0x0F, 0x3C,             // cell 2: 3900mV (BE)
        0x0F, 0xA0,             // cell 3: 4000mV (BE)
        0x10, 0x04,             // cell 4: 4100mV (BE)
        0x10, 0x68,             // cell 5: 4200mV (BE)
    };

    srxl2_telem_decoded_t decoded;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_SMART_BAT_CELLS, decoded.type);

    ASSERT_FLOAT_EQ(3.7f, decoded.smart_bat_cells.cell[0], 0.001f);
    ASSERT_FLOAT_EQ(3.8f, decoded.smart_bat_cells.cell[1], 0.001f);
    ASSERT_FLOAT_EQ(3.9f, decoded.smart_bat_cells.cell[2], 0.001f);
    ASSERT_FLOAT_EQ(4.0f, decoded.smart_bat_cells.cell[3], 0.001f);
    ASSERT_FLOAT_EQ(4.1f, decoded.smart_bat_cells.cell[4], 0.001f);
    ASSERT_FLOAT_EQ(4.2f, decoded.smart_bat_cells.cell[5], 0.001f);
    TEST_END();
}

static void test_smart_bat_id(void)
{
    TEST_BEGIN(test_smart_bat_id);
    // sub=0x80 (ID)
    uint8_t raw[16] = {
        0x42, 0x00,             // identifier, s_id
        0x80,                   // type = ID
        0x01,                   // chemistry = 1 (LiPo)
        6,                      // cells = 6
        0x02,                   // mfg_code
        0x00, 0x64,             // cycles = 100 (BE)
        0x42,                   // uid
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // padding
    };

    srxl2_telem_decoded_t decoded;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_SMART_BAT_ID, decoded.type);

    ASSERT_EQ(6, decoded.smart_bat_id.cells);
    ASSERT_EQ(100, decoded.smart_bat_id.cycles);
    ASSERT_EQ(1, decoded.smart_bat_id.chemistry);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Edge Cases
///////////////////////////////////////////////////////////////////////////////

static void test_unknown_sensor(void)
{
    TEST_BEGIN(test_unknown_sensor);
    uint8_t raw[16] = {0x99}; // Unknown sensor
    srxl2_telem_decoded_t decoded;
    ASSERT_FALSE(srxl2_decode_telemetry(raw, &decoded));
    ASSERT_EQ(SRXL2_TELEM_TYPE_UNKNOWN, decoded.type);
    TEST_END();
}

static void test_null_input(void)
{
    TEST_BEGIN(test_null_input);
    srxl2_telem_decoded_t decoded;
    ASSERT_FALSE(srxl2_decode_telemetry(NULL, &decoded));
    ASSERT_FALSE(srxl2_decode_telemetry(NULL, NULL));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Round-trip Encode/Decode Tests
///////////////////////////////////////////////////////////////////////////////

static void test_roundtrip_esc(void)
{
    TEST_BEGIN(test_roundtrip_esc);
    srxl2_telem_esc_t in = {
        .rpm = 15000.0f, .voltage = 22.5f, .current = 35.0f,
        .temp_fet = 65.0f, .temp_bec = 40.0f,
        .current_bec = 1.5f, .voltage_bec = 5.0f,
        .throttle = 75.0f, .power_out = 80.0f, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_esc(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_ESC, dec.type);
    ASSERT_FLOAT_EQ(15000.0f, dec.esc.rpm, 10.0f);
    ASSERT_FLOAT_EQ(22.5f, dec.esc.voltage, 0.02f);
    ASSERT_FLOAT_EQ(35.0f, dec.esc.current, 0.02f);
    ASSERT_FLOAT_EQ(65.0f, dec.esc.temp_fet, 0.2f);
    ASSERT_FLOAT_EQ(40.0f, dec.esc.temp_bec, 0.2f);
    ASSERT_FLOAT_EQ(1.5f, dec.esc.current_bec, 0.2f);
    ASSERT_FLOAT_EQ(5.0f, dec.esc.voltage_bec, 0.06f);
    ASSERT_FLOAT_EQ(75.0f, dec.esc.throttle, 1.0f);
    ASSERT_FLOAT_EQ(80.0f, dec.esc.power_out, 1.0f);
    TEST_END();
}

static void test_roundtrip_esc_nodata(void)
{
    TEST_BEGIN(test_roundtrip_esc_nodata);
    srxl2_telem_esc_t in = {
        .rpm = NAN, .voltage = NAN, .current = NAN,
        .temp_fet = NAN, .temp_bec = NAN,
        .current_bec = NAN, .voltage_bec = NAN,
        .throttle = NAN, .power_out = NAN, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_esc(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_NAN(dec.esc.rpm);
    ASSERT_NAN(dec.esc.voltage);
    ASSERT_NAN(dec.esc.current);
    ASSERT_NAN(dec.esc.temp_fet);
    ASSERT_NAN(dec.esc.temp_bec);
    ASSERT_NAN(dec.esc.current_bec);
    ASSERT_NAN(dec.esc.voltage_bec);
    ASSERT_NAN(dec.esc.throttle);
    ASSERT_NAN(dec.esc.power_out);
    TEST_END();
}

static void test_roundtrip_fp_mah(void)
{
    TEST_BEGIN(test_roundtrip_fp_mah);
    srxl2_telem_fp_mah_t in = {
        .current_a = 12.5f, .charge_used_a = 1500.0f, .temp_a = 35.0f,
        .current_b = 0.0f, .charge_used_b = 0.0f, .temp_b = NAN,
        .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_fp_mah(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_FP_MAH, dec.type);
    ASSERT_FLOAT_EQ(12.5f, dec.fp_mah.current_a, 0.2f);
    ASSERT_FLOAT_EQ(1500.0f, dec.fp_mah.charge_used_a, 1.0f);
    ASSERT_FLOAT_EQ(35.0f, dec.fp_mah.temp_a, 0.2f);
    ASSERT_NAN(dec.fp_mah.temp_b);
    TEST_END();
}

static void test_roundtrip_rpm(void)
{
    TEST_BEGIN(test_roundtrip_rpm);
    srxl2_telem_rpm_t in = {
        .rpm = 15000.0f, .voltage = 22.2f, .temperature = NAN,
        .rssi_a = -50, .rssi_b = -60, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_rpm(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_RPM, dec.type);
    ASSERT_FLOAT_EQ(15000.0f, dec.rpm.rpm, 100.0f);
    ASSERT_FLOAT_EQ(22.2f, dec.rpm.voltage, 0.02f);
    ASSERT_NAN(dec.rpm.temperature);
    ASSERT_EQ(-50, dec.rpm.rssi_a);
    ASSERT_EQ(-60, dec.rpm.rssi_b);
    TEST_END();
}

static void test_roundtrip_lipomon(void)
{
    TEST_BEGIN(test_roundtrip_lipomon);
    srxl2_telem_lipomon_t in = {
        .cell = {3.70f, 3.80f, 3.90f, 4.00f, NAN, NAN},
        .temp = 30.0f, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_lipomon(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_LIPOMON, dec.type);
    ASSERT_FLOAT_EQ(3.70f, dec.lipomon.cell[0], 0.02f);
    ASSERT_FLOAT_EQ(3.80f, dec.lipomon.cell[1], 0.02f);
    ASSERT_FLOAT_EQ(3.90f, dec.lipomon.cell[2], 0.02f);
    ASSERT_FLOAT_EQ(4.00f, dec.lipomon.cell[3], 0.02f);
    ASSERT_NAN(dec.lipomon.cell[4]);
    ASSERT_NAN(dec.lipomon.cell[5]);
    ASSERT_FLOAT_EQ(30.0f, dec.lipomon.temp, 0.2f);
    TEST_END();
}

static void test_roundtrip_flitectrl(void)
{
    TEST_BEGIN(test_roundtrip_flitectrl);
    srxl2_telem_flitectrl_t in = { .flight_mode = 3, .s_id = 0 };
    srxl2_telem_raw_t raw;
    srxl2_encode_flitectrl(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_FLITECTRL, dec.type);
    ASSERT_EQ(3, dec.flitectrl.flight_mode);
    TEST_END();
}

static void test_roundtrip_airspeed(void)
{
    TEST_BEGIN(test_roundtrip_airspeed);
    srxl2_telem_airspeed_t in = { .speed = 120.0f, .max_speed = 150.0f, .s_id = 0 };
    srxl2_telem_raw_t raw;
    srxl2_encode_airspeed(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_AIRSPEED, dec.type);
    ASSERT_FLOAT_EQ(120.0f, dec.airspeed.speed, 1.0f);
    ASSERT_FLOAT_EQ(150.0f, dec.airspeed.max_speed, 1.0f);
    TEST_END();
}

static void test_roundtrip_gmeter(void)
{
    TEST_BEGIN(test_roundtrip_gmeter);
    srxl2_telem_gmeter_t in = {
        .x = 1.5f, .y = -0.5f, .z = 9.8f,
        .max_x = 2.0f, .max_y = 1.0f, .max_z = 10.0f, .min_z = -1.0f,
        .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_gmeter(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_GMETER, dec.type);
    ASSERT_FLOAT_EQ(1.5f, dec.gmeter.x, 0.02f);
    ASSERT_FLOAT_EQ(-0.5f, dec.gmeter.y, 0.02f);
    ASSERT_FLOAT_EQ(9.8f, dec.gmeter.z, 0.02f);
    ASSERT_FLOAT_EQ(2.0f, dec.gmeter.max_x, 0.02f);
    ASSERT_FLOAT_EQ(-1.0f, dec.gmeter.min_z, 0.02f);
    TEST_END();
}

static void test_roundtrip_gyro(void)
{
    TEST_BEGIN(test_roundtrip_gyro);
    srxl2_telem_gyro_t in = {
        .x = 100.5f, .y = -50.0f, .z = 25.0f,
        .max_x = 200.0f, .max_y = 100.0f, .max_z = 50.0f,
        .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_gyro(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_GYRO, dec.type);
    ASSERT_FLOAT_EQ(100.5f, dec.gyro.x, 0.2f);
    ASSERT_FLOAT_EQ(-50.0f, dec.gyro.y, 0.2f);
    ASSERT_FLOAT_EQ(25.0f, dec.gyro.z, 0.2f);
    ASSERT_FLOAT_EQ(200.0f, dec.gyro.max_x, 0.2f);
    TEST_END();
}

static void test_roundtrip_attmag(void)
{
    TEST_BEGIN(test_roundtrip_attmag);
    srxl2_telem_attmag_t in = {
        .roll = 15.5f, .pitch = -10.0f, .yaw = 180.0f,
        .mag_x = 25.0f, .mag_y = -30.0f, .mag_z = 45.0f,
        .heading = 270.0f, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_attmag(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_ATTMAG, dec.type);
    ASSERT_FLOAT_EQ(15.5f, dec.attmag.roll, 0.2f);
    ASSERT_FLOAT_EQ(-10.0f, dec.attmag.pitch, 0.2f);
    ASSERT_FLOAT_EQ(180.0f, dec.attmag.yaw, 0.2f);
    ASSERT_FLOAT_EQ(25.0f, dec.attmag.mag_x, 0.2f);
    ASSERT_FLOAT_EQ(-30.0f, dec.attmag.mag_y, 0.2f);
    ASSERT_FLOAT_EQ(270.0f, dec.attmag.heading, 0.2f);
    TEST_END();
}

static void test_roundtrip_gps_binary(void)
{
    TEST_BEGIN(test_roundtrip_gps_binary);
    srxl2_telem_gps_binary_t in = {
        .altitude = 150.0f,
        .latitude = 48.8566f,     /* Paris */
        .longitude = 2.3522f,
        .heading = 90.0f,
        .ground_speed = 50.0f,
        .num_sats = 12, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_gps_binary(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_GPS_BINARY, dec.type);
    ASSERT_FLOAT_EQ(150.0f, dec.gps_binary.altitude, 1.0f);
    ASSERT_FLOAT_EQ(48.8566f, dec.gps_binary.latitude, 0.0001f);
    ASSERT_FLOAT_EQ(2.3522f, dec.gps_binary.longitude, 0.0001f);
    ASSERT_FLOAT_EQ(90.0f, dec.gps_binary.heading, 0.2f);
    ASSERT_FLOAT_EQ(50.0f, dec.gps_binary.ground_speed, 1.0f);
    ASSERT_EQ(12, dec.gps_binary.num_sats);
    TEST_END();
}

static void test_roundtrip_gps_negative_coords(void)
{
    TEST_BEGIN(test_roundtrip_gps_negative_coords);
    srxl2_telem_gps_binary_t in = {
        .altitude = -50.0f,       /* Below sea level */
        .latitude = -33.8688f,    /* Sydney */
        .longitude = 151.2093f,
        .heading = 0.0f,
        .ground_speed = 0.0f,
        .num_sats = 8, .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_gps_binary(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_FLOAT_EQ(-50.0f, dec.gps_binary.altitude, 1.0f);
    ASSERT_FLOAT_EQ(-33.8688f, dec.gps_binary.latitude, 0.0001f);
    ASSERT_FLOAT_EQ(151.2093f, dec.gps_binary.longitude, 0.001f);
    TEST_END();
}

static void test_roundtrip_vario(void)
{
    TEST_BEGIN(test_roundtrip_vario);
    srxl2_telem_vario_t in = {
        .altitude = 500,
        .delta_0250ms = 2.5f, .delta_0500ms = 2.0f, .delta_1000ms = 1.5f,
        .delta_1500ms = 1.0f, .delta_2000ms = 0.5f, .delta_3000ms = -0.5f,
        .s_id = 0,
    };
    srxl2_telem_raw_t raw;
    srxl2_encode_vario(raw, &in);

    srxl2_telem_decoded_t dec;
    ASSERT_TRUE(srxl2_decode_telemetry(raw, &dec));
    ASSERT_EQ(SRXL2_TELEM_TYPE_VARIO, dec.type);
    ASSERT_EQ(500, dec.vario.altitude);
    ASSERT_FLOAT_EQ(2.5f, dec.vario.delta_0250ms, 0.2f);
    ASSERT_FLOAT_EQ(2.0f, dec.vario.delta_0500ms, 0.2f);
    ASSERT_FLOAT_EQ(1.5f, dec.vario.delta_1000ms, 0.2f);
    ASSERT_FLOAT_EQ(-0.5f, dec.vario.delta_3000ms, 0.2f);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Telemetry Decoder & Encoder Tests ===\n");

    // ESC
    RUN_TEST(test_esc_normal_values);
    RUN_TEST(test_esc_all_nodata);

    // Smart Battery
    RUN_TEST(test_smart_bat_realtime);
    RUN_TEST(test_smart_bat_cells);
    RUN_TEST(test_smart_bat_id);

    // Edge cases
    RUN_TEST(test_unknown_sensor);
    RUN_TEST(test_null_input);

    // Round-trip encode/decode (all 11 types: 4 existing + 7 new)
    RUN_TEST(test_roundtrip_esc);
    RUN_TEST(test_roundtrip_esc_nodata);
    RUN_TEST(test_roundtrip_fp_mah);
    RUN_TEST(test_roundtrip_rpm);
    RUN_TEST(test_roundtrip_lipomon);
    RUN_TEST(test_roundtrip_flitectrl);
    RUN_TEST(test_roundtrip_airspeed);
    RUN_TEST(test_roundtrip_gmeter);
    RUN_TEST(test_roundtrip_gyro);
    RUN_TEST(test_roundtrip_attmag);
    RUN_TEST(test_roundtrip_gps_binary);
    RUN_TEST(test_roundtrip_gps_negative_coords);
    RUN_TEST(test_roundtrip_vario);

    TEST_SUMMARY();
}
