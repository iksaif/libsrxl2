/*
 * Telemetry Decoder Unit Tests
 *
 * Tests the standalone telemetry payload decoder for ESC (0x20),
 * Smart Battery (0x42), and edge cases.
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
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Telemetry Decoder Tests ===\n");

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

    TEST_SUMMARY();
}
