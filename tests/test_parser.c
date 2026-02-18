/*
 * SRXL2 Packet Parser Tests
 *
 * Tests the libsrxl2 packet parser: handshake, control, telemetry,
 * validation errors, etc.
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2_packet.h"

///////////////////////////////////////////////////////////////////////////////
// Handshake Parsing
///////////////////////////////////////////////////////////////////////////////

static void test_parse_handshake(void)
{
    TEST_BEGIN(test_parse_handshake);
    // Use the encoder to build a valid handshake packet
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0x40, 20, 0x01, 0x03, 0xDEADBEEF);
    ASSERT_EQ(14, len);

    srxl2_decoded_pkt_t parsed;
    srxl2_parse_result_t result = srxl2_pkt_parse(pkt, len, &parsed);
    ASSERT_EQ(SRXL2_PARSE_OK, result);
    ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, parsed.packet_type);
    ASSERT_EQ_U(0x10, parsed.handshake.src_id);
    ASSERT_EQ_U(0x40, parsed.handshake.dest_id);
    ASSERT_EQ(20, parsed.handshake.priority);
    ASSERT_EQ_U(0x01, parsed.handshake.baud_supported);
    ASSERT_EQ_U(0x03, parsed.handshake.info);
    ASSERT_EQ_U(0xDEADBEEF, parsed.handshake.uid);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Control Data Parsing
///////////////////////////////////////////////////////////////////////////////

static void test_parse_control_channel(void)
{
    TEST_BEGIN(test_parse_control_channel);
    // Use encoder: 1 channel (mask=0x01), value=32768
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0x40,
                                     -50, 0, 0x01, values);

    srxl2_decoded_pkt_t parsed;
    srxl2_parse_result_t result = srxl2_pkt_parse(pkt, len, &parsed);
    ASSERT_EQ(SRXL2_PARSE_OK, result);
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, parsed.packet_type);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL, parsed.control.cmd);
    ASSERT_EQ_U(0x40, parsed.control.reply_id);
    ASSERT_EQ(-50, parsed.control.channel.rssi);
    ASSERT_EQ_U(1, parsed.control.channel.num_channels);
    ASSERT_EQ_U(0x01, parsed.control.channel.mask);
    ASSERT_EQ(32768, parsed.control.channel.values[0]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Parsing
///////////////////////////////////////////////////////////////////////////////

static void test_parse_telemetry(void)
{
    TEST_BEGIN(test_parse_telemetry);
    uint8_t payload[16];
    for (int i = 0; i < 16; i++)
        payload[i] = (uint8_t)i;

    uint8_t pkt[22];
    uint8_t len = srxl2_pkt_telemetry(pkt, 0x10, payload);
    ASSERT_EQ(22, len);

    srxl2_decoded_pkt_t parsed;
    srxl2_parse_result_t result = srxl2_pkt_parse(pkt, len, &parsed);
    ASSERT_EQ(SRXL2_PARSE_OK, result);
    ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, parsed.packet_type);
    ASSERT_EQ_U(0x10, parsed.telemetry.dest_id);
    ASSERT_EQ_U(0x00, parsed.telemetry.payload[0]);
    ASSERT_EQ_U(0x0F, parsed.telemetry.payload[15]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Validation Errors
///////////////////////////////////////////////////////////////////////////////

static void test_parse_too_short(void)
{
    TEST_BEGIN(test_parse_too_short);
    uint8_t pkt[4] = {0xA6, 0x21, 4, 0x00};
    srxl2_decoded_pkt_t parsed;
    ASSERT_EQ(SRXL2_PARSE_ERR_LENGTH, srxl2_pkt_parse(pkt, 4, &parsed));
    TEST_END();
}

static void test_parse_bad_magic(void)
{
    TEST_BEGIN(test_parse_bad_magic);
    uint8_t pkt[14] = {0xB7, 0x21, 14}; // wrong magic
    srxl2_decoded_pkt_t parsed;
    ASSERT_EQ(SRXL2_PARSE_ERR_MAGIC, srxl2_pkt_parse(pkt, 14, &parsed));
    TEST_END();
}

static void test_parse_bad_crc(void)
{
    TEST_BEGIN(test_parse_bad_crc);
    // Build valid handshake, then corrupt CRC
    uint8_t pkt[14];
    srxl2_pkt_handshake(pkt, 0x10, 0x40, 20, 0x01, 0x03, 0x12345678);
    pkt[12] = 0xDE;
    pkt[13] = 0xAD;

    srxl2_decoded_pkt_t parsed;
    ASSERT_EQ(SRXL2_PARSE_ERR_CRC, srxl2_pkt_parse(pkt, 14, &parsed));
    TEST_END();
}

static void test_parse_length_mismatch(void)
{
    TEST_BEGIN(test_parse_length_mismatch);
    // Build valid handshake, then change length field
    uint8_t pkt[14];
    srxl2_pkt_handshake(pkt, 0x10, 0x40, 20, 0x01, 0x03, 0x12345678);
    pkt[2] = 10; // length says 10 but we pass 14
    srxl2_decoded_pkt_t parsed;
    // Should fail because CRC won't match with corrupted length field
    srxl2_parse_result_t result = srxl2_pkt_parse(pkt, 14, &parsed);
    ASSERT_TRUE(result != SRXL2_PARSE_OK);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Parser Tests ===\n");

    // Packet parsing
    RUN_TEST(test_parse_handshake);
    RUN_TEST(test_parse_control_channel);
    RUN_TEST(test_parse_telemetry);

    // Validation errors
    RUN_TEST(test_parse_too_short);
    RUN_TEST(test_parse_bad_magic);
    RUN_TEST(test_parse_bad_crc);
    RUN_TEST(test_parse_length_mismatch);

    TEST_SUMMARY();
}
