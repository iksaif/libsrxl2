/*
 * Packet Codec Tests for libsrxl2
 *
 * Tests CRC, encode/decode roundtrip for all packet types,
 * and error handling for malformed packets.
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2_packet.h"
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// CRC Known Vectors
///////////////////////////////////////////////////////////////////////////////

static void test_crc_empty(void)
{
    TEST_BEGIN(test_crc_empty);
    ASSERT_EQ_U(0x0000, srxl2_crc16(NULL, 0));
    TEST_END();
}

static void test_crc_known_vector(void)
{
    TEST_BEGIN(test_crc_known_vector);
    /* The CRC of a handshake header A6 21 0E should be a known value */
    uint8_t data[] = {0xA6, 0x21, 0x0E};
    uint16_t crc = srxl2_crc16(data, 3);
    /* Verify non-zero and deterministic */
    ASSERT_TRUE(crc != 0);
    ASSERT_EQ_U(crc, srxl2_crc16(data, 3));
    TEST_END();
}

static void test_crc_matches_legacy(void)
{
    TEST_BEGIN(test_crc_matches_legacy);
    /* Bitwise reference implementation (same as test_helpers.c) */
    uint8_t data[] = {0xA6, 0x21, 0x0E, 0x10, 0x40, 0x14, 0x01, 0x00,
                      0x78, 0x56, 0x34, 0x12};
    uint16_t crc_ref = 0;
    for (size_t i = 0; i < sizeof(data); i++) {
        crc_ref ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc_ref & 0x8000)
                crc_ref = (uint16_t)((crc_ref << 1) ^ 0x1021);
            else
                crc_ref = (uint16_t)(crc_ref << 1);
        }
    }
    ASSERT_EQ_U(crc_ref, srxl2_crc16(data, sizeof(data)));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_handshake_roundtrip(void)
{
    TEST_BEGIN(test_handshake_roundtrip);
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_handshake(buf, 0x10, 0x40, 20, 0x01, 0x03, 0xDEADBEEF);
    ASSERT_EQ(14, len);
    ASSERT_EQ_U(0xA6, buf[0]);
    ASSERT_EQ_U(0x21, buf[1]);
    ASSERT_EQ(14, buf[2]);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, pkt.packet_type);
    ASSERT_EQ_U(0x10, pkt.handshake.src_id);
    ASSERT_EQ_U(0x40, pkt.handshake.dest_id);
    ASSERT_EQ(20, pkt.handshake.priority);
    ASSERT_EQ_U(0x01, pkt.handshake.baud_supported);
    ASSERT_EQ_U(0x03, pkt.handshake.info);
    ASSERT_EQ_U(0xDEADBEEF, pkt.handshake.uid);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Channel Data Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_channel_1ch_roundtrip(void)
{
    TEST_BEGIN(test_channel_1ch_roundtrip);
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_channel(buf, SRXL2_CMD_CHANNEL, 0x40,
                                     -50, 100, 0x01, values);
    /* 14 + 1*2 = 16 */
    ASSERT_EQ(16, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt.packet_type);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL, pkt.control.cmd);
    ASSERT_EQ_U(0x40, pkt.control.reply_id);
    ASSERT_EQ(-50, (int8_t)pkt.control.channel.rssi);
    ASSERT_EQ(100, pkt.control.channel.frame_losses);
    ASSERT_EQ_U(0x01, pkt.control.channel.mask);
    ASSERT_EQ(32768, pkt.control.channel.values[0]);
    ASSERT_EQ(1, pkt.control.channel.num_channels);
    TEST_END();
}

static void test_channel_4ch_roundtrip(void)
{
    TEST_BEGIN(test_channel_4ch_roundtrip);
    uint16_t values[32] = {0};
    values[0] = 1000; values[1] = 2000; values[2] = 3000; values[3] = 4000;
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_channel(buf, SRXL2_CMD_CHANNEL, 0xB0,
                                     -30, 0, 0x0F, values);
    /* 14 + 4*2 = 22 */
    ASSERT_EQ(22, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(0x0F, pkt.control.channel.mask);
    ASSERT_EQ(1000, pkt.control.channel.values[0]);
    ASSERT_EQ(2000, pkt.control.channel.values[1]);
    ASSERT_EQ(3000, pkt.control.channel.values[2]);
    ASSERT_EQ(4000, pkt.control.channel.values[3]);
    ASSERT_EQ(4, pkt.control.channel.num_channels);
    TEST_END();
}

static void test_channel_failsafe_roundtrip(void)
{
    TEST_BEGIN(test_channel_failsafe_roundtrip);
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_channel(buf, SRXL2_CMD_CHANNEL_FS, 0x00,
                                     -80, 50, 0x01, values);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL_FS, pkt.control.cmd);
    ASSERT_EQ_U(0x00, pkt.control.reply_id);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_telemetry_roundtrip(void)
{
    TEST_BEGIN(test_telemetry_roundtrip);
    uint8_t payload[16] = {0x20, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                           0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_telemetry(buf, 0x10, payload);
    ASSERT_EQ(22, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, pkt.packet_type);
    ASSERT_EQ_U(0x10, pkt.telemetry.dest_id);
    ASSERT_MEM_EQ(payload, pkt.telemetry.payload, 16);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Bind Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_bind_roundtrip(void)
{
    TEST_BEGIN(test_bind_roundtrip);
    srxl2_bind_data_t data = {
        .type = 0xB2,
        .options = 0x01,
        .guid = 0x0102030405060708ULL,
        .uid = 0xDEADBEEF,
    };
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_bind(buf, SRXL2_BIND_REQ_ENTER, 0x21, &data);
    ASSERT_EQ(21, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_BIND, pkt.packet_type);
    ASSERT_EQ_U(SRXL2_BIND_REQ_ENTER, pkt.bind.request);
    ASSERT_EQ_U(0x21, pkt.bind.device_id);
    ASSERT_EQ_U(0xB2, pkt.bind.data.type);
    ASSERT_EQ_U(0x01, pkt.bind.data.options);
    ASSERT_EQ_U(0x0102030405060708ULL, pkt.bind.data.guid);
    ASSERT_EQ_U(0xDEADBEEF, pkt.bind.data.uid);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// VTX Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_vtx_roundtrip(void)
{
    TEST_BEGIN(test_vtx_roundtrip);
    srxl2_vtx_data_t vtx = {
        .band = 1, .channel = 5, .pit = 0, .power = 4,
        .power_mw = 250, .region = 0,
    };
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_vtx(buf, 0x81, &vtx);
    ASSERT_EQ(14, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt.packet_type);
    ASSERT_EQ_U(SRXL2_CMD_VTX, pkt.control.cmd);
    ASSERT_EQ_U(0x81, pkt.control.reply_id);
    ASSERT_EQ(1, pkt.control.vtx.band);
    ASSERT_EQ(5, pkt.control.vtx.channel);
    ASSERT_EQ(0, pkt.control.vtx.pit);
    ASSERT_EQ(4, pkt.control.vtx.power);
    ASSERT_EQ(250, pkt.control.vtx.power_mw);
    ASSERT_EQ(0, pkt.control.vtx.region);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Forward Programming Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_fwd_pgm_roundtrip(void)
{
    TEST_BEGIN(test_fwd_pgm_roundtrip);
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_fwd_pgm(buf, 0x40, -20, data, 8);
    /* 10 + 8 = 18 */
    ASSERT_EQ(18, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt.packet_type);
    ASSERT_EQ_U(SRXL2_CMD_FWDPGM, pkt.control.cmd);
    ASSERT_EQ(-20, (int8_t)pkt.control.fwd_pgm.rssi);
    ASSERT_EQ(8, pkt.control.fwd_pgm.len);
    ASSERT_MEM_EQ(data, pkt.control.fwd_pgm.data, 8);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// RSSI Roundtrip
///////////////////////////////////////////////////////////////////////////////

static void test_rssi_roundtrip(void)
{
    TEST_BEGIN(test_rssi_roundtrip);
    uint8_t buf[80];
    uint8_t len = srxl2_pkt_rssi(buf, 0x53, -40, -50, -60, -70);
    ASSERT_EQ(10, len);

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_OK, srxl2_pkt_parse(buf, len, &pkt));
    ASSERT_EQ_U(SRXL2_PKT_RSSI, pkt.packet_type);
    ASSERT_EQ_U(0x53, pkt.rssi.request);
    ASSERT_EQ(-40, pkt.rssi.antenna_a);
    ASSERT_EQ(-50, pkt.rssi.antenna_b);
    ASSERT_EQ(-60, pkt.rssi.antenna_c);
    ASSERT_EQ(-70, pkt.rssi.antenna_d);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Error Cases
///////////////////////////////////////////////////////////////////////////////

static void test_parse_bad_magic(void)
{
    TEST_BEGIN(test_parse_bad_magic);
    uint8_t buf[14] = {0xB7, 0x21, 0x0E}; /* wrong magic */
    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_ERR_MAGIC, srxl2_pkt_parse(buf, 14, &pkt));
    TEST_END();
}

static void test_parse_bad_crc(void)
{
    TEST_BEGIN(test_parse_bad_crc);
    uint8_t buf[80];
    srxl2_pkt_handshake(buf, 0x10, 0x40, 20, 0x01, 0x00, 0x12345678);
    buf[12] ^= 0xFF; /* corrupt CRC */

    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_ERR_CRC, srxl2_pkt_parse(buf, 14, &pkt));
    TEST_END();
}

static void test_parse_truncated(void)
{
    TEST_BEGIN(test_parse_truncated);
    uint8_t buf[80];
    srxl2_pkt_handshake(buf, 0x10, 0x40, 20, 0x01, 0x00, 0x12345678);

    srxl2_decoded_pkt_t pkt;
    /* Pass only 10 bytes when packet says 14 */
    ASSERT_EQ(SRXL2_PARSE_ERR_LENGTH, srxl2_pkt_parse(buf, 10, &pkt));
    TEST_END();
}

static void test_parse_too_short(void)
{
    TEST_BEGIN(test_parse_too_short);
    uint8_t buf[4] = {0xA6, 0x21, 0x0E, 0x00};
    srxl2_decoded_pkt_t pkt;
    ASSERT_EQ(SRXL2_PARSE_ERR_LENGTH, srxl2_pkt_parse(buf, 4, &pkt));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Byte-order helpers
///////////////////////////////////////////////////////////////////////////////

static void test_byte_order_helpers(void)
{
    TEST_BEGIN(test_byte_order_helpers);
    uint8_t buf[8];

    srxl2_wr_le16(buf, 0x1234);
    ASSERT_EQ_U(0x34, buf[0]);
    ASSERT_EQ_U(0x12, buf[1]);
    ASSERT_EQ_U(0x1234, srxl2_rd_le16(buf));

    srxl2_wr_le32(buf, 0xDEADBEEF);
    ASSERT_EQ_U(0xEF, buf[0]);
    ASSERT_EQ_U(0xBE, buf[1]);
    ASSERT_EQ_U(0xAD, buf[2]);
    ASSERT_EQ_U(0xDE, buf[3]);
    ASSERT_EQ_U(0xDEADBEEF, srxl2_rd_le32(buf));

    srxl2_wr_be16(buf, 0xABCD);
    ASSERT_EQ_U(0xAB, buf[0]);
    ASSERT_EQ_U(0xCD, buf[1]);
    ASSERT_EQ_U(0xABCD, srxl2_rd_be16(buf));

    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== libsrxl2 Packet Codec Tests ===\n");

    /* CRC */
    RUN_TEST(test_crc_empty);
    RUN_TEST(test_crc_known_vector);
    RUN_TEST(test_crc_matches_legacy);

    /* Roundtrip */
    RUN_TEST(test_handshake_roundtrip);
    RUN_TEST(test_channel_1ch_roundtrip);
    RUN_TEST(test_channel_4ch_roundtrip);
    RUN_TEST(test_channel_failsafe_roundtrip);
    RUN_TEST(test_telemetry_roundtrip);
    RUN_TEST(test_bind_roundtrip);
    RUN_TEST(test_vtx_roundtrip);
    RUN_TEST(test_fwd_pgm_roundtrip);
    RUN_TEST(test_rssi_roundtrip);

    /* Error cases */
    RUN_TEST(test_parse_bad_magic);
    RUN_TEST(test_parse_bad_crc);
    RUN_TEST(test_parse_truncated);
    RUN_TEST(test_parse_too_short);

    /* Helpers */
    RUN_TEST(test_byte_order_helpers);

    TEST_SUMMARY();
}
