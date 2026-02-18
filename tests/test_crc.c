/*
 * CRC-16 Unit Tests
 *
 * Tests libsrxl2's CRC functions: srxl2_crc16() and srxl2_validate_crc().
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2_packet.h"

// Known CRC-16/XMODEM test vectors
static void test_crc_zero(void)
{
    TEST_BEGIN(test_crc_zero);
    uint8_t data[] = {0x00};
    uint16_t crc = srxl2_crc16(data, 1);
    ASSERT_EQ_U(0x0000, crc);
    TEST_END();
}

static void test_crc_one(void)
{
    TEST_BEGIN(test_crc_one);
    uint8_t data[] = {0x01};
    uint16_t crc = srxl2_crc16(data, 1);
    ASSERT_EQ_U(0x1021, crc);
    TEST_END();
}

static void test_crc_srxl_header(void)
{
    TEST_BEGIN(test_crc_srxl_header);
    // A6 21 0E (SRXL2 handshake header)
    uint8_t data[] = {0xA6, 0x21, 0x0E};
    uint16_t crc = srxl2_crc16(data, 3);
    // Just verify it's non-zero and deterministic
    ASSERT_TRUE(crc != 0);
    uint16_t crc2 = srxl2_crc16(data, 3);
    ASSERT_EQ_U(crc, crc2);
    TEST_END();
}

static void test_crc_roundtrip(void)
{
    TEST_BEGIN(test_crc_roundtrip);
    // Build a packet, compute CRC, append it, then validate
    uint8_t pkt[10] = {0xA6, 0x21, 0x0A, 0x10, 0x40, 0x14, 0x00, 0x00};
    pkt[2] = 10; // length including CRC
    uint16_t crc = srxl2_crc16(pkt, 8);
    pkt[8] = (crc >> 8) & 0xFF;
    pkt[9] = crc & 0xFF;
    ASSERT_TRUE(srxl2_validate_crc(pkt, 10));
    TEST_END();
}

static void test_crc_corruption(void)
{
    TEST_BEGIN(test_crc_corruption);
    // Build valid packet, then flip a bit
    uint8_t pkt[10] = {0xA6, 0x21, 0x0A, 0x10, 0x40, 0x14, 0x00, 0x00};
    pkt[2] = 10;
    uint16_t crc = srxl2_crc16(pkt, 8);
    pkt[8] = (crc >> 8) & 0xFF;
    pkt[9] = crc & 0xFF;
    ASSERT_TRUE(srxl2_validate_crc(pkt, 10));

    // Flip bit in data
    pkt[4] ^= 0x01;
    ASSERT_FALSE(srxl2_validate_crc(pkt, 10));
    TEST_END();
}

static void test_crc_null_input(void)
{
    TEST_BEGIN(test_crc_null_input);
    // validate_crc should handle NULL gracefully
    ASSERT_FALSE(srxl2_validate_crc(NULL, 10));
    TEST_END();
}

static void test_crc_zero_length(void)
{
    TEST_BEGIN(test_crc_zero_length);
    uint8_t data[] = {0x01};
    // Zero length should return seed (0)
    uint16_t crc = srxl2_crc16(data, 0);
    ASSERT_EQ_U(0x0000, crc);
    TEST_END();
}

static void test_crc_validate_too_short(void)
{
    TEST_BEGIN(test_crc_validate_too_short);
    uint8_t data[] = {0xA6};
    // Too short for CRC validation (need at least 5 bytes)
    ASSERT_FALSE(srxl2_validate_crc(data, 1));
    TEST_END();
}

int main(void)
{
    printf("=== CRC Tests ===\n");
    RUN_TEST(test_crc_zero);
    RUN_TEST(test_crc_one);
    RUN_TEST(test_crc_srxl_header);
    RUN_TEST(test_crc_roundtrip);
    RUN_TEST(test_crc_corruption);
    RUN_TEST(test_crc_null_input);
    RUN_TEST(test_crc_zero_length);
    RUN_TEST(test_crc_validate_too_short);
    TEST_SUMMARY();
}
