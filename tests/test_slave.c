/*
 * Slave State Machine Tests
 *
 * Tests the SRXL2 slave behavior: startup, handshake response,
 * channel data reception, telemetry replies, timeout/recovery, failsafe.
 *
 * MIT License
 */

#include "test_harness.h"
#include "test_helpers.h"
#include "uart_adapter.h"

///////////////////////////////////////////////////////////////////////////////
// Startup
///////////////////////////////////////////////////////////////////////////////

static void test_slave_starts_in_listen_on_startup(void)
{
    TEST_BEGIN(test_slave_starts_in_listen_on_startup);
    test_init_slave(0xB1);
    ASSERT_EQ(SrxlState_ListenOnStartup, srxlBus[0].state);
    TEST_END();
}

static void test_slave_sends_unprompted_handshake_on_timeout(void)
{
    TEST_BEGIN(test_slave_sends_unprompted_handshake_on_timeout);
    test_init_slave(0xB0); // unit ID 0 → sends unprompted handshake
    test_advance_ms(0, 50);

    // State should now be SendHandshake
    ASSERT_EQ(SrxlState_SendHandshake, srxlBus[0].state);

    // The slave state machine runs after timeout detection, so
    // a handshake should have been sent
    ASSERT_TRUE(g_tx_count > 0);

    const uint8_t *pkt;
    size_t pkt_len;
    capture_get_packet(0, &pkt, &pkt_len);
    ASSERT_EQ_U(0xA6, pkt[0]);
    ASSERT_EQ_U(0x21, pkt[1]); // Handshake
    ASSERT_EQ_U(0xB0, pkt[3]); // srcDevID = us
    // dest=0 for unprompted (requestID was set to 0 since we haven't heard from anyone)
    TEST_END();
}

static void test_slave_with_nonzero_unit_listens(void)
{
    TEST_BEGIN(test_slave_with_nonzero_unit_listens);
    test_init_slave(0xB1); // unit ID 1
    test_advance_ms(0, 50);
    ASSERT_EQ(SrxlState_ListenForHandshake, srxlBus[0].state);
    // No packet should have been sent
    ASSERT_EQ(0, g_tx_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Response
///////////////////////////////////////////////////////////////////////////////

static void test_slave_replies_to_handshake(void)
{
    TEST_BEGIN(test_slave_replies_to_handshake);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_ListenForHandshake;

    // Inject handshake from master 0x10 addressed to us (0xB1)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x10, 0xB1, 20, 0x01, 0x03, 0xAAAAAAAA);
    test_inject_packet(0, pkt, len);

    // Slave should have replied with a handshake
    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *reply;
    size_t reply_len;
    capture_get_packet(0, &reply, &reply_len);
    ASSERT_EQ_U(0x21, reply[1]); // Handshake
    ASSERT_EQ_U(0xB1, reply[3]); // src = us
    ASSERT_EQ_U(0x10, reply[4]); // dest = master
    TEST_END();
}

static void test_slave_ignores_handshake_for_other_device(void)
{
    TEST_BEGIN(test_slave_ignores_handshake_for_other_device);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_ListenForHandshake;

    // Inject handshake from 0x10 addressed to 0x40 (ESC, not us)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x10, 0x40, 20, 0x01, 0x00, 0xBBBBBBBB);
    test_inject_packet(0, pkt, len);

    // Slave should NOT have sent anything
    ASSERT_EQ(0, g_tx_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Baud Rate Negotiation
///////////////////////////////////////////////////////////////////////////////

static void test_slave_switches_baud_on_broadcast(void)
{
    TEST_BEGIN(test_slave_switches_baud_on_broadcast);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_ListenForHandshake;

    // Inject broadcast handshake (dest=0xFF) from master with baud=1 (400k)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x10, 0xFF, 20, 0x01, 0x03, 0xCCCCCCCC);
    test_inject_packet(0, pkt, len);

    ASSERT_EQ_U(400000, g_baud_rate);
    TEST_END();
}

static void test_slave_transitions_to_running_on_broadcast(void)
{
    TEST_BEGIN(test_slave_transitions_to_running_on_broadcast);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_ListenForHandshake;

    // Inject broadcast handshake
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x10, 0xFF, 20, 0x00, 0x03, 0xDDDDDDDD);
    test_inject_packet(0, pkt, len);

    ASSERT_EQ(SrxlState_Running, pBus->state);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Channel Data Reception
///////////////////////////////////////////////////////////////////////////////

static void test_slave_receives_channel_data(void)
{
    TEST_BEGIN(test_slave_receives_channel_data);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;

    uint16_t values[] = {32768, 16000};
    uint8_t pkt[80];
    uint8_t len = test_build_channel_data(pkt, 0x00, 0x00, 0x03, values, -50);
    test_inject_packet(0, pkt, len);

    ASSERT_EQ(32768, srxlChData.values[0]);
    ASSERT_EQ(16000, srxlChData.values[1]);
    ASSERT_EQ(1, g_cb_recv_channel_count);
    ASSERT_FALSE(g_cb_recv_channel_is_failsafe);
    TEST_END();
}

static void test_slave_sends_telemetry_when_polled(void)
{
    TEST_BEGIN(test_slave_sends_telemetry_when_polled);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;

    // Inject channel data with reply_id = our device ID (0xB1)
    uint16_t values[] = {32768};
    uint8_t pkt[80];
    uint8_t len = test_build_channel_data(pkt, 0x00, 0xB1, 0x01, values, -50);
    test_inject_packet(0, pkt, len);

    // Slave should send a telemetry reply
    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *reply;
    size_t reply_len;
    capture_get_packet(0, &reply, &reply_len);
    ASSERT_EQ_U(0x80, reply[1]); // Telemetry packet
    TEST_END();
}

static void test_slave_silent_when_not_polled(void)
{
    TEST_BEGIN(test_slave_silent_when_not_polled);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;

    // Inject channel data with reply_id = 0 (nobody should reply)
    uint16_t values[] = {32768};
    uint8_t pkt[80];
    uint8_t len = test_build_channel_data(pkt, 0x00, 0x00, 0x01, values, -50);
    test_inject_packet(0, pkt, len);

    // Slave should NOT send anything
    ASSERT_EQ(0, g_tx_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Timeout / Recovery
///////////////////////////////////////////////////////////////////////////////

static void test_slave_resets_on_50ms_timeout(void)
{
    TEST_BEGIN(test_slave_resets_on_50ms_timeout);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;
    g_baud_rate = 400000;
    pBus->baudRate = SRXL_BAUD_400000;

    // Advance 50ms with no packets
    test_advance_ms(0, 50);

    ASSERT_EQ_U(115200, g_baud_rate);
    ASSERT_EQ(SrxlState_ListenOnStartup, pBus->state);
    TEST_END();
}

static void test_slave_timeout_resets_master_receiver_info(void)
{
    TEST_BEGIN(test_slave_timeout_resets_master_receiver_info);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;

    // Set up a fake master receiver entry
    SrxlRcvrEntry masterRcvr = {0};
    masterRcvr.channelMask = 0xFF;
    masterRcvr.fades = 100;
    pBus->pMasterRcvr = &masterRcvr;

    test_advance_ms(0, 50);

    ASSERT_EQ_U(0, masterRcvr.channelMask);
    ASSERT_EQ_U(0xFFFF, masterRcvr.fades);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Failsafe Channel Data
///////////////////////////////////////////////////////////////////////////////

static void test_slave_receives_failsafe_data(void)
{
    TEST_BEGIN(test_slave_receives_failsafe_data);
    test_init_slave(0xB1);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;

    uint16_t values[] = {32768};
    uint8_t pkt[80];
    uint8_t len = test_build_channel_data(pkt, 0x01, 0x00, 0x01, values, -50);
    test_inject_packet(0, pkt, len);

    ASSERT_TRUE(srxlChDataIsFailsafe);
    ASSERT_EQ(1, g_cb_recv_channel_count);
    ASSERT_TRUE(g_cb_recv_channel_is_failsafe);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Slave State Machine Tests ===\n");

    // Startup
    RUN_TEST(test_slave_starts_in_listen_on_startup);
    RUN_TEST(test_slave_sends_unprompted_handshake_on_timeout);
    RUN_TEST(test_slave_with_nonzero_unit_listens);

    // Handshake Response
    RUN_TEST(test_slave_replies_to_handshake);
    RUN_TEST(test_slave_ignores_handshake_for_other_device);

    // Baud Rate Negotiation
    RUN_TEST(test_slave_switches_baud_on_broadcast);
    RUN_TEST(test_slave_transitions_to_running_on_broadcast);

    // Channel Data Reception
    RUN_TEST(test_slave_receives_channel_data);
    RUN_TEST(test_slave_sends_telemetry_when_polled);
    RUN_TEST(test_slave_silent_when_not_polled);

    // Timeout / Recovery
    RUN_TEST(test_slave_resets_on_50ms_timeout);
    RUN_TEST(test_slave_timeout_resets_master_receiver_info);

    // Failsafe
    RUN_TEST(test_slave_receives_failsafe_data);

    TEST_SUMMARY();
}
