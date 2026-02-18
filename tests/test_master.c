/*
 * Master State Machine Tests
 *
 * Tests the SRXL2 bus master implementation: startup, handshake,
 * channel data, telemetry polling, and failsafe.
 *
 * MIT License
 */

#include "test_harness.h"
#include "test_helpers.h"
#include "uart_adapter.h"

///////////////////////////////////////////////////////////////////////////////
// Startup & State Transitions
///////////////////////////////////////////////////////////////////////////////

static void test_master_starts_in_listen_on_startup(void)
{
    TEST_BEGIN(test_master_starts_in_listen_on_startup);
    test_init_master(0x10);
    ASSERT_EQ(SrxlState_ListenOnStartup, srxlBus[0].state);
    TEST_END();
}

static void test_master_transitions_to_handshake_after_50ms(void)
{
    TEST_BEGIN(test_master_transitions_to_handshake_after_50ms);
    test_init_master(0x10);
    ASSERT_EQ(SrxlState_ListenOnStartup, srxlBus[0].state);

    // Advance 49ms — should still be listening
    test_advance_ms(0, 49);
    ASSERT_EQ(SrxlState_ListenOnStartup, srxlBus[0].state);

    // Advance 1 more ms (total 50ms) — unit ID 0 should send handshake
    test_advance_ms(0, 1);
    ASSERT_EQ(SrxlState_SendHandshake, srxlBus[0].state);
    TEST_END();
}

static void test_master_stays_listening_if_unit_id_nonzero(void)
{
    TEST_BEGIN(test_master_stays_listening_if_unit_id_nonzero);
    test_init_master(0x11); // unit ID = 1
    test_advance_ms(0, 50);
    // With unit ID nonzero, should go to ListenForHandshake
    ASSERT_EQ(SrxlState_ListenForHandshake, srxlBus[0].state);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Sequence
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_first_handshake_to_self(void)
{
    TEST_BEGIN(test_master_sends_first_handshake_to_self);
    test_init_master(0x10);
    test_advance_ms(0, 50);

    // Master should now be in SendHandshake; run state machine to actually send
    test_advance_ms(0, 0);

    ASSERT_TRUE(g_tx_count > 0);

    const uint8_t *pkt;
    size_t pkt_len;
    ASSERT_EQ(0, capture_get_packet(0, &pkt, &pkt_len));
    ASSERT_EQ_U(0xA6, pkt[0]);
    ASSERT_EQ_U(0x21, pkt[1]); // Handshake
    // Source should be 0x10, dest should be 0x10 (self)
    ASSERT_EQ_U(0x10, pkt[3]); // srcDevID
    ASSERT_EQ_U(0x10, pkt[4]); // destDevID
    TEST_END();
}

static void test_master_scans_device_ids(void)
{
    TEST_BEGIN(test_master_scans_device_ids);
    test_init_master(0x10);
    test_advance_ms(0, 50);

    // Run the state machine many times to advance through handshake scanning
    // Each run should send a handshake to the next device ID
    for (int i = 0; i < 250; i++)
    {
        test_advance_ms(0, 0);
    }

    // We should have sent many handshakes
    ASSERT_TRUE(g_tx_count > 10);

    // Verify they're all handshake packets
    for (size_t i = 0; i < g_tx_count; i++)
    {
        const uint8_t *pkt;
        size_t pkt_len;
        ASSERT_EQ(0, capture_get_packet(i, &pkt, &pkt_len));
        ASSERT_EQ_U(0xA6, pkt[0]);
        ASSERT_EQ_U(0x21, pkt[1]); // All handshakes
    }
    TEST_END();
}

static void test_master_registers_responding_device(void)
{
    TEST_BEGIN(test_master_registers_responding_device);
    test_init_master(0x10);
    SrxlBus *pBus = &srxlBus[0];

    // Put bus into running state first with a simple handshake sequence
    pBus->state = SrxlState_Running;

    uint8_t before = pBus->rxDevCount;

    // Inject a handshake reply from ESC (0x40)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x40, 0x10, 30, 0x01, 0, 0xDEADBEEF);
    test_inject_packet(0, pkt, len);

    ASSERT_EQ(before + 1, pBus->rxDevCount);
    ASSERT_EQ_U(0x40, pBus->rxDev[before].deviceID);
    ASSERT_EQ(30, pBus->rxDev[before].priority);
    TEST_END();
}

static void test_master_tracks_baud_support(void)
{
    TEST_BEGIN(test_master_tracks_baud_support);
    test_init_master(0x10);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_SendHandshake;
    pBus->baudSupported = 0x01; // starts supporting 400k

    // Inject handshake from device that supports 400k
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x40, 0x10, 10, 0x01, 0, 0x11111111);
    test_inject_packet(0, pkt, len);
    ASSERT_EQ_U(0x01, pBus->baudSupported); // AND of 0x01 & 0x01

    // Inject handshake from device that does NOT support 400k
    len = test_build_handshake(pkt, 0x41, 0x10, 10, 0x00, 0, 0x22222222);
    test_inject_packet(0, pkt, len);
    ASSERT_EQ_U(0x00, pBus->baudSupported); // AND of 0x01 & 0x00
    TEST_END();
}

static void test_master_sends_broadcast_handshake(void)
{
    TEST_BEGIN(test_master_sends_broadcast_handshake);
    test_init_master(0x10);
    test_advance_ms(0, 50);

    // Run the master through all handshake steps
    for (int i = 0; i < 300; i++)
    {
        test_advance_ms(0, 0);
    }

    // Last handshake should have dest=0xFF (broadcast)
    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *last_pkt;
    size_t last_len;
    // Find the last handshake packet
    int last_hs = -1;
    for (size_t i = 0; i < g_tx_count; i++)
    {
        const uint8_t *p;
        size_t l;
        capture_get_packet(i, &p, &l);
        if (p[1] == 0x21)
            last_hs = (int)i;
    }
    ASSERT_TRUE(last_hs >= 0);
    capture_get_packet(last_hs, &last_pkt, &last_len);
    ASSERT_EQ_U(0xFF, last_pkt[4]); // dest = broadcast
    TEST_END();
}

static void test_master_transitions_to_running_after_broadcast(void)
{
    TEST_BEGIN(test_master_transitions_to_running_after_broadcast);
    test_init_master(0x10);
    test_advance_ms(0, 50);

    // Run through all handshake steps
    for (int i = 0; i < 300; i++)
    {
        test_advance_ms(0, 0);
    }

    ASSERT_EQ(SrxlState_Running, srxlBus[0].state);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Master Election
///////////////////////////////////////////////////////////////////////////////

static void test_master_yields_to_lower_id(void)
{
    TEST_BEGIN(test_master_yields_to_lower_id);
    test_init_master(0x30); // FC at 0x30
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_SendHandshake;

    // Receive handshake from 0x10 (lower ID, should become master)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x10, 0x30, 20, 0x01, 0x03, 0xAAAAAAAA);
    test_inject_packet(0, pkt, len);

    // 0x30 should yield
    ASSERT_FALSE(pBus->master);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Channel Data (Running State)
///////////////////////////////////////////////////////////////////////////////

// Helper: get master to running state with one device registered
static void setup_master_running_with_esc(void)
{
    test_init_master(0x10);
    SrxlBus *pBus = &srxlBus[0];

    // Skip handshake, go directly to running
    pBus->state = SrxlState_Running;
    pBus->pollOnceMore = false;

    // Register an ESC device
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x40, 0x10, 10, 0x00, 0, 0x11111111);
    test_inject_packet(0, pkt, len);

    // Reset capture so we only see new packets
    test_reset_capture();
    pBus->state = SrxlState_Running;
}

static void test_master_sends_channel_data_on_mask_set(void)
{
    TEST_BEGIN(test_master_sends_channel_data_on_mask_set);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    srxlChData.values[0] = 32768;
    pBus->channelOutMask = 0x01;
    pBus->timeoutCount_ms = 10; // Ensure timeout trigger

    test_advance_ms(0, 0);

    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *pkt;
    size_t pkt_len;
    capture_get_packet(0, &pkt, &pkt_len);
    ASSERT_EQ_U(0xA6, pkt[0]);
    ASSERT_EQ_U(0xCD, pkt[1]); // Control data
    ASSERT_EQ_U(0x00, pkt[3]); // cmd = CHANNEL
    TEST_END();
}

static void test_master_sends_channel_data_on_timeout(void)
{
    TEST_BEGIN(test_master_sends_channel_data_on_timeout);
    setup_master_running_with_esc();

    // Advance 10ms to trigger timeout-based send
    test_advance_ms(0, 10);

    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *pkt;
    size_t pkt_len;
    capture_get_packet(0, &pkt, &pkt_len);
    ASSERT_EQ_U(0xCD, pkt[1]); // Control data
    TEST_END();
}

static void test_master_channel_mask_cleared_after_send(void)
{
    TEST_BEGIN(test_master_channel_mask_cleared_after_send);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    pBus->channelOutMask = 0x0F;
    srxlChData.values[0] = 32768;
    srxlChData.values[1] = 16000;
    srxlChData.values[2] = 48000;
    srxlChData.values[3] = 32768;
    pBus->timeoutCount_ms = 10;

    test_advance_ms(0, 0);

    ASSERT_EQ_U(0x00, pBus->channelOutMask);
    TEST_END();
}

static void test_master_respects_response_wait_time(void)
{
    TEST_BEGIN(test_master_respects_response_wait_time);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    // Send initial channel data
    pBus->channelOutMask = 0x01;
    pBus->timeoutCount_ms = 10;
    test_advance_ms(0, 0);
    size_t initial_count = g_tx_count;
    ASSERT_TRUE(initial_count > 0);

    // After send, timeoutCount_ms was reset to 0.
    // Advance 1ms — should NOT send (wait time ~4ms at 115200)
    test_advance_ms(0, 1);
    ASSERT_EQ(initial_count, g_tx_count);

    // Advance 2 more ms (total 3ms) — still shouldn't send
    test_advance_ms(0, 2);
    ASSERT_EQ(initial_count, g_tx_count);

    // Advance 7 more ms (total 10ms) — NOW should send
    test_advance_ms(0, 7);
    ASSERT_TRUE(g_tx_count > initial_count);
    TEST_END();
}

static void test_master_multiple_channels(void)
{
    TEST_BEGIN(test_master_multiple_channels);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    pBus->channelOutMask = 0x0F;
    srxlChData.values[0] = 1000;
    srxlChData.values[1] = 2000;
    srxlChData.values[2] = 3000;
    srxlChData.values[3] = 4000;
    pBus->timeoutCount_ms = 10;

    test_advance_ms(0, 0);

    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *pkt;
    size_t pkt_len;
    capture_get_packet(0, &pkt, &pkt_len);
    ASSERT_EQ_U(0xCD, pkt[1]);

    // The lib computes: SRXL_CTRL_BASE_LENGTH + 7 + (2 * channelIndex)
    // SRXL_CTRL_BASE_LENGTH = 3+2+2 = 7, so 7 + 7 + 8 = 22
    ASSERT_EQ(22, pkt[2]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Polling
///////////////////////////////////////////////////////////////////////////////

static void test_master_reply_id_selects_device(void)
{
    TEST_BEGIN(test_master_reply_id_selects_device);
    setup_master_running_with_esc();

    // Advance 10ms to trigger channel data send
    test_advance_ms(0, 10);

    // The reply_id in the channel data should be the ESC's device ID
    ASSERT_EQ_U(0x40, g_hook_channel_sent_telem_id);
    TEST_END();
}

static void test_master_reply_id_rotates_by_priority(void)
{
    TEST_BEGIN(test_master_reply_id_rotates_by_priority);
    test_init_master(0x10);
    SrxlBus *pBus = &srxlBus[0];
    pBus->state = SrxlState_Running;
    pBus->pollOnceMore = false;

    // Register device A (0x40, priority=30) and B (0xB0, priority=10)
    uint8_t pkt[14];
    uint8_t len = test_build_handshake(pkt, 0x40, 0x10, 30, 0x00, 0, 0x11111111);
    test_inject_packet(0, pkt, len);
    len = test_build_handshake(pkt, 0xB0, 0x10, 10, 0x00, 0, 0x22222222);
    test_inject_packet(0, pkt, len);

    test_reset_capture();
    pBus->state = SrxlState_Running;

    int count_40 = 0;
    int count_b0 = 0;
    for (int i = 0; i < 40; i++)
    {
        // Advance 10ms to trigger channel data send each iteration
        test_advance_ms(0, 10);
        if (g_hook_channel_sent_telem_id == 0x40)
            count_40++;
        else if (g_hook_channel_sent_telem_id == 0xB0)
            count_b0++;
    }

    // A (priority 30) should be polled more than B (priority 10)
    ASSERT_TRUE(count_40 > count_b0);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Reception
///////////////////////////////////////////////////////////////////////////////

static void test_master_receives_telemetry(void)
{
    TEST_BEGIN(test_master_receives_telemetry);
    setup_master_running_with_esc();

    // Build telemetry packet with ESC payload
    uint8_t payload[16] = {0x20, 0x00, 0x01, 0x00, 0x10, 0x27, 0x00, 0xC8,
                           0x01, 0xF4, 0x00, 0x64, 0x0A, 0x14, 0x50, 0x64};
    uint8_t pkt[22];
    uint8_t len = test_build_telemetry(pkt, 0x10, payload);
    test_inject_packet(0, pkt, len);

    // srxlTelemData should have the raw payload
    ASSERT_MEM_EQ(payload, srxlTelemData.raw, 16);
    TEST_END();
}

static void test_master_rehandshake_on_telem_dest_ff(void)
{
    TEST_BEGIN(test_master_rehandshake_on_telem_dest_ff);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    // Inject telemetry with dest=0xFF
    uint8_t payload[16] = {0};
    uint8_t pkt[22];
    uint8_t len = test_build_telemetry(pkt, 0xFF, payload);
    test_inject_packet(0, pkt, len);

    // Master should transition to SendHandshake
    ASSERT_EQ(SrxlState_SendHandshake, pBus->state);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Failsafe
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_failsafe_channel_data(void)
{
    TEST_BEGIN(test_master_sends_failsafe_channel_data);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    srxlChDataIsFailsafe = true;
    srxlFailsafeChMask = 0x01;
    srxlChData.values[0] = 32768;
    pBus->timeoutCount_ms = 10;

    test_advance_ms(0, 0);

    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *pkt_data;
    size_t pkt_len;
    capture_get_packet(0, &pkt_data, &pkt_len);
    ASSERT_EQ_U(0xCD, pkt_data[1]);
    ASSERT_EQ_U(0x01, pkt_data[3]); // cmd = CHANNEL_FS
    ASSERT_EQ_U(0x00, pkt_data[4]); // reply_id = 0 for failsafe
    TEST_END();
}

static void test_master_failsafe_overrides_normal_trigger(void)
{
    TEST_BEGIN(test_master_failsafe_overrides_normal_trigger);
    setup_master_running_with_esc();
    SrxlBus *pBus = &srxlBus[0];

    // No mask set, timeout not reached, but failsafe is set
    srxlChDataIsFailsafe = true;
    srxlFailsafeChMask = 0x01;
    pBus->channelOutMask = 0;
    pBus->timeoutCount_ms = 5; // above response wait time

    test_advance_ms(0, 0);

    // Should still send because failsafe flag is checked
    ASSERT_TRUE(g_tx_count > 0);
    const uint8_t *pkt_data;
    size_t pkt_len;
    capture_get_packet(0, &pkt_data, &pkt_len);
    ASSERT_EQ_U(0x01, pkt_data[3]); // cmd = CHANNEL_FS
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Master State Machine Tests ===\n");

    // Startup
    RUN_TEST(test_master_starts_in_listen_on_startup);
    RUN_TEST(test_master_transitions_to_handshake_after_50ms);
    RUN_TEST(test_master_stays_listening_if_unit_id_nonzero);

    // Handshake
    RUN_TEST(test_master_sends_first_handshake_to_self);
    RUN_TEST(test_master_scans_device_ids);
    RUN_TEST(test_master_registers_responding_device);
    RUN_TEST(test_master_tracks_baud_support);
    RUN_TEST(test_master_sends_broadcast_handshake);
    RUN_TEST(test_master_transitions_to_running_after_broadcast);

    // Master Election
    RUN_TEST(test_master_yields_to_lower_id);

    // Channel Data
    RUN_TEST(test_master_sends_channel_data_on_mask_set);
    RUN_TEST(test_master_sends_channel_data_on_timeout);
    RUN_TEST(test_master_channel_mask_cleared_after_send);
    RUN_TEST(test_master_respects_response_wait_time);
    RUN_TEST(test_master_multiple_channels);

    // Telemetry Polling
    RUN_TEST(test_master_reply_id_selects_device);
    RUN_TEST(test_master_reply_id_rotates_by_priority);

    // Telemetry Reception
    RUN_TEST(test_master_receives_telemetry);
    RUN_TEST(test_master_rehandshake_on_telem_dest_ff);

    // Failsafe
    RUN_TEST(test_master_sends_failsafe_channel_data);
    RUN_TEST(test_master_failsafe_overrides_normal_trigger);

    TEST_SUMMARY();
}
