/*
 * Shared Test Scenario Definitions
 *
 * Included (not linked) by each scenario runner.  Uses test_harness.h
 * ASSERT macros and the g_harness pointer.  Uses srxl2_pkt_*() from
 * srxl2_packet.h to build test packets.
 *
 * MIT License
 */

/* This file is #include'd into a .c that already has:
 *   #include "test_harness.h"
 *   #include "scenario_harness.h"
 *   #include "srxl2_packet.h"
 *   scenario_harness_t *g_harness;
 */

///////////////////////////////////////////////////////////////////////////////
// Helper: get slave to RUNNING via broadcast handshake
///////////////////////////////////////////////////////////////////////////////

static void scenario_helper_slave_to_running(uint8_t device_id)
{
    H->init_slave(H, device_id);
    H->tick(H, 50);  /* past startup delay */

    /* Inject broadcast handshake from master 0x10 */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_115200, 0x03, 0xCCCCCCCC);
    H->inject(H, pkt, len);
    H->reset_capture(H);
}

///////////////////////////////////////////////////////////////////////////////
// Helper: get master to RUNNING with one ESC (0x40) registered
///////////////////////////////////////////////////////////////////////////////

static void scenario_helper_master_to_running(uint8_t device_id)
{
    H->init_master(H, device_id);
    H->tick(H, 50);  /* past startup delay */

    /* Advance a few ticks into the handshake scan */
    for (int i = 0; i < 5; i++)
        H->tick(H, 0);

    /* Inject handshake from ESC 0x40 *during* handshake scan
     * so it's registered before the broadcast/RUNNING transition */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x40, device_id, 30,
                                        SRXL2_BAUD_400000, 0, 0x11111111);
    H->inject(H, pkt, len);

    /* Run handshake to completion (broadcast → RUNNING) */
    complete_handshake(H, 500);
    H->reset_capture(H);
}

///////////////////////////////////////////////////////////////////////////////
// Slave scenarios
///////////////////////////////////////////////////////////////////////////////

static void scenario_slave_startup(void)
{
    TEST_BEGIN(scenario_slave_startup);
    H->init_slave(H, 0xB1);
    ASSERT_STR_EQ("STARTUP", H->state(H));
    TEST_END();
}

static void scenario_slave_startup_delay(void)
{
    TEST_BEGIN(scenario_slave_startup_delay);
    H->init_slave(H, 0xB1);

    H->tick(H, 49);
    ASSERT_STR_EQ("STARTUP", H->state(H));

    H->tick(H, 1);
    ASSERT_STR_EQ("HANDSHAKE", H->state(H));
    TEST_END();
}

static void scenario_slave_unprompted_handshake(void)
{
    TEST_BEGIN(scenario_slave_unprompted_handshake);
    H->init_slave(H, 0xB0); /* unit_id = 0 */

    H->tick(H, 50);
    ASSERT_STR_EQ("HANDSHAKE", H->state(H));
    ASSERT_TRUE(H->tx_count(H) > 0);

    uint8_t len;
    const uint8_t *pkt = H->tx_packet(H, 0, &len);
    ASSERT_NOT_NULL(pkt);
    ASSERT_EQ_U(0xA6, pkt[0]);
    ASSERT_EQ_U(0x21, pkt[1]); /* handshake */
    ASSERT_EQ_U(0xB0, pkt[3]); /* src = us */
    TEST_END();
}

static void scenario_slave_nonzero_listens(void)
{
    TEST_BEGIN(scenario_slave_nonzero_listens);
    H->init_slave(H, 0xB1); /* unit_id = 1 */

    H->tick(H, 50);
    ASSERT_STR_EQ("HANDSHAKE", H->state(H));
    ASSERT_EQ(0, (int)H->tx_count(H));
    TEST_END();
}

static void scenario_slave_replies_to_handshake(void)
{
    TEST_BEGIN(scenario_slave_replies_to_handshake);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);

    /* Inject handshake from master 0x10 addressed to us */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xB1, 20,
                                        SRXL2_BAUD_400000, 0x03, 0xAAAAAAAA);
    H->inject(H, pkt, len);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t rlen;
    const uint8_t *reply = H->tx_packet(H, H->tx_count(H) - 1, &rlen);
    ASSERT_NOT_NULL(reply);
    ASSERT_EQ_U(0x21, reply[1]); /* handshake */
    ASSERT_EQ_U(0xB1, reply[3]); /* src = us */
    ASSERT_EQ_U(0x10, reply[4]); /* dest = master */
    TEST_END();
}

static void scenario_slave_ignores_other_handshake(void)
{
    TEST_BEGIN(scenario_slave_ignores_other_handshake);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);
    size_t before = H->tx_count(H);

    /* Inject handshake for 0x40 (ESC, not us) */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0x40, 20,
                                        SRXL2_BAUD_400000, 0, 0xBBBBBBBB);
    H->inject(H, pkt, len);

    ASSERT_EQ((int)before, (int)H->tx_count(H));
    TEST_END();
}

static void scenario_slave_broadcast_to_running(void)
{
    TEST_BEGIN(scenario_slave_broadcast_to_running);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);

    /* Inject broadcast handshake */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_115200, 0x03, 0xDDDDDDDD);
    H->inject(H, pkt, len);

    ASSERT_TRUE(H->connected(H));
    ASSERT_STR_EQ("RUNNING", H->state(H));
    TEST_END();
}

static void scenario_slave_broadcast_baud_115200(void)
{
    TEST_BEGIN(scenario_slave_broadcast_baud_115200);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);

    /* Broadcast with baud=0 → stays 115200 */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_115200, 0x03, 0xCCCCCCCC);
    H->inject(H, pkt, len);

    ASSERT_EQ_U(115200, H->baud(H));
    TEST_END();
}

static void scenario_slave_broadcast_baud_400000(void)
{
    TEST_BEGIN(scenario_slave_broadcast_baud_400000);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);

    /* Broadcast with baud=1 → switch to 400000 */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_400000, 0x03, 0xCCCCCCCC);
    H->inject(H, pkt, len);

    ASSERT_EQ_U(400000, H->baud(H));
    TEST_END();
}

static void scenario_slave_receives_channel_data(void)
{
    TEST_BEGIN(scenario_slave_receives_channel_data);
    scenario_helper_slave_to_running(0xB1);

    uint16_t values[32] = {0};
    values[0] = 32768;
    values[1] = 16000;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0x00,
                                      -50, 0, 0x03, values);
    H->inject(H, pkt, len);

    uint16_t out[32] = {0};
    uint32_t mask = 0;
    bool fs = false;
    ASSERT_TRUE(H->get_channels(H, out, &mask, &fs));
    ASSERT_EQ(32768, out[0]);
    ASSERT_EQ(16000, out[1]);
    ASSERT_FALSE(fs);
    TEST_END();
}

static void scenario_slave_sends_telemetry_when_polled(void)
{
    TEST_BEGIN(scenario_slave_sends_telemetry_when_polled);
    scenario_helper_slave_to_running(0xB1);

    /* Set telemetry payload */
    uint8_t telem[16] = {0x20, 0x00, 1, 2, 3, 4, 5, 6,
                          7, 8, 9, 10, 11, 12, 13, 14};
    H->set_telemetry(H, telem);

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                      -50, 0, 0x01, values);
    H->inject(H, pkt, len);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t rlen;
    const uint8_t *reply = H->tx_packet(H, H->tx_count(H) - 1, &rlen);
    ASSERT_NOT_NULL(reply);
    ASSERT_EQ_U(0x80, reply[1]); /* telemetry */
    TEST_END();
}

static void scenario_slave_silent_when_not_polled(void)
{
    TEST_BEGIN(scenario_slave_silent_when_not_polled);
    scenario_helper_slave_to_running(0xB1);

    /* Inject channel data with reply_id = 0 */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0x00,
                                      -50, 0, 0x01, values);
    H->inject(H, pkt, len);

    ASSERT_EQ(0, (int)H->tx_count(H));
    TEST_END();
}

static void scenario_slave_timeout_resets(void)
{
    TEST_BEGIN(scenario_slave_timeout_resets);
    scenario_helper_slave_to_running(0xB1);

    /* 50ms silence → back to STARTUP */
    H->tick(H, 50);

    ASSERT_EQ_U(115200, H->baud(H));
    ASSERT_STR_EQ("STARTUP", H->state(H));
    TEST_END();
}

static void scenario_slave_receives_failsafe(void)
{
    TEST_BEGIN(scenario_slave_receives_failsafe);
    scenario_helper_slave_to_running(0xB1);

    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL_FS, 0x00,
                                      -80, 0, 0x01, values);
    H->inject(H, pkt, len);

    uint16_t out[32] = {0};
    uint32_t mask = 0;
    bool fs = false;
    ASSERT_TRUE(H->get_channels(H, out, &mask, &fs));
    ASSERT_TRUE(fs);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Master scenarios
///////////////////////////////////////////////////////////////////////////////

static void scenario_master_startup(void)
{
    TEST_BEGIN(scenario_master_startup);
    H->init_master(H, 0x10);
    ASSERT_STR_EQ("STARTUP", H->state(H));
    TEST_END();
}

static void scenario_master_startup_delay(void)
{
    TEST_BEGIN(scenario_master_startup_delay);
    H->init_master(H, 0x10);

    H->tick(H, 49);
    ASSERT_STR_EQ("STARTUP", H->state(H));

    H->tick(H, 1);
    ASSERT_STR_EQ("HANDSHAKE", H->state(H));
    TEST_END();
}

static void scenario_master_handshake_completes(void)
{
    TEST_BEGIN(scenario_master_handshake_completes);
    H->init_master(H, 0x10);
    H->tick(H, 50);

    ASSERT_TRUE(complete_handshake(H, 500));
    ASSERT_STR_EQ("RUNNING", H->state(H));
    TEST_END();
}

static void scenario_master_broadcast_sent(void)
{
    TEST_BEGIN(scenario_master_broadcast_sent);
    H->init_master(H, 0x10);
    H->tick(H, 50);
    complete_handshake(H, 500);

    /* The last handshake should be broadcast (dest=0xFF) */
    uint8_t len;
    const uint8_t *last = find_last_tx_of_type(H, 0x21, &len);
    ASSERT_NOT_NULL(last);
    ASSERT_EQ_U(0xFF, last[4]); /* dest = broadcast */
    TEST_END();
}

static void scenario_master_registers_peer(void)
{
    TEST_BEGIN(scenario_master_registers_peer);
    H->init_master(H, 0x10);
    H->tick(H, 50);
    complete_handshake(H, 500);

    uint8_t before = H->peer_count(H);

    /* Inject handshake from ESC */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x40, 0x10, 30,
                                        SRXL2_BAUD_400000, 0, 0xDEADBEEF);
    H->inject(H, pkt, len);

    ASSERT_EQ(before + 1, H->peer_count(H));
    TEST_END();
}

static void scenario_master_baud_negotiation_and(void)
{
    TEST_BEGIN(scenario_master_baud_negotiation_and);
    H->init_master(H, 0x10);
    H->tick(H, 50);

    /* Inject device supporting 400k */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x40, 0x10, 10,
                                        SRXL2_BAUD_400000, 0, 0x11111111);
    H->inject(H, pkt, len);

    /* Inject device NOT supporting 400k → AND should be 0 → 115200 */
    len = srxl2_pkt_handshake(pkt, 0xB0, 0x10, 10,
                                SRXL2_BAUD_115200, 0, 0x22222222);
    H->inject(H, pkt, len);

    /* Complete handshake so baud takes effect */
    complete_handshake(H, 500);

    ASSERT_EQ_U(115200, H->baud(H));
    TEST_END();
}

static void scenario_master_sends_channel_on_timeout(void)
{
    TEST_BEGIN(scenario_master_sends_channel_on_timeout);
    scenario_helper_master_to_running(0x10);

    /* Advance past frame period */
    H->tick(H, 11);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t len;
    const uint8_t *pkt = H->tx_packet(H, 0, &len);
    ASSERT_NOT_NULL(pkt);
    ASSERT_EQ_U(0xCD, pkt[1]); /* CONTROL packet */
    TEST_END();
}

static void scenario_master_channel_data_format(void)
{
    TEST_BEGIN(scenario_master_channel_data_format);
    scenario_helper_master_to_running(0x10);

    uint16_t values[32] = {0};
    values[0] = 1000; values[1] = 2000;
    values[2] = 3000; values[3] = 4000;
    H->set_channels(H, values, 0x0F);

    H->tick(H, 11);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t len;
    const uint8_t *pkt = H->tx_packet(H, 0, &len);
    ASSERT_NOT_NULL(pkt);
    ASSERT_EQ_U(0xCD, pkt[1]);
    /* 3(hdr) + 1(cmd) + 1(reply) + 1(rssi) + 2(losses) + 4(mask) + 8(4ch) + 2(crc) = 22 */
    ASSERT_EQ(22, pkt[2]);
    TEST_END();
}

static void scenario_master_sends_failsafe(void)
{
    TEST_BEGIN(scenario_master_sends_failsafe);
    scenario_helper_master_to_running(0x10);

    uint16_t values[32] = {0};
    values[0] = 32768;
    H->set_channels(H, values, 0x01);
    H->set_failsafe(H, true);

    H->tick(H, 11);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t len;
    const uint8_t *pkt = H->tx_packet(H, 0, &len);
    ASSERT_NOT_NULL(pkt);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL_FS, pkt[3]);
    ASSERT_EQ_U(0x00, pkt[4]); /* reply_id=0 for failsafe */
    TEST_END();
}

static void scenario_master_receives_telemetry(void)
{
    TEST_BEGIN(scenario_master_receives_telemetry);
    scenario_helper_master_to_running(0x10);

    /* Send a channel frame first to set up telem context */
    H->tick(H, 11);

    /* Inject telemetry with dest=0xFF → triggers rehandshake */
    uint8_t payload[16] = {0};
    uint8_t pkt[22];
    uint8_t len = srxl2_pkt_telemetry(pkt, 0xFF, payload);
    H->inject(H, pkt, len);

    ASSERT_STR_EQ("HANDSHAKE", H->state(H));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Wire format scenarios
///////////////////////////////////////////////////////////////////////////////

static void scenario_wire_handshake_reply_format(void)
{
    TEST_BEGIN(scenario_wire_handshake_reply_format);
    H->init_slave(H, 0xB1);
    H->tick(H, 50);

    /* Inject handshake from master to us */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xB1, 20,
                                        SRXL2_BAUD_400000, 0x03, 0xAAAAAAAA);
    H->inject(H, pkt, len);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t rlen;
    const uint8_t *reply = H->tx_packet(H, H->tx_count(H) - 1, &rlen);
    ASSERT_NOT_NULL(reply);

    /* Verify wire format */
    ASSERT_EQ_U(0xA6, reply[0]); /* magic */
    ASSERT_EQ_U(0x21, reply[1]); /* type = handshake */
    ASSERT_EQ(14, reply[2]);     /* length */
    ASSERT_EQ(14, rlen);         /* actual length */
    ASSERT_EQ_U(0xB1, reply[3]); /* src */
    ASSERT_EQ_U(0x10, reply[4]); /* dest */

    /* Verify CRC */
    uint16_t crc = srxl2_crc16(reply, rlen - 2);
    uint16_t wire_crc = ((uint16_t)reply[rlen - 2] << 8) | reply[rlen - 1];
    ASSERT_EQ_U(crc, wire_crc);
    TEST_END();
}

static void scenario_wire_telemetry_reply_format(void)
{
    TEST_BEGIN(scenario_wire_telemetry_reply_format);
    scenario_helper_slave_to_running(0xB1);

    /* Set telemetry payload */
    uint8_t telem[16] = {0x20, 0x00, 1, 2, 3, 4, 5, 6,
                          7, 8, 9, 10, 11, 12, 13, 14};
    H->set_telemetry(H, telem);

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                      -50, 0, 0x01, values);
    H->inject(H, pkt, len);

    ASSERT_TRUE(H->tx_count(H) > 0);
    uint8_t rlen;
    const uint8_t *reply = H->tx_packet(H, H->tx_count(H) - 1, &rlen);
    ASSERT_NOT_NULL(reply);

    /* Verify wire format */
    ASSERT_EQ_U(0xA6, reply[0]); /* magic */
    ASSERT_EQ_U(0x80, reply[1]); /* type = telemetry */
    ASSERT_EQ(22, reply[2]);     /* length */
    ASSERT_EQ(22, rlen);         /* actual length */

    /* Verify CRC */
    uint16_t crc = srxl2_crc16(reply, rlen - 2);
    uint16_t wire_crc = ((uint16_t)reply[rlen - 2] << 8) | reply[rlen - 1];
    ASSERT_EQ_U(crc, wire_crc);
    TEST_END();
}
