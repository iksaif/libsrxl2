/*
 * Slave State Machine Tests for libsrxl2
 *
 * Tests: startup, handshake response, channel data reception,
 * telemetry replies, timeout/recovery, failsafe.
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// Mock HAL (same pattern as master tests)
///////////////////////////////////////////////////////////////////////////////

#define MOCK_TX_BUF_SIZE 2048
#define MOCK_MAX_PACKETS 64

static uint8_t mock_tx_buf[MOCK_TX_BUF_SIZE];
static size_t mock_tx_len;

typedef struct {
    size_t offset;
    uint8_t len;
} mock_pkt_entry_t;

static mock_pkt_entry_t mock_packets[MOCK_MAX_PACKETS];
static size_t mock_pkt_count;
static uint32_t mock_baud;
static uint32_t mock_time;

static int evt_channel_count;
static bool evt_last_failsafe;
static int evt_timeout_count;
static int evt_handshake_count;
static int evt_telem_request_count;

static void mock_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    if (mock_tx_len + len <= MOCK_TX_BUF_SIZE &&
        mock_pkt_count < MOCK_MAX_PACKETS) {
        mock_packets[mock_pkt_count].offset = mock_tx_len;
        mock_packets[mock_pkt_count].len = len;
        mock_pkt_count++;
        memcpy(&mock_tx_buf[mock_tx_len], buf, len);
        mock_tx_len += len;
    }
}

static void mock_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    mock_baud = baud;
}

static uint32_t mock_time_ms(void *user)
{
    (void)user;
    return mock_time;
}

static const uint8_t *mock_get_packet(size_t idx, uint8_t *len_out)
{
    if (idx >= mock_pkt_count) return NULL;
    *len_out = mock_packets[idx].len;
    return &mock_tx_buf[mock_packets[idx].offset];
}

static void mock_event_cb(srxl2_ctx_t *ctx, const srxl2_event_t *evt,
                           void *user)
{
    (void)ctx;
    (void)user;
    switch (evt->type) {
    case SRXL2_EVT_CHANNEL:
        evt_channel_count++;
        evt_last_failsafe = evt->channel.data->is_failsafe;
        break;
    case SRXL2_EVT_TIMEOUT:
        evt_timeout_count++;
        break;
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        evt_handshake_count++;
        break;
    case SRXL2_EVT_TELEM_REQUEST:
        evt_telem_request_count++;
        break;
    default:
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Test Helpers
///////////////////////////////////////////////////////////////////////////////

static void mock_reset(void)
{
    mock_tx_len = 0;
    mock_pkt_count = 0;
    mock_baud = 115200;
    mock_time = 0;
    evt_channel_count = 0;
    evt_last_failsafe = false;
    evt_timeout_count = 0;
    evt_handshake_count = 0;
    evt_telem_request_count = 0;
}

static uint8_t ctx_buf[4096];

static srxl2_ctx_t *create_slave(uint8_t device_id)
{
    mock_reset();
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_SLAVE,
        .device = {
            .device_id = device_id,
            .priority = 10,
            .info = SRXL2_DEVINFO_NO_RF,
            .uid = 0xAABBCCDD,
        },
        .hal = {
            .uart_send = mock_uart_send,
            .uart_set_baud = mock_uart_set_baud,
            .time_ms = mock_time_ms,
            .user = NULL,
        },
        .baud_supported = SRXL2_BAUD_400000,
    };
    srxl2_ctx_t *ctx = srxl2_init_static(ctx_buf, sizeof(ctx_buf), &cfg);
    srxl2_on_event(ctx, mock_event_cb, NULL);
    return ctx;
}

static void advance_ms(srxl2_ctx_t *ctx, uint32_t ms)
{
    mock_time += ms;
    srxl2_tick(ctx);
}

static void inject_packet(srxl2_ctx_t *ctx, const uint8_t *pkt, uint8_t len)
{
    srxl2_feed(ctx, pkt, len);
    srxl2_tick(ctx);
}

/* Get slave to RUNNING state with a successful handshake */
static srxl2_ctx_t *slave_running(uint8_t device_id)
{
    srxl2_ctx_t *ctx = create_slave(device_id);

    /* Advance past startup */
    advance_ms(ctx, 50);

    /* Inject broadcast handshake from master 0x10 */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_115200, 0x03, 0xCCCCCCCC);
    inject_packet(ctx, pkt, len);

    /* Reset capture */
    mock_tx_len = 0;
    mock_pkt_count = 0;

    return ctx;
}

///////////////////////////////////////////////////////////////////////////////
// Startup
///////////////////////////////////////////////////////////////////////////////

static void test_slave_starts_in_startup(void)
{
    TEST_BEGIN(test_slave_starts_in_startup);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));
    TEST_END();
}

static void test_slave_sends_unprompted_handshake(void)
{
    TEST_BEGIN(test_slave_sends_unprompted_handshake);
    srxl2_ctx_t *ctx = create_slave(0xB0); /* unit_id = 0 */

    advance_ms(ctx, 50);

    ASSERT_STR_EQ("HANDSHAKE", srxl2_get_state(ctx));
    ASSERT_TRUE(mock_pkt_count > 0);

    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, pkt[1]);
    ASSERT_EQ_U(0xB0, pkt[3]); /* src = us */
    TEST_END();
}

static void test_slave_nonzero_unit_listens(void)
{
    TEST_BEGIN(test_slave_nonzero_unit_listens);
    srxl2_ctx_t *ctx = create_slave(0xB1); /* unit_id = 1 */

    advance_ms(ctx, 50);

    ASSERT_STR_EQ("HANDSHAKE", srxl2_get_state(ctx));
    /* No packet should have been sent */
    ASSERT_EQ(0, mock_pkt_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Response
///////////////////////////////////////////////////////////////////////////////

static void test_slave_replies_to_handshake(void)
{
    TEST_BEGIN(test_slave_replies_to_handshake);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    advance_ms(ctx, 50);

    /* Inject handshake from master 0x10 to us */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xB1, 20,
                                        SRXL2_BAUD_400000, 0x03, 0xAAAAAAAA);
    inject_packet(ctx, pkt, len);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t rlen;
    const uint8_t *reply = mock_get_packet(mock_pkt_count - 1, &rlen);
    ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, reply[1]);
    ASSERT_EQ_U(0xB1, reply[3]); /* src = us */
    ASSERT_EQ_U(0x10, reply[4]); /* dest = master */
    TEST_END();
}

static void test_slave_ignores_handshake_for_other(void)
{
    TEST_BEGIN(test_slave_ignores_handshake_for_other);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    advance_ms(ctx, 50);
    size_t before = mock_pkt_count;

    /* Inject handshake for 0x40 (ESC, not us) */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0x40, 20,
                                        SRXL2_BAUD_400000, 0, 0xBBBBBBBB);
    inject_packet(ctx, pkt, len);

    ASSERT_EQ(before, mock_pkt_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Baud Rate Negotiation
///////////////////////////////////////////////////////////////////////////////

static void test_slave_switches_baud_on_broadcast(void)
{
    TEST_BEGIN(test_slave_switches_baud_on_broadcast);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    advance_ms(ctx, 50);

    /* Inject broadcast with baud=400k */
    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_400000, 0x03, 0xCCCCCCCC);
    inject_packet(ctx, pkt, len);

    ASSERT_EQ_U(400000, mock_baud);
    TEST_END();
}

static void test_slave_transitions_to_running_on_broadcast(void)
{
    TEST_BEGIN(test_slave_transitions_to_running_on_broadcast);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    advance_ms(ctx, 50);

    uint8_t pkt[14];
    uint8_t len = srxl2_pkt_handshake(pkt, 0x10, 0xFF, 20,
                                        SRXL2_BAUD_115200, 0x03, 0xDDDDDDDD);
    inject_packet(ctx, pkt, len);

    ASSERT_TRUE(srxl2_is_connected(ctx));
    ASSERT_STR_EQ("RUNNING", srxl2_get_state(ctx));
    ASSERT_TRUE(evt_handshake_count > 0);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Channel Data Reception
///////////////////////////////////////////////////////////////////////////////

static void test_slave_receives_channel_data(void)
{
    TEST_BEGIN(test_slave_receives_channel_data);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    uint16_t values[32] = {0};
    values[0] = 32768;
    values[1] = 16000;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0x00,
                                     -50, 0, 0x03, values);
    inject_packet(ctx, pkt, len);

    srxl2_channel_data_t data;
    ASSERT_TRUE(srxl2_get_channels(ctx, &data));
    ASSERT_EQ(32768, data.values[0]);
    ASSERT_EQ(16000, data.values[1]);
    ASSERT_FALSE(data.is_failsafe);
    ASSERT_EQ(1, evt_channel_count);
    TEST_END();
}

static void test_slave_sends_telemetry_when_polled(void)
{
    TEST_BEGIN(test_slave_sends_telemetry_when_polled);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Set telemetry payload */
    uint8_t telem[16] = {0x20, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    srxl2_set_telemetry(ctx, telem);

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                     -50, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t rlen;
    const uint8_t *reply = mock_get_packet(mock_pkt_count - 1, &rlen);
    ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, reply[1]);
    TEST_END();
}

static void test_slave_silent_when_not_polled(void)
{
    TEST_BEGIN(test_slave_silent_when_not_polled);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Inject channel data with reply_id = 0 */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0x00,
                                     -50, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    ASSERT_EQ(0, mock_pkt_count);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Timeout / Recovery
///////////////////////////////////////////////////////////////////////////////

static void test_slave_resets_on_timeout(void)
{
    TEST_BEGIN(test_slave_resets_on_timeout);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Set baud to 400k to verify reset */
    ctx->negotiated_baud = SRXL2_BAUD_400000;
    mock_baud = 400000;

    /* Advance 50ms with no packets */
    advance_ms(ctx, 50);

    ASSERT_EQ_U(115200, mock_baud);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));
    ASSERT_TRUE(evt_timeout_count > 0);
    TEST_END();
}

static void test_slave_timeout_fires_event(void)
{
    TEST_BEGIN(test_slave_timeout_fires_event);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    advance_ms(ctx, 50);

    ASSERT_TRUE(evt_timeout_count > 0);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Failsafe Channel Data
///////////////////////////////////////////////////////////////////////////////

static void test_slave_receives_failsafe_data(void)
{
    TEST_BEGIN(test_slave_receives_failsafe_data);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL_FS, 0x00,
                                     -80, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    srxl2_channel_data_t data;
    ASSERT_TRUE(srxl2_get_channels(ctx, &data));
    ASSERT_TRUE(data.is_failsafe);
    ASSERT_TRUE(evt_last_failsafe);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Timeout
///////////////////////////////////////////////////////////////////////////////

static void test_slave_handshake_timeout_resets(void)
{
    TEST_BEGIN(test_slave_handshake_timeout_resets);
    srxl2_ctx_t *ctx = create_slave(0xB1);
    advance_ms(ctx, 50); /* enter handshake */
    ASSERT_STR_EQ("HANDSHAKE", srxl2_get_state(ctx));

    /* Wait 200ms without receiving anything */
    advance_ms(ctx, 200);

    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// TELEM_REQUEST Event
///////////////////////////////////////////////////////////////////////////////

static void test_slave_telem_request_fires_when_polled(void)
{
    TEST_BEGIN(test_slave_telem_request_fires_when_polled);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Set some telemetry */
    uint8_t telem[16] = {0x34, 0x00};
    srxl2_set_telemetry(ctx, telem);

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                     -50, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    /* TELEM_REQUEST should have fired */
    ASSERT_TRUE(evt_telem_request_count > 0);
    TEST_END();
}

static void telem_request_cb(srxl2_ctx_t *ctx, const srxl2_event_t *evt,
                               void *user)
{
    (void)user;
    mock_event_cb(ctx, evt, user);

    if (evt->type == SRXL2_EVT_TELEM_REQUEST) {
        /* Fill telemetry in the callback (pull model) */
        uint8_t payload[16] = {0x7E, 0x00, 0x01, 0x02, 0x03, 0x04,
                               0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
                               0x0B, 0x0C, 0x0D, 0x0E};
        srxl2_set_telemetry(ctx, payload);
    }
}

static void test_slave_telem_set_in_callback_is_sent(void)
{
    TEST_BEGIN(test_slave_telem_set_in_callback_is_sent);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Override callback to use pull model */
    srxl2_on_event(ctx, telem_request_cb, NULL);
    evt_telem_request_count = 0;

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                     -50, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    /* Should have sent a telemetry reply */
    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t rlen;
    const uint8_t *reply = mock_get_packet(mock_pkt_count - 1, &rlen);
    ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, reply[1]);

    /* The payload in the telemetry packet should be what we set in callback.
       Telemetry packet: 0xA6, type(0x80), len, dest_id, payload[16], crc[2]
       payload starts at offset 4. */
    ASSERT_EQ_U(0x7E, reply[4]); /* sensor ID we set */
    TEST_END();
}

static void test_slave_sends_zeros_if_never_set(void)
{
    TEST_BEGIN(test_slave_sends_zeros_if_never_set);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Do NOT call srxl2_set_telemetry - payload should be all zeros */

    /* Inject channel data with reply_id = us */
    uint16_t values[32] = {0};
    values[0] = 32768;
    uint8_t pkt[80];
    uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                     -50, 0, 0x01, values);
    inject_packet(ctx, pkt, len);

    /* Should still send a reply (bus timing requires it) */
    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t rlen;
    const uint8_t *reply = mock_get_packet(mock_pkt_count - 1, &rlen);
    ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, reply[1]);

    /* Payload should be all zeros (offset 4) */
    ASSERT_EQ_U(0x00, reply[4]);
    ASSERT_EQ_U(0x00, reply[5]);
    TEST_END();
}

static void test_slave_telem_latches_across_polls(void)
{
    TEST_BEGIN(test_slave_telem_latches_across_polls);
    srxl2_ctx_t *ctx = slave_running(0xB1);

    /* Set telemetry once */
    uint8_t telem[16] = {0x34, 0x01, 0xAA, 0xBB};
    srxl2_set_telemetry(ctx, telem);

    /* Poll twice - payload should be the same both times */
    for (int poll = 0; poll < 2; poll++) {
        mock_tx_len = 0;
        mock_pkt_count = 0;

        uint16_t values[32] = {0};
        values[0] = 32768;
        uint8_t pkt[80];
        uint8_t len = srxl2_pkt_channel(pkt, SRXL2_CMD_CHANNEL, 0xB1,
                                         -50, 0, 0x01, values);
        /* Keep connection alive */
        mock_time += 5;
        inject_packet(ctx, pkt, len);

        ASSERT_TRUE(mock_pkt_count > 0);
        uint8_t rlen;
        const uint8_t *reply = mock_get_packet(mock_pkt_count - 1, &rlen);
        ASSERT_EQ_U(SRXL2_PKT_TELEMETRY, reply[1]);
        ASSERT_EQ_U(0x34, reply[4]);
        ASSERT_EQ_U(0x01, reply[5]);
    }
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== libsrxl2 Slave Tests ===\n");

    /* Startup */
    RUN_TEST(test_slave_starts_in_startup);
    RUN_TEST(test_slave_sends_unprompted_handshake);
    RUN_TEST(test_slave_nonzero_unit_listens);

    /* Handshake Response */
    RUN_TEST(test_slave_replies_to_handshake);
    RUN_TEST(test_slave_ignores_handshake_for_other);

    /* Baud Rate Negotiation */
    RUN_TEST(test_slave_switches_baud_on_broadcast);
    RUN_TEST(test_slave_transitions_to_running_on_broadcast);

    /* Channel Data */
    RUN_TEST(test_slave_receives_channel_data);
    RUN_TEST(test_slave_sends_telemetry_when_polled);
    RUN_TEST(test_slave_silent_when_not_polled);

    /* Timeout */
    RUN_TEST(test_slave_resets_on_timeout);
    RUN_TEST(test_slave_timeout_fires_event);

    /* Failsafe */
    RUN_TEST(test_slave_receives_failsafe_data);

    /* Handshake Timeout */
    RUN_TEST(test_slave_handshake_timeout_resets);

    /* TELEM_REQUEST */
    RUN_TEST(test_slave_telem_request_fires_when_polled);
    RUN_TEST(test_slave_telem_set_in_callback_is_sent);
    RUN_TEST(test_slave_sends_zeros_if_never_set);
    RUN_TEST(test_slave_telem_latches_across_polls);

    TEST_SUMMARY();
}
