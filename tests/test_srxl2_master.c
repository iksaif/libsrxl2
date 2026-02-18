/*
 * Master State Machine Tests for libsrxl2
 *
 * Tests: startup, handshake, channel data, telemetry polling,
 * failsafe, bind/vtx/fwdpgm, master election.
 *
 * Uses a mock HAL that captures sent packets.
 *
 * MIT License
 */

#include "test_harness.h"
#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// Mock HAL
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

/* Event tracking */
static int evt_channel_count;
static int evt_telem_count;
static int evt_handshake_count;
static int evt_timeout_count;
static uint8_t evt_last_telem_device;

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
        break;
    case SRXL2_EVT_TELEMETRY:
        evt_telem_count++;
        evt_last_telem_device = evt->telemetry.device_id;
        break;
    case SRXL2_EVT_HANDSHAKE_COMPLETE:
        evt_handshake_count++;
        break;
    case SRXL2_EVT_TIMEOUT:
        evt_timeout_count++;
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
    evt_telem_count = 0;
    evt_handshake_count = 0;
    evt_timeout_count = 0;
    evt_last_telem_device = 0;
}

static uint8_t ctx_buf[4096];

static srxl2_ctx_t *create_master(uint8_t device_id)
{
    mock_reset();
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = {
            .device_id = device_id,
            .priority = 20,
            .info = SRXL2_DEVINFO_TELEM_TX_ENABLED | SRXL2_DEVINFO_TELEM_FULL_RANGE,
            .uid = 0x12345678,
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

/* Inject a raw packet into ctx via feed+tick */
static void inject_packet(srxl2_ctx_t *ctx, const uint8_t *pkt, uint8_t len)
{
    srxl2_feed(ctx, pkt, len);
    srxl2_tick(ctx);
}

/* Build handshake for injection */
static uint8_t build_hs(uint8_t *buf, uint8_t src, uint8_t dest,
                          uint8_t priority, uint8_t baud, uint8_t info,
                          uint32_t uid)
{
    return srxl2_pkt_handshake(buf, src, dest, priority, baud, info, uid);
}

/* Build telemetry for injection */
static uint8_t build_telem(uint8_t *buf, uint8_t dest_id,
                             const uint8_t payload[16])
{
    return srxl2_pkt_telemetry(buf, dest_id, payload);
}

/* Get master to RUNNING state with one ESC (0x40) registered */
static srxl2_ctx_t *master_running_with_esc(void)
{
    srxl2_ctx_t *ctx = create_master(0x10);

    /* Advance past startup */
    advance_ms(ctx, 50);

    /* Run through all handshake scans */
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    /* Inject handshake from ESC */
    uint8_t pkt[14];
    uint8_t len = build_hs(pkt, 0x40, 0x10, 30, SRXL2_BAUD_400000, 0, 0x11111111);
    inject_packet(ctx, pkt, len);

    /* Reset capture */
    mock_tx_len = 0;
    mock_pkt_count = 0;

    return ctx;
}

///////////////////////////////////////////////////////////////////////////////
// Startup & State Transitions
///////////////////////////////////////////////////////////////////////////////

static void test_master_starts_in_startup(void)
{
    TEST_BEGIN(test_master_starts_in_startup);
    srxl2_ctx_t *ctx = create_master(0x10);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));
    TEST_END();
}

static void test_master_transitions_to_handshake_after_50ms(void)
{
    TEST_BEGIN(test_master_transitions_to_handshake_after_50ms);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 49);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));

    advance_ms(ctx, 1);
    ASSERT_STR_EQ("HANDSHAKE", srxl2_get_state(ctx));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Handshake Sequence
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_handshake_packets(void)
{
    TEST_BEGIN(test_master_sends_handshake_packets);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50); /* enter HANDSHAKE */
    advance_ms(ctx, 0);  /* first handshake tick */
    /* Should have sent first handshake */
    ASSERT_TRUE(mock_pkt_count > 0);

    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_NOT_NULL(pkt);
    ASSERT_EQ_U(0xA6, pkt[0]);
    ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, pkt[1]);
    ASSERT_EQ_U(0x10, pkt[3]); /* src */
    TEST_END();
}

static void test_master_scans_default_ids(void)
{
    TEST_BEGIN(test_master_scans_default_ids);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50);

    /* Run through handshake scans */
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    /* Should have sent multiple handshakes */
    ASSERT_TRUE(mock_pkt_count >= 10);

    /* All should be handshake packets */
    for (size_t i = 0; i < mock_pkt_count; i++) {
        uint8_t len;
        const uint8_t *p = mock_get_packet(i, &len);
        ASSERT_EQ_U(SRXL2_PKT_HANDSHAKE, p[1]);
    }
    TEST_END();
}

static void test_master_registers_responding_device(void)
{
    TEST_BEGIN(test_master_registers_responding_device);
    srxl2_ctx_t *ctx = create_master(0x10);

    /* Get to running state */
    advance_ms(ctx, 50);
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    uint8_t before = srxl2_peer_count(ctx);

    /* Inject handshake from ESC */
    uint8_t pkt[14];
    uint8_t len = build_hs(pkt, 0x40, 0x10, 30, 0x01, 0, 0xDEADBEEF);
    inject_packet(ctx, pkt, len);

    ASSERT_EQ(before + 1, srxl2_peer_count(ctx));
    ASSERT_EQ_U(0x40, ctx->peers[before].device_id);
    ASSERT_EQ(30, ctx->peers[before].priority);
    TEST_END();
}

static void test_master_baud_negotiation(void)
{
    TEST_BEGIN(test_master_baud_negotiation);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50);

    /* During handshake, inject a device that supports 400k */
    uint8_t pkt[14];
    uint8_t len = build_hs(pkt, 0x40, 0x10, 10, SRXL2_BAUD_400000, 0, 0x11111111);
    inject_packet(ctx, pkt, len);
    ASSERT_EQ_U(SRXL2_BAUD_400000, ctx->hs_baud_and);

    /* Inject a device that only supports 115200 */
    len = build_hs(pkt, 0xB0, 0x10, 10, SRXL2_BAUD_115200, 0, 0x22222222);
    inject_packet(ctx, pkt, len);
    ASSERT_EQ_U(SRXL2_BAUD_115200, ctx->hs_baud_and);
    TEST_END();
}

static void test_master_sends_broadcast_handshake(void)
{
    TEST_BEGIN(test_master_sends_broadcast_handshake);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50);
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    /* Find the last handshake - should be broadcast (dest=0xFF) */
    int last_hs = -1;
    for (size_t i = 0; i < mock_pkt_count; i++) {
        uint8_t len;
        const uint8_t *p = mock_get_packet(i, &len);
        if (p[1] == SRXL2_PKT_HANDSHAKE)
            last_hs = (int)i;
    }
    ASSERT_TRUE(last_hs >= 0);

    uint8_t len;
    const uint8_t *last = mock_get_packet((size_t)last_hs, &len);
    ASSERT_EQ_U(0xFF, last[4]); /* dest = broadcast */
    TEST_END();
}

static void test_master_transitions_to_running(void)
{
    TEST_BEGIN(test_master_transitions_to_running);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50);
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    ASSERT_TRUE(srxl2_is_connected(ctx));
    ASSERT_STR_EQ("RUNNING", srxl2_get_state(ctx));
    TEST_END();
}

static void test_master_handshake_complete_event(void)
{
    TEST_BEGIN(test_master_handshake_complete_event);
    srxl2_ctx_t *ctx = create_master(0x10);

    advance_ms(ctx, 50);
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    ASSERT_TRUE(evt_handshake_count > 0);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Master Election
///////////////////////////////////////////////////////////////////////////////

static void test_master_yields_to_lower_id(void)
{
    TEST_BEGIN(test_master_yields_to_lower_id);
    srxl2_ctx_t *ctx = create_master(0x30); /* FC at 0x30 */

    advance_ms(ctx, 50); /* enter handshake */

    /* Receive handshake from 0x10 (lower ID) */
    uint8_t pkt[14];
    uint8_t len = build_hs(pkt, 0x10, 0x30, 20, 0x01, 0x03, 0xAAAAAAAA);
    inject_packet(ctx, pkt, len);

    ASSERT_FALSE(ctx->is_master);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Channel Data (Running State)
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_channel_data(void)
{
    TEST_BEGIN(test_master_sends_channel_data);
    srxl2_ctx_t *ctx = master_running_with_esc();

    uint16_t values[32] = {0};
    values[0] = 32768;
    srxl2_set_channels(ctx, values, 0x01);

    /* Advance past response wait */
    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt[1]);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL, pkt[3]);
    TEST_END();
}

static void test_master_sends_on_timeout(void)
{
    TEST_BEGIN(test_master_sends_on_timeout);
    srxl2_ctx_t *ctx = master_running_with_esc();

    /* Advance enough for frame timeout */
    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt[1]);
    TEST_END();
}

static void test_master_channel_mask_cleared_after_send(void)
{
    TEST_BEGIN(test_master_channel_mask_cleared_after_send);
    srxl2_ctx_t *ctx = master_running_with_esc();

    uint16_t values[32] = {0};
    values[0] = 32768;
    srxl2_set_channels(ctx, values, 0x0F);

    advance_ms(ctx, 11);

    ASSERT_EQ_U(0x00, ctx->chan_out.mask);
    TEST_END();
}

static void test_master_respects_response_wait(void)
{
    TEST_BEGIN(test_master_respects_response_wait);
    srxl2_ctx_t *ctx = master_running_with_esc();

    /* Send first frame */
    advance_ms(ctx, 11);
    size_t first_count = mock_pkt_count;
    ASSERT_TRUE(first_count > 0);

    /* 1ms later: should NOT send (response wait) */
    advance_ms(ctx, 1);
    ASSERT_EQ(first_count, mock_pkt_count);

    /* After enough time: should send */
    advance_ms(ctx, 10);
    ASSERT_TRUE(mock_pkt_count > first_count);
    TEST_END();
}

static void test_master_multiple_channels(void)
{
    TEST_BEGIN(test_master_multiple_channels);
    srxl2_ctx_t *ctx = master_running_with_esc();

    uint16_t values[32] = {0};
    values[0] = 1000; values[1] = 2000;
    values[2] = 3000; values[3] = 4000;
    srxl2_set_channels(ctx, values, 0x0F);

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    /* header(3) + cmd(1) + reply(1) + rssi(1) + losses(2) + mask(4) + 4ch(8) + crc(2) = 22 */
    ASSERT_EQ(22, pkt[2]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Polling
///////////////////////////////////////////////////////////////////////////////

static void test_master_reply_id_selects_device(void)
{
    TEST_BEGIN(test_master_reply_id_selects_device);
    srxl2_ctx_t *ctx = master_running_with_esc();

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    /* reply_id at offset 4 should be ESC device ID */
    ASSERT_EQ_U(0x40, pkt[4]);
    TEST_END();
}

static void test_master_reply_id_rotates_by_priority(void)
{
    TEST_BEGIN(test_master_reply_id_rotates_by_priority);
    srxl2_ctx_t *ctx = create_master(0x10);

    /* Get to running */
    advance_ms(ctx, 50);
    for (int i = 0; i < 15; i++)
        advance_ms(ctx, 0);

    /* Register ESC (priority=30) and Sensor (priority=10) */
    uint8_t pkt[14];
    uint8_t len = build_hs(pkt, 0x40, 0x10, 30, 0x00, 0, 0x11111111);
    inject_packet(ctx, pkt, len);
    len = build_hs(pkt, 0xB0, 0x10, 10, 0x00, 0, 0x22222222);
    inject_packet(ctx, pkt, len);

    mock_tx_len = 0;
    mock_pkt_count = 0;

    int count_40 = 0, count_b0 = 0;
    for (int i = 0; i < 40; i++) {
        advance_ms(ctx, 11);
        if (mock_pkt_count > 0) {
            uint8_t plen;
            const uint8_t *p = mock_get_packet(mock_pkt_count - 1, &plen);
            if (p[1] == SRXL2_PKT_CONTROL) {
                if (p[4] == 0x40) count_40++;
                else if (p[4] == 0xB0) count_b0++;
            }
        }
    }

    /* ESC (priority 30) should be polled more often */
    ASSERT_TRUE(count_40 > count_b0);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Reception
///////////////////////////////////////////////////////////////////////////////

static void test_master_receives_telemetry(void)
{
    TEST_BEGIN(test_master_receives_telemetry);
    srxl2_ctx_t *ctx = master_running_with_esc();

    /* Send a channel data frame to set telem_poll_idx */
    advance_ms(ctx, 11);

    /* Inject telemetry from ESC */
    uint8_t payload[16] = {0x20, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                           0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    uint8_t pkt[22];
    uint8_t len = build_telem(pkt, 0x10, payload);
    inject_packet(ctx, pkt, len);

    /* Should be able to retrieve it */
    uint8_t out[16];
    uint32_t age;
    ASSERT_TRUE(srxl2_get_telemetry(ctx, 0x40, out, &age));
    ASSERT_MEM_EQ(payload, out, 16);
    TEST_END();
}

static void test_master_telem_event(void)
{
    TEST_BEGIN(test_master_telem_event);
    srxl2_ctx_t *ctx = master_running_with_esc();

    advance_ms(ctx, 11);

    uint8_t payload[16] = {0};
    uint8_t pkt[22];
    uint8_t len = build_telem(pkt, 0x10, payload);
    inject_packet(ctx, pkt, len);

    ASSERT_TRUE(evt_telem_count > 0);
    ASSERT_EQ_U(0x40, evt_last_telem_device);
    TEST_END();
}

static void test_master_rehandshake_on_telem_dest_ff(void)
{
    TEST_BEGIN(test_master_rehandshake_on_telem_dest_ff);
    srxl2_ctx_t *ctx = master_running_with_esc();

    advance_ms(ctx, 11);

    /* Inject telemetry with dest=0xFF */
    uint8_t payload[16] = {0};
    uint8_t pkt[22];
    uint8_t len = build_telem(pkt, 0xFF, payload);
    inject_packet(ctx, pkt, len);

    ASSERT_STR_EQ("HANDSHAKE", srxl2_get_state(ctx));
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Failsafe
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_failsafe(void)
{
    TEST_BEGIN(test_master_sends_failsafe);
    srxl2_ctx_t *ctx = master_running_with_esc();

    uint16_t values[32] = {0};
    values[0] = 32768;
    srxl2_set_channels(ctx, values, 0x01);
    srxl2_set_failsafe(ctx, true);

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL_FS, pkt[3]);
    ASSERT_EQ_U(0x00, pkt[4]); /* reply_id=0 for failsafe */
    TEST_END();
}

static void test_master_failsafe_sends_even_without_mask(void)
{
    TEST_BEGIN(test_master_failsafe_sends_even_without_mask);
    srxl2_ctx_t *ctx = master_running_with_esc();

    srxl2_set_failsafe(ctx, true);

    advance_ms(ctx, 11);

    /* Should still send because failsafe flag is set */
    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_CMD_CHANNEL_FS, pkt[3]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Bind / VTX / Forward Programming TX Flags
///////////////////////////////////////////////////////////////////////////////

static void test_master_sends_bind(void)
{
    TEST_BEGIN(test_master_sends_bind);
    srxl2_ctx_t *ctx = master_running_with_esc();

    srxl2_enter_bind(ctx, 0xB2, false);

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_BIND, pkt[1]);
    TEST_END();
}

static void test_master_sends_vtx(void)
{
    TEST_BEGIN(test_master_sends_vtx);
    srxl2_ctx_t *ctx = master_running_with_esc();

    srxl2_vtx_data_t vtx = { .band = 1, .channel = 5 };
    srxl2_set_vtx(ctx, &vtx);

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt[1]);
    ASSERT_EQ_U(SRXL2_CMD_VTX, pkt[3]);
    TEST_END();
}

static void test_master_sends_fwd_pgm(void)
{
    TEST_BEGIN(test_master_sends_fwd_pgm);
    srxl2_ctx_t *ctx = master_running_with_esc();

    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};
    srxl2_send_fwd_pgm(ctx, 0x40, data, 4);

    advance_ms(ctx, 11);

    ASSERT_TRUE(mock_pkt_count > 0);
    uint8_t len;
    const uint8_t *pkt = mock_get_packet(0, &len);
    ASSERT_EQ_U(SRXL2_PKT_CONTROL, pkt[1]);
    ASSERT_EQ_U(SRXL2_CMD_FWDPGM, pkt[3]);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Static vs Dynamic Allocation
///////////////////////////////////////////////////////////////////////////////

static void test_static_init(void)
{
    TEST_BEGIN(test_static_init);
    uint8_t buf[4096];
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = { .device_id = 0x10, .priority = 20, .uid = 1 },
        .hal = { .uart_send = mock_uart_send,
                 .uart_set_baud = mock_uart_set_baud,
                 .time_ms = mock_time_ms },
    };
    srxl2_ctx_t *ctx = srxl2_init_static(buf, sizeof(buf), &cfg);
    ASSERT_NOT_NULL(ctx);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));

    /* Too small buffer should fail */
    srxl2_ctx_t *ctx2 = srxl2_init_static(buf, 10, &cfg);
    ASSERT_NULL(ctx2);
    TEST_END();
}

static void test_dynamic_init(void)
{
    TEST_BEGIN(test_dynamic_init);
    mock_reset();
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = { .device_id = 0x10, .priority = 20, .uid = 1 },
        .hal = { .uart_send = mock_uart_send,
                 .uart_set_baud = mock_uart_set_baud,
                 .time_ms = mock_time_ms },
    };
    srxl2_ctx_t *ctx = srxl2_init(&cfg);
    ASSERT_NOT_NULL(ctx);
    ASSERT_STR_EQ("STARTUP", srxl2_get_state(ctx));
    srxl2_destroy(ctx);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== libsrxl2 Master Tests ===\n");

    /* Startup */
    RUN_TEST(test_master_starts_in_startup);
    RUN_TEST(test_master_transitions_to_handshake_after_50ms);

    /* Handshake */
    RUN_TEST(test_master_sends_handshake_packets);
    RUN_TEST(test_master_scans_default_ids);
    RUN_TEST(test_master_registers_responding_device);
    RUN_TEST(test_master_baud_negotiation);
    RUN_TEST(test_master_sends_broadcast_handshake);
    RUN_TEST(test_master_transitions_to_running);
    RUN_TEST(test_master_handshake_complete_event);

    /* Master Election */
    RUN_TEST(test_master_yields_to_lower_id);

    /* Channel Data */
    RUN_TEST(test_master_sends_channel_data);
    RUN_TEST(test_master_sends_on_timeout);
    RUN_TEST(test_master_channel_mask_cleared_after_send);
    RUN_TEST(test_master_respects_response_wait);
    RUN_TEST(test_master_multiple_channels);

    /* Telemetry Polling */
    RUN_TEST(test_master_reply_id_selects_device);
    RUN_TEST(test_master_reply_id_rotates_by_priority);

    /* Telemetry Reception */
    RUN_TEST(test_master_receives_telemetry);
    RUN_TEST(test_master_telem_event);
    RUN_TEST(test_master_rehandshake_on_telem_dest_ff);

    /* Failsafe */
    RUN_TEST(test_master_sends_failsafe);
    RUN_TEST(test_master_failsafe_sends_even_without_mask);

    /* Bind / VTX / Forward Programming */
    RUN_TEST(test_master_sends_bind);
    RUN_TEST(test_master_sends_vtx);
    RUN_TEST(test_master_sends_fwd_pgm);

    /* Init */
    RUN_TEST(test_static_init);
    RUN_TEST(test_dynamic_init);

    TEST_SUMMARY();
}
