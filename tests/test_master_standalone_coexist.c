/*
 * Coexistence + wire-compatibility test for SRXL2_Master_Standalone.
 *
 * Proves the two hard requirements from ArduPilot (issue ardupilot#27603):
 *
 *   1. WIRE COMPATIBILITY: bytes emitted by the standalone master are
 *      accepted and decoded by the REAL upstream spm_srxl.c parser
 *      (srxlParsePacket), and bytes emitted in genuine upstream format are
 *      parsed by the standalone master. This links the actual spm_srxl.c
 *      engine (compiled as a slave, device 0x40) into the test.
 *
 *   2. ZERO SHARED GLOBAL STATE: two standalone master instances with
 *      different device identities run in the same process without
 *      interfering -- the exact thing spm_srxl.c cannot do, because its
 *      identity lives in the process-wide global srxlThisDev.
 *
 * spm_srxl.c is driven via the shared tests/ infrastructure (test_helpers.c,
 * uart_adapter.h capture buffer). The standalone master uses its own mock HAL.
 *
 * MIT License
 */

#include "test_harness.h"
#include "test_helpers.h"     /* real spm_srxl.c driver + globals */
#include "uart_adapter.h"     /* g_tx_buf capture for the spm_srxl slave */
#include "srxl_master_standalone.h"
#include <string.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// Standalone master mock HAL
///////////////////////////////////////////////////////////////////////////////

#define M_TX_BUF_SIZE 4096
#define M_MAX_PACKETS 256

typedef struct {
    uint8_t  tx_buf[M_TX_BUF_SIZE];
    size_t   tx_len;
    struct { size_t offset; uint8_t len; } packets[M_MAX_PACKETS];
    size_t   pkt_count;
    uint32_t baud;
    uint32_t time;
    uint8_t  ctx_buf[8192];
} master_io_t;

static void master_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    master_io_t *d = user;
    if (d->tx_len + len <= M_TX_BUF_SIZE && d->pkt_count < M_MAX_PACKETS) {
        d->packets[d->pkt_count].offset = d->tx_len;
        d->packets[d->pkt_count].len = len;
        d->pkt_count++;
        memcpy(&d->tx_buf[d->tx_len], buf, len);
        d->tx_len += len;
    }
}

static void master_uart_set_baud(void *user, uint32_t baud)
{
    ((master_io_t *)user)->baud = baud;
}

static uint32_t master_time_ms(void *user)
{
    return ((master_io_t *)user)->time;
}

static SrxlMasterCtx *make_master(master_io_t *io, uint8_t device_id)
{
    memset(io, 0, sizeof(*io));
    io->baud = 115200;
    SrxlMasterConfig cfg = {
        .device = { .device_id = device_id, .priority = 20,
                    .info = SRXL_DEVINFO_TELEM_TX_ENABLED, .uid = 0x12345678 },
        .hal = { .uart_send = master_uart_send,
                 .uart_set_baud = master_uart_set_baud,
                 .time_ms = master_time_ms, .user = io },
        .baud_supported = SRXL_BAUD_400000,
    };
    return srxlMasterInitStatic(io->ctx_buf, sizeof(io->ctx_buf), &cfg);
}

static const uint8_t *master_last_pkt(master_io_t *io, uint8_t type,
                                       uint8_t *len_out)
{
    for (size_t i = io->pkt_count; i > 0; i--) {
        const uint8_t *p = &io->tx_buf[io->packets[i - 1].offset];
        if (p[1] == type) {
            *len_out = io->packets[i - 1].len;
            return p;
        }
    }
    return NULL;
}

static void master_run_to_running(SrxlMasterCtx *ctx, master_io_t *io)
{
    io->time += 50;
    srxlMasterTick(ctx);
    for (int i = 0; i < 500 && !srxlMasterIsConnected(ctx); i++)
        srxlMasterTick(ctx);
}

///////////////////////////////////////////////////////////////////////////////
// Test 1: standalone master's channel packet is decoded by real spm_srxl.c
///////////////////////////////////////////////////////////////////////////////

static void test_master_channel_accepted_by_upstream_parser(void)
{
    TEST_BEGIN(test_master_channel_accepted_by_upstream_parser);

    /* Real spm_srxl.c engine as a slave (ESC 0x40), brought to RUNNING. */
    test_init_slave(0x40);
    test_advance_ms(0, 50);
    uint8_t hs[14];
    uint8_t hlen = test_build_handshake(hs, 0x21, 0xFF, 20,
                                        SRXL_BAUD_115200, 0x03, 0xCCCCCCCC);
    test_inject_packet(0, hs, hlen);
    ASSERT_EQ(SrxlState_Running, srxlBus[0].state);
    test_reset_capture();

    /* Standalone master (Receiver 0x21) -> RUNNING with 0x40 as a peer. */
    master_io_t io;
    SrxlMasterCtx *m = make_master(&io, 0x21);
    io.time += 50;
    srxlMasterTick(m);
    for (int i = 0; i < 5; i++) srxlMasterTick(m);
    uint8_t peer_hs[14];
    uint8_t plen = test_build_handshake(peer_hs, 0x40, 0x21, 30,
                                        SRXL_BAUD_400000, 0, 0x11111111);
    srxlMasterFeed(m, peer_hs, plen);
    srxlMasterTick(m);
    for (int i = 0; i < 500 && !srxlMasterIsConnected(m); i++)
        srxlMasterTick(m);
    ASSERT_STR_EQ("RUNNING", srxlMasterGetState(m));
    ASSERT_TRUE(srxlMasterPeerCount(m) >= 1);

    /* Emit a channel frame from the standalone master. */
    uint16_t values[4] = { 1000, 2000, 3000, 4000 };
    srxlMasterSetChannels(m, values, 0x0F);
    io.pkt_count = 0; io.tx_len = 0;
    io.time += 11;
    srxlMasterTick(m);

    uint8_t clen;
    const uint8_t *chan = master_last_pkt(&io, SRXL_CTRL_ID, &clen);
    ASSERT_NOT_NULL(chan);

    /* Feed those exact bytes to the REAL spm_srxl.c parser. */
    int before = g_cb_recv_channel_count;
    bool ok = test_inject_packet(0, (uint8_t *)chan, clen);
    ASSERT_TRUE(ok);                               /* valid magic/length/CRC */
    ASSERT_EQ(before + 1, g_cb_recv_channel_count); /* channel data decoded  */

    srxlMasterDestroy(m);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Test 2: standalone master parses genuine upstream-format packets
///////////////////////////////////////////////////////////////////////////////

static void test_master_parses_upstream_handshake(void)
{
    TEST_BEGIN(test_master_parses_upstream_handshake);

    master_io_t io;
    SrxlMasterCtx *m = make_master(&io, 0x21);
    master_run_to_running(m, &io);
    ASSERT_STR_EQ("RUNNING", srxlMasterGetState(m));

    uint8_t before = srxlMasterPeerCount(m);

    /* Handshake built with the upstream test's own known-good builder. */
    uint8_t hs[14];
    uint8_t hlen = test_build_handshake(hs, 0x40, 0x21, 30,
                                        SRXL_BAUD_400000, 0, 0xDEADBEEF);
    srxlMasterFeed(m, hs, hlen);
    srxlMasterTick(m);

    ASSERT_EQ(before + 1, srxlMasterPeerCount(m));

    srxlMasterDestroy(m);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Test 3: standalone master emits an upstream-parseable handshake
///////////////////////////////////////////////////////////////////////////////

static void test_master_handshake_accepted_by_upstream_parser(void)
{
    TEST_BEGIN(test_master_handshake_accepted_by_upstream_parser);

    /* Fresh spm_srxl.c slave (device 0x40) just past startup. */
    test_init_slave(0x40);
    test_advance_ms(0, 50);
    test_reset_capture();

    master_io_t io;
    SrxlMasterCtx *m = make_master(&io, 0x21);
    io.time += 50;
    srxlMasterTick(m);
    /* Advance the scan until a handshake addressed to 0x40 is emitted. */
    const uint8_t *hs = NULL;
    uint8_t hlen = 0;
    for (int i = 0; i < 20 && !hs; i++) {
        srxlMasterTick(m);
        for (size_t k = 0; k < io.pkt_count; k++) {
            const uint8_t *p = &io.tx_buf[io.packets[k].offset];
            if (p[1] == SRXL_HANDSHAKE_ID && p[4] == 0x40) {
                hs = p; hlen = io.packets[k].len;
            }
        }
    }
    ASSERT_NOT_NULL(hs);

    /* Real parser accepts it (magic/length/CRC all valid). */
    ASSERT_TRUE(test_inject_packet(0, (uint8_t *)hs, hlen));

    srxlMasterDestroy(m);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Test 4: two standalone masters, different identities, zero interference
///////////////////////////////////////////////////////////////////////////////

static void test_two_masters_independent_state(void)
{
    TEST_BEGIN(test_two_masters_independent_state);

    master_io_t io_a, io_b;
    SrxlMasterCtx *a = make_master(&io_a, 0x21);  /* Receiver */
    SrxlMasterCtx *b = make_master(&io_b, 0x30);  /* Flight Controller */

    master_run_to_running(a, &io_a);
    master_run_to_running(b, &io_b);
    ASSERT_STR_EQ("RUNNING", srxlMasterGetState(a));
    ASSERT_STR_EQ("RUNNING", srxlMasterGetState(b));

    /* Register a peer with master A only. */
    uint8_t hs[14];
    uint8_t hlen = test_build_handshake(hs, 0x40, 0x21, 30,
                                        SRXL_BAUD_400000, 0, 0xAAAAAAAA);
    srxlMasterFeed(a, hs, hlen);
    srxlMasterTick(a);

    /* A gained a peer; B is entirely unaffected (no shared globals). */
    ASSERT_TRUE(srxlMasterPeerCount(a) >= 1);
    ASSERT_EQ(0, srxlMasterPeerCount(b));

    srxlMasterDestroy(a);
    srxlMasterDestroy(b);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Test 5: standalone master coexists with a live spm_srxl.c slave instance
///////////////////////////////////////////////////////////////////////////////

static void test_coexist_master_and_upstream_slave(void)
{
    TEST_BEGIN(test_coexist_master_and_upstream_slave);

    /* Real spm_srxl.c slave (ESC 0x40) in RUNNING, distinct identity. */
    test_init_slave(0x40);
    test_advance_ms(0, 50);
    uint8_t hs[14];
    uint8_t hlen = test_build_handshake(hs, 0x21, 0xFF, 20,
                                        SRXL_BAUD_115200, 0x03, 0xCCCCCCCC);
    test_inject_packet(0, hs, hlen);
    ASSERT_EQ(SrxlState_Running, srxlBus[0].state);
    /* The upstream engine's global identity is the slave's, not the master's. */
    ASSERT_EQ_U(0x40, srxlThisDev.devEntry.deviceID);
    test_reset_capture();

    /* Standalone master with a DIFFERENT identity, same process. */
    master_io_t io;
    SrxlMasterCtx *m = make_master(&io, 0x21);
    master_run_to_running(m, &io);
    ASSERT_STR_EQ("RUNNING", srxlMasterGetState(m));

    /* Master's identity is independent of the upstream global. */
    ASSERT_EQ_U(0x40, srxlThisDev.devEntry.deviceID);

    /* Drive one channel frame from master into the live slave. */
    uint16_t values[1] = { 32768 };
    srxlMasterSetChannels(m, values, 0x01);
    io.pkt_count = 0; io.tx_len = 0;
    io.time += 11;
    srxlMasterTick(m);
    uint8_t clen;
    const uint8_t *chan = master_last_pkt(&io, SRXL_CTRL_ID, &clen);
    ASSERT_NOT_NULL(chan);

    int before = g_cb_recv_channel_count;
    ASSERT_TRUE(test_inject_packet(0, (uint8_t *)chan, clen));
    ASSERT_EQ(before + 1, g_cb_recv_channel_count);

    srxlMasterDestroy(m);
    TEST_END();
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== SRXL2_Master_Standalone coexistence + wire-compat ===\n");

    RUN_TEST(test_master_channel_accepted_by_upstream_parser);
    RUN_TEST(test_master_parses_upstream_handshake);
    RUN_TEST(test_master_handshake_accepted_by_upstream_parser);
    RUN_TEST(test_two_masters_independent_state);
    RUN_TEST(test_coexist_master_and_upstream_slave);

    TEST_SUMMARY();
}
