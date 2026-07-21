/*
 * Shared Scenario Runner — SRXL2_Master_Standalone
 *
 * Implements scenario_harness_t using the SrxlMasterCtx API, then includes
 * scenario_defs.inc and runs the master + master-facing scenarios.
 *
 * This module is master-only (no slave role), so slave/wire scenarios that
 * require init_slave() are not run here; their functions are still compiled
 * from the shared scenario_defs.inc and referenced (void) in main() to keep
 * -Wall -Wextra quiet.
 *
 * MIT License
 */

#include "test_harness.h"
#include "scenario_harness.h"
#include "srxl2_packet.h"   /* test packet builders (srxl2_pkt_*) */
#include "srxl_master_standalone.h"
#include <string.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// Mock HAL
///////////////////////////////////////////////////////////////////////////////

#define MOCK_TX_BUF_SIZE 4096
#define MOCK_MAX_PACKETS 256

typedef struct {
    uint8_t  tx_buf[MOCK_TX_BUF_SIZE];
    size_t   tx_len;
    struct { size_t offset; uint8_t len; } packets[MOCK_MAX_PACKETS];
    size_t   pkt_count;
    uint32_t baud;
    uint32_t time;
    uint8_t  ctx_buf[8192];
    SrxlMasterCtx *ctx;
} std_data_t;

static std_data_t g_std;

static void std_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    std_data_t *d = &g_std;
    if (d->tx_len + len <= MOCK_TX_BUF_SIZE &&
        d->pkt_count < MOCK_MAX_PACKETS) {
        d->packets[d->pkt_count].offset = d->tx_len;
        d->packets[d->pkt_count].len = len;
        d->pkt_count++;
        memcpy(&d->tx_buf[d->tx_len], buf, len);
        d->tx_len += len;
    }
}

static void std_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    g_std.baud = baud;
}

static uint32_t std_time_ms(void *user)
{
    (void)user;
    return g_std.time;
}

///////////////////////////////////////////////////////////////////////////////
// Harness implementation
///////////////////////////////////////////////////////////////////////////////

static void std_init_master(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    std_data_t *d = &g_std;
    memset(d, 0, sizeof(*d));
    d->baud = 115200;
    SrxlMasterConfig cfg = {
        .device = {
            .device_id = device_id,
            .priority = 20,
            .info = SRXL_DEVINFO_TELEM_TX_ENABLED | SRXL_DEVINFO_TELEM_FULL_RANGE,
            .uid = 0x12345678,
        },
        .hal = {
            .uart_send = std_uart_send,
            .uart_set_baud = std_uart_set_baud,
            .time_ms = std_time_ms,
            .user = NULL,
        },
        .baud_supported = SRXL_BAUD_400000,
    };
    d->ctx = srxlMasterInitStatic(d->ctx_buf, sizeof(d->ctx_buf), &cfg);
}

/* Master-only module -- slave init is unsupported. Provided so the vtable
 * is complete; scenarios that call it are never run from this runner. */
static void std_init_slave(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    (void)device_id;
}

static void std_inject(scenario_harness_t *h, const uint8_t *pkt, uint8_t len)
{
    (void)h;
    srxlMasterFeed(g_std.ctx, pkt, len);
    srxlMasterTick(g_std.ctx);
}

static void std_tick(scenario_harness_t *h, uint32_t ms)
{
    (void)h;
    g_std.time += ms;
    srxlMasterTick(g_std.ctx);
}

static void std_reset_capture(scenario_harness_t *h)
{
    (void)h;
    g_std.tx_len = 0;
    g_std.pkt_count = 0;
}

static size_t std_tx_count(scenario_harness_t *h)
{
    (void)h;
    return g_std.pkt_count;
}

static const uint8_t *std_tx_packet(scenario_harness_t *h, size_t idx,
                                     uint8_t *len_out)
{
    (void)h;
    if (idx >= g_std.pkt_count) return NULL;
    *len_out = g_std.packets[idx].len;
    return &g_std.tx_buf[g_std.packets[idx].offset];
}

static const char *std_state(scenario_harness_t *h)
{
    (void)h;
    return srxlMasterGetState(g_std.ctx);
}

static uint32_t std_baud(scenario_harness_t *h)
{
    (void)h;
    return g_std.baud;
}

static uint8_t std_peer_count(scenario_harness_t *h)
{
    (void)h;
    return srxlMasterPeerCount(g_std.ctx);
}

static bool std_connected(scenario_harness_t *h)
{
    (void)h;
    return srxlMasterIsConnected(g_std.ctx);
}

static void std_set_channels(scenario_harness_t *h, const uint16_t *values,
                              uint32_t mask)
{
    (void)h;
    srxlMasterSetChannels(g_std.ctx, values, mask);
}

static void std_set_failsafe(scenario_harness_t *h, bool failsafe)
{
    (void)h;
    srxlMasterSetFailsafe(g_std.ctx, failsafe);
}

/* Slave ops -- unsupported in a master-only module. */
static void std_set_telemetry(scenario_harness_t *h, const uint8_t payload[16])
{
    (void)h;
    (void)payload;
}

static bool std_get_channels(scenario_harness_t *h, uint16_t *values_out,
                             uint32_t *mask_out, bool *is_failsafe_out)
{
    (void)h;
    (void)values_out;
    (void)mask_out;
    (void)is_failsafe_out;
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// Harness instance
///////////////////////////////////////////////////////////////////////////////

static scenario_harness_t std_harness = {
    .init_master   = std_init_master,
    .init_slave    = std_init_slave,
    .inject        = std_inject,
    .tick          = std_tick,
    .reset_capture = std_reset_capture,
    .tx_count      = std_tx_count,
    .tx_packet     = std_tx_packet,
    .state         = std_state,
    .baud          = std_baud,
    .peer_count    = std_peer_count,
    .connected     = std_connected,
    .set_channels  = std_set_channels,
    .set_failsafe  = std_set_failsafe,
    .set_telemetry = std_set_telemetry,
    .get_channels  = std_get_channels,
    .data          = &g_std,
};

scenario_harness_t *g_harness = &std_harness;

///////////////////////////////////////////////////////////////////////////////
// Include shared scenarios
///////////////////////////////////////////////////////////////////////////////

#include "scenario_defs.inc"

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Shared Scenarios (standalone master) ===\n");

    /* Master scenarios */
    RUN_TEST(scenario_master_startup);
    RUN_TEST(scenario_master_startup_delay);
    RUN_TEST(scenario_master_handshake_completes);
    RUN_TEST(scenario_master_broadcast_sent);
    RUN_TEST(scenario_master_registers_peer);
    RUN_TEST(scenario_master_baud_negotiation_and);
    RUN_TEST(scenario_master_sends_channel_on_timeout);
    RUN_TEST(scenario_master_channel_data_format);
    RUN_TEST(scenario_master_sends_failsafe);
    RUN_TEST(scenario_master_receives_telemetry);

    /* Slave/wire scenarios require a slave role this module does not have.
     * Reference them so -Wunused-function stays quiet. */
    (void)&scenario_slave_startup;
    (void)&scenario_slave_startup_delay;
    (void)&scenario_slave_unprompted_handshake;
    (void)&scenario_slave_nonzero_listens;
    (void)&scenario_slave_replies_to_handshake;
    (void)&scenario_slave_ignores_other_handshake;
    (void)&scenario_slave_broadcast_to_running;
    (void)&scenario_slave_broadcast_baud_115200;
    (void)&scenario_slave_broadcast_baud_400000;
    (void)&scenario_slave_receives_channel_data;
    (void)&scenario_slave_sends_telemetry_when_polled;
    (void)&scenario_slave_silent_when_not_polled;
    (void)&scenario_slave_timeout_resets;
    (void)&scenario_slave_receives_failsafe;
    (void)&scenario_wire_handshake_reply_format;
    (void)&scenario_wire_telemetry_reply_format;
    (void)&scenario_helper_slave_to_running;

    TEST_SUMMARY();
}
