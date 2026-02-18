/*
 * Shared Scenario Runner — New lib (libsrxl2)
 *
 * Implements scenario_harness_t using the srxl2_ctx_t API,
 * then includes scenario_defs.inc and runs all scenarios.
 *
 * MIT License
 */

#include "test_harness.h"
#include "scenario_harness.h"
#include "srxl2.h"
#include "srxl2_internal.h"
#include "srxl2_packet.h"
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
    uint8_t  ctx_buf[4096];
    srxl2_ctx_t *ctx;
} new_data_t;

static new_data_t g_new;

static void new_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    new_data_t *d = &g_new;
    if (d->tx_len + len <= MOCK_TX_BUF_SIZE &&
        d->pkt_count < MOCK_MAX_PACKETS) {
        d->packets[d->pkt_count].offset = d->tx_len;
        d->packets[d->pkt_count].len = len;
        d->pkt_count++;
        memcpy(&d->tx_buf[d->tx_len], buf, len);
        d->tx_len += len;
    }
}

static void new_uart_set_baud(void *user, uint32_t baud)
{
    (void)user;
    g_new.baud = baud;
}

static uint32_t new_time_ms(void *user)
{
    (void)user;
    return g_new.time;
}

///////////////////////////////////////////////////////////////////////////////
// Harness implementation
///////////////////////////////////////////////////////////////////////////////

static void new_init_master(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    new_data_t *d = &g_new;
    memset(d, 0, sizeof(*d));
    d->baud = 115200;
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = {
            .device_id = device_id,
            .priority = 20,
            .info = SRXL2_DEVINFO_TELEM_TX_ENABLED | SRXL2_DEVINFO_TELEM_FULL_RANGE,
            .uid = 0x12345678,
        },
        .hal = {
            .uart_send = new_uart_send,
            .uart_set_baud = new_uart_set_baud,
            .time_ms = new_time_ms,
            .user = NULL,
        },
        .baud_supported = SRXL2_BAUD_400000,
    };
    d->ctx = srxl2_init_static(d->ctx_buf, sizeof(d->ctx_buf), &cfg);
}

static void new_init_slave(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    new_data_t *d = &g_new;
    memset(d, 0, sizeof(*d));
    d->baud = 115200;
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_SLAVE,
        .device = {
            .device_id = device_id,
            .priority = 10,
            .info = SRXL2_DEVINFO_NO_RF,
            .uid = 0xAABBCCDD,
        },
        .hal = {
            .uart_send = new_uart_send,
            .uart_set_baud = new_uart_set_baud,
            .time_ms = new_time_ms,
            .user = NULL,
        },
        .baud_supported = SRXL2_BAUD_400000,
    };
    d->ctx = srxl2_init_static(d->ctx_buf, sizeof(d->ctx_buf), &cfg);
}

static void new_inject(scenario_harness_t *h, const uint8_t *pkt, uint8_t len)
{
    (void)h;
    srxl2_feed(g_new.ctx, pkt, len);
    srxl2_tick(g_new.ctx);
}

static void new_tick(scenario_harness_t *h, uint32_t ms)
{
    (void)h;
    g_new.time += ms;
    srxl2_tick(g_new.ctx);
}

static void new_reset_capture(scenario_harness_t *h)
{
    (void)h;
    g_new.tx_len = 0;
    g_new.pkt_count = 0;
}

static size_t new_tx_count(scenario_harness_t *h)
{
    (void)h;
    return g_new.pkt_count;
}

static const uint8_t *new_tx_packet(scenario_harness_t *h, size_t idx,
                                     uint8_t *len_out)
{
    (void)h;
    if (idx >= g_new.pkt_count) return NULL;
    *len_out = g_new.packets[idx].len;
    return &g_new.tx_buf[g_new.packets[idx].offset];
}

static const char *new_state(scenario_harness_t *h)
{
    (void)h;
    return srxl2_get_state(g_new.ctx);
}

static uint32_t new_baud(scenario_harness_t *h)
{
    (void)h;
    return g_new.baud;
}

static uint8_t new_peer_count(scenario_harness_t *h)
{
    (void)h;
    return srxl2_peer_count(g_new.ctx);
}

static bool new_connected(scenario_harness_t *h)
{
    (void)h;
    return srxl2_is_connected(g_new.ctx);
}

static void new_set_channels(scenario_harness_t *h, const uint16_t *values,
                               uint32_t mask)
{
    (void)h;
    srxl2_set_channels(g_new.ctx, values, mask);
}

static void new_set_failsafe(scenario_harness_t *h, bool failsafe)
{
    (void)h;
    srxl2_set_failsafe(g_new.ctx, failsafe);
}

static void new_set_telemetry(scenario_harness_t *h, const uint8_t payload[16])
{
    (void)h;
    srxl2_set_telemetry(g_new.ctx, payload);
}

static bool new_get_channels(scenario_harness_t *h, uint16_t *values_out,
                               uint32_t *mask_out, bool *is_failsafe_out)
{
    (void)h;
    srxl2_channel_data_t data;
    if (!srxl2_get_channels(g_new.ctx, &data))
        return false;
    if (values_out) memcpy(values_out, data.values, sizeof(data.values));
    if (mask_out) *mask_out = data.mask;
    if (is_failsafe_out) *is_failsafe_out = data.is_failsafe;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Harness instance
///////////////////////////////////////////////////////////////////////////////

static scenario_harness_t new_harness = {
    .init_master   = new_init_master,
    .init_slave    = new_init_slave,
    .inject        = new_inject,
    .tick          = new_tick,
    .reset_capture = new_reset_capture,
    .tx_count      = new_tx_count,
    .tx_packet     = new_tx_packet,
    .state         = new_state,
    .baud          = new_baud,
    .peer_count    = new_peer_count,
    .connected     = new_connected,
    .set_channels  = new_set_channels,
    .set_failsafe  = new_set_failsafe,
    .set_telemetry = new_set_telemetry,
    .get_channels  = new_get_channels,
    .data          = &g_new,
};

scenario_harness_t *g_harness = &new_harness;

///////////////////////////////////////////////////////////////////////////////
// Include shared scenarios
///////////////////////////////////////////////////////////////////////////////

#include "scenario_defs.inc"

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Shared Scenarios (new lib) ===\n");

    /* Slave scenarios */
    RUN_TEST(scenario_slave_startup);
    RUN_TEST(scenario_slave_startup_delay);
    RUN_TEST(scenario_slave_unprompted_handshake);
    RUN_TEST(scenario_slave_nonzero_listens);
    RUN_TEST(scenario_slave_replies_to_handshake);
    RUN_TEST(scenario_slave_ignores_other_handshake);
    RUN_TEST(scenario_slave_broadcast_to_running);
    RUN_TEST(scenario_slave_broadcast_baud_115200);
    RUN_TEST(scenario_slave_broadcast_baud_400000);
    RUN_TEST(scenario_slave_receives_channel_data);
    RUN_TEST(scenario_slave_sends_telemetry_when_polled);
    RUN_TEST(scenario_slave_silent_when_not_polled);
    RUN_TEST(scenario_slave_timeout_resets);
    RUN_TEST(scenario_slave_receives_failsafe);

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

    /* Wire format scenarios */
    RUN_TEST(scenario_wire_handshake_reply_format);
    RUN_TEST(scenario_wire_telemetry_reply_format);

    TEST_SUMMARY();
}
