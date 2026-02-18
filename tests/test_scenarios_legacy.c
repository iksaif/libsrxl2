/*
 * Shared Scenario Runner — Legacy stack (spm_srxl + spm_srxl_master)
 *
 * Implements scenario_harness_t using legacy globals and test_helpers,
 * then includes scenario_defs.inc and runs all scenarios.
 *
 * MIT License
 */

#include "test_harness.h"
#include "scenario_harness.h"
#include "test_helpers.h"
#include "uart_adapter.h"
#include "srxl2_packet.h"
#include <string.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// State string mapping
///////////////////////////////////////////////////////////////////////////////

static const char *legacy_state_string(void)
{
    switch (srxlBus[0].state) {
    case SrxlState_ListenOnStartup:
    case SrxlState_Disabled:
        return "STARTUP";
    case SrxlState_SendHandshake:
    case SrxlState_ListenForHandshake:
        return "HANDSHAKE";
    case SrxlState_Running:
    case SrxlState_SendTelemetry:
    case SrxlState_SendVTX:
    case SrxlState_SendEnterBind:
    case SrxlState_SendBoundDataReport:
    case SrxlState_SendSetBindInfo:
    case SrxlState_RequestBindInfo:
    case SrxlState_SendInternal:
        return "RUNNING";
    default:
        return "UNKNOWN";
    }
}

///////////////////////////////////////////////////////////////////////////////
// Harness implementation
///////////////////////////////////////////////////////////////////////////////

static void legacy_init_master(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    test_reset_all();
    srxlInitDevice(device_id, 20,
                    SRXL_DEVINFO_TELEM_TX_ENABLED | SRXL_DEVINFO_TELEM_FULL_RANGE,
                    0x12345678);
    srxlInitBus(0, 0, SRXL_BAUD_400000);
    srxlBus[0].master = true;
    capture_reset();
}

static void legacy_init_slave(scenario_harness_t *h, uint8_t device_id)
{
    (void)h;
    test_reset_all();
    srxlInitDevice(device_id, 10, SRXL_DEVINFO_NO_RF, 0xAABBCCDD);
    srxlInitBus(0, 0, SRXL_BAUD_400000);
    srxlBus[0].master = false;
    capture_reset();
}

static void legacy_inject(scenario_harness_t *h, const uint8_t *pkt,
                            uint8_t len)
{
    (void)h;
    /* Legacy takes non-const buffer — make a stack copy */
    uint8_t buf[80];
    memcpy(buf, pkt, len);
    srxlParsePacket(0, buf, len);
}

static void legacy_tick(scenario_harness_t *h, uint32_t ms)
{
    (void)h;
    srxlRun(0, (int16_t)ms);
}

static void legacy_reset_capture(scenario_harness_t *h)
{
    (void)h;
    capture_reset();
    /* Also reset hook counters */
    g_hook_run_start_count = 0;
    g_hook_run_end_count = 0;
    g_hook_handshake_start_count = 0;
    g_hook_handshake_complete_count = 0;
    g_hook_handshake_complete_dev_count = 0;
    g_hook_frame_count = 0;
    g_hook_channel_sent_count = 0;
    g_hook_channel_sent_telem_id = 0;
    g_hook_select_telem_count = 0;
    g_hook_select_telem_default_id = 0;
    g_hook_set_telem_tx_count = 0;
    g_hook_set_telem_tx_enabled = false;
    g_hook_telem_sent_count = 0;
    g_hook_suppress_telem_count = 0;
    g_hook_bind_count = 0;
    g_hook_parse_internal_count = 0;
    g_hook_fill_internal_count = 0;
    g_hook_frame_skip = false;
    g_hook_select_telem_passthrough = true;
    g_hook_select_telem_override = 0;
    g_cb_fill_telem_count = 0;
    g_cb_recv_channel_count = 0;
    g_cb_recv_channel_is_failsafe = false;
    g_cb_vtx_count = 0;
}

static size_t legacy_tx_count(scenario_harness_t *h)
{
    (void)h;
    return g_tx_count;
}

static const uint8_t *legacy_tx_packet(scenario_harness_t *h, size_t idx,
                                         uint8_t *len_out)
{
    (void)h;
    const uint8_t *ptr;
    size_t pkt_len;
    if (capture_get_packet(idx, &ptr, &pkt_len) != 0)
        return NULL;
    *len_out = (uint8_t)pkt_len;
    return ptr;
}

static const char *legacy_state(scenario_harness_t *h)
{
    (void)h;
    return legacy_state_string();
}

static uint32_t legacy_baud(scenario_harness_t *h)
{
    (void)h;
    return g_baud_rate;
}

static uint8_t legacy_peer_count(scenario_harness_t *h)
{
    (void)h;
    return srxlBus[0].rxDevCount;
}

static bool legacy_connected(scenario_harness_t *h)
{
    (void)h;
    return srxlBus[0].state == SrxlState_Running;
}

static void legacy_set_channels(scenario_harness_t *h, const uint16_t *values,
                                  uint32_t mask)
{
    (void)h;
    for (int i = 0; i < 32; i++) {
        if (mask & (1u << i))
            srxlChData.values[i] = values[i];
    }
    srxlBus[0].channelOutMask = mask;
}

static void legacy_set_failsafe(scenario_harness_t *h, bool failsafe)
{
    (void)h;
    srxlChDataIsFailsafe = failsafe;
    if (failsafe)
        srxlFailsafeChMask = srxlBus[0].channelOutMask ? srxlBus[0].channelOutMask : 0x01;
}

static void legacy_set_telemetry(scenario_harness_t *h,
                                   const uint8_t payload[16])
{
    (void)h;
    memcpy(srxlTelemData.raw, payload, 16);
}

static bool legacy_get_channels(scenario_harness_t *h, uint16_t *values_out,
                                  uint32_t *mask_out, bool *is_failsafe_out)
{
    (void)h;
    /* Legacy always has channel data once it was received */
    if (g_cb_recv_channel_count == 0)
        return false;
    if (values_out)
        memcpy(values_out, srxlChData.values, sizeof(srxlChData.values));
    if (mask_out)
        *mask_out = srxlChData.mask;
    if (is_failsafe_out)
        *is_failsafe_out = g_cb_recv_channel_is_failsafe;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Harness instance
///////////////////////////////////////////////////////////////////////////////

static scenario_harness_t legacy_harness = {
    .init_master   = legacy_init_master,
    .init_slave    = legacy_init_slave,
    .inject        = legacy_inject,
    .tick          = legacy_tick,
    .reset_capture = legacy_reset_capture,
    .tx_count      = legacy_tx_count,
    .tx_packet     = legacy_tx_packet,
    .state         = legacy_state,
    .baud          = legacy_baud,
    .peer_count    = legacy_peer_count,
    .connected     = legacy_connected,
    .set_channels  = legacy_set_channels,
    .set_failsafe  = legacy_set_failsafe,
    .set_telemetry = legacy_set_telemetry,
    .get_channels  = legacy_get_channels,
    .data          = NULL,
};

scenario_harness_t *g_harness = &legacy_harness;

///////////////////////////////////////////////////////////////////////////////
// Include shared scenarios
///////////////////////////////////////////////////////////////////////////////

#include "scenario_defs.inc"

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printf("=== Shared Scenarios (legacy stack) ===\n");

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
