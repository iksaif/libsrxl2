/*
 * Coexistence demo: standalone SRXL2 master + real spm_srxl.c slave, one
 * process, two device identities, zero shared global state.
 *
 * This is the scenario ArduPilot needs (issue ardupilot#27603): a flight
 * controller already runs Spektrum's spm_srxl.c engine as a slave to talk to
 * a receiver, while a *separate* SRXL2 bus needs a receiver-role master to
 * drive a Smart ESC. spm_srxl.c cannot be both, because its identity lives in
 * the process-wide global srxlThisDev. This module can, because all of its
 * state is in a caller-owned SrxlMasterCtx.
 *
 * The two engines are wired together in-process over a pair of byte FIFOs
 * standing in for a physical SRXL2 UART link:
 *
 *      standalone master (0x21)  --TX-->  [wire]  --RX-->  spm_srxl.c slave (0x40)
 *      standalone master (0x21)  <--RX--  [wire]  <--TX--  spm_srxl.c slave (0x40)
 *
 * MIT License
 */

#include "spm_srxl.h"
#include "srxl_master_standalone.h"
#include <stdio.h>
#include <string.h>

/* spm_srxl.c globals we inspect (declared here since spm_srxl.h omits them). */
extern SrxlDevice srxlThisDev;
extern SrxlBus    srxlBus[];

/*---------------------------------------------------------------------------
 * Simple byte FIFOs standing in for the two directions of one UART link
 *---------------------------------------------------------------------------*/

#define WIRE_SIZE 1024

typedef struct {
    uint8_t buf[WIRE_SIZE];
    size_t  head, tail;
} wire_t;

static wire_t wire_m2s;  /* master -> slave */
static wire_t wire_s2m;  /* slave  -> master */

static void wire_push(wire_t *w, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        size_t next = (w->head + 1) % WIRE_SIZE;
        if (next == w->tail) return;  /* full, drop */
        w->buf[w->head] = data[i];
        w->head = next;
    }
}

static size_t wire_drain(wire_t *w, uint8_t *out, size_t max)
{
    size_t n = 0;
    while (w->tail != w->head && n < max) {
        out[n++] = w->buf[w->tail];
        w->tail = (w->tail + 1) % WIRE_SIZE;
    }
    return n;
}

/*---------------------------------------------------------------------------
 * spm_srxl.c slave hooks (declared in examples/spm_srxl_config.h)
 *---------------------------------------------------------------------------*/

static int      g_slave_channel_frames = 0;
static uint32_t g_slave_baud = 115200;

void example_slave_uart_send(uint8_t uart, uint8_t *buf, uint8_t len)
{
    (void)uart;
    wire_push(&wire_s2m, buf, len);
}

void example_slave_set_baud(uint8_t uart, uint32_t baud)
{
    (void)uart;
    g_slave_baud = baud;
}

void example_slave_fill_telemetry(SrxlTelemetryData *pTelemetry)
{
    /* Report a trivial ESC-style telemetry payload. */
    memset(pTelemetry, 0, sizeof(*pTelemetry));
    pTelemetry->sensorID = 0x20;
}

void example_slave_recv_channel(SrxlChannelData *pChannelData, bool isFailsafe)
{
    (void)pChannelData;
    (void)isFailsafe;
    g_slave_channel_frames++;
}

/*---------------------------------------------------------------------------
 * Standalone master HAL
 *---------------------------------------------------------------------------*/

static uint32_t g_master_time_ms = 0;
static uint32_t g_master_baud = 115200;

static void master_uart_send(void *user, const uint8_t *buf, uint8_t len)
{
    (void)user;
    wire_push(&wire_m2s, buf, len);
}

static void master_set_baud(void *user, uint32_t baud)
{
    (void)user;
    g_master_baud = baud;
}

static uint32_t master_time_ms(void *user)
{
    (void)user;
    return g_master_time_ms;
}

static void master_on_event(SrxlMasterCtx *ctx, const SrxlMasterEvent *evt,
                            void *user)
{
    (void)ctx;
    (void)user;
    switch (evt->type) {
    case SRXL_MASTER_EVT_HANDSHAKE_COMPLETE:
        printf("  [master] handshake complete, %u peer(s)\n",
               evt->handshake.peer_count);
        break;
    case SRXL_MASTER_EVT_TELEMETRY:
        printf("  [master] telemetry from 0x%02X (sensorID 0x%02X)\n",
               evt->telemetry.device_id, evt->telemetry.data->sensorID);
        break;
    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

int main(void)
{
    printf("=== SRXL2 coexistence demo (standalone master + spm_srxl.c slave) ===\n\n");

    /* --- spm_srxl.c engine as a SLAVE, device 0x40 (ESC) --- */
    srxlInitDevice(0x40, 10, SRXL_DEVINFO_NO_RF, 0x40404040);
    srxlInitBus(0, 0, SRXL_BAUD_400000);
    srxlBus[0].master = false;
    printf("spm_srxl.c slave  : device 0x%02X (global srxlThisDev.deviceID = 0x%02X)\n",
           0x40, srxlThisDev.devEntry.deviceID);

    /* --- standalone master, device 0x21 (Receiver) --- */
    SrxlMasterConfig cfg = {
        .device = { .device_id = 0x21, .priority = 20,
                    .info = SRXL_DEVINFO_TELEM_TX_ENABLED, .uid = 0x21212121 },
        .hal = { .uart_send = master_uart_send, .uart_set_baud = master_set_baud,
                 .time_ms = master_time_ms, .user = NULL },
        .baud_supported = SRXL_BAUD_400000,
    };
    SrxlMasterCtx *master = srxlMasterInit(&cfg);
    srxlMasterOnEvent(master, master_on_event, NULL);
    printf("standalone master : device 0x21 (own SrxlMasterCtx, no globals)\n\n");
    printf("Note: the upstream global identity stays 0x%02X (the slave). "
           "The master's 0x21 identity lives only in its context.\n\n",
           srxlThisDev.devEntry.deviceID);

    uint8_t scratch[SRXL_MAX_BUFFER_SIZE];

    /* Run 300 x 1ms ticks, pumping both engines and the wires between them. */
    for (int ms = 0; ms < 300; ms++) {
        g_master_time_ms = (uint32_t)ms;

        /* Master: advance state machine. */
        srxlMasterTick(master);

        /* Deliver master -> slave, one whole packet at a time. */
        size_t n = wire_drain(&wire_m2s, scratch, sizeof(scratch));
        for (size_t i = 0; i + 2 < n; ) {
            uint8_t plen = scratch[i + 2];
            if (plen < 5 || i + plen > n) break;
            srxlParsePacket(0, &scratch[i], plen);
            i += plen;
        }

        /* Slave: advance 1ms of its state machine. */
        srxlRun(0, 1);

        /* Deliver slave -> master. */
        n = wire_drain(&wire_s2m, scratch, sizeof(scratch));
        if (n) {
            srxlMasterFeed(master, scratch, n);
            srxlMasterTick(master);
        }

        /* Once running, push some channel data every ~11ms. */
        if (srxlMasterIsConnected(master) && (ms % 11) == 0) {
            uint16_t values[4] = { 1000, 2000, 3000, 4000 };
            srxlMasterSetChannels(master, values, 0x0F);
        }
    }

    printf("\nResult:\n");
    printf("  master state              : %s\n", srxlMasterGetState(master));
    printf("  master negotiated baud    : %u\n", srxlMasterGetBaud(master));
    printf("  master peer count         : %u\n", srxlMasterPeerCount(master));
    printf("  slave state (SrxlState)   : %d (Running=%d)\n",
           srxlBus[0].state, SrxlState_Running);
    printf("  slave channel frames recv : %d\n", g_slave_channel_frames);
    printf("  upstream global deviceID  : 0x%02X (unchanged, = slave)\n",
           srxlThisDev.devEntry.deviceID);

    bool ok = srxlMasterIsConnected(master) &&
              (srxlBus[0].state == SrxlState_Running) &&
              (g_slave_channel_frames > 0) &&
              (srxlThisDev.devEntry.deviceID == 0x40);

    printf("\n%s: both engines ran independently in one process with distinct IDs.\n",
           ok ? "SUCCESS" : "FAILURE");

    srxlMasterDestroy(master);
    return ok ? 0 : 1;
}
