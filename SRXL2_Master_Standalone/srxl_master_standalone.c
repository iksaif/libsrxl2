/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * NOTE ON ENDIANNESS: like Spektrum's own spm_srxl.c (see srxlSend(),
 * e.g. `pBus->srxlOut.handshake.payload.uid = srxlThisDev.uid;`), this file
 * fills multi-byte packet fields via direct struct assignment into the
 * packed spm_srxl.h structs. That only produces correct SRXL2 wire bytes
 * on little-endian hosts (every ARM Cortex-M/A target and x86, which is the
 * entire practical target set for this protocol). Only the CRC is written
 * as explicit big-endian bytes, matching the wire spec and matching
 * spm_srxl.c's own srxlSend() convention.
 */

#include "srxl_master_standalone.h"
#include <string.h>
#include <stdlib.h>

/*---------------------------------------------------------------------------
 * Internal constants
 *---------------------------------------------------------------------------*/

#define SRXL_MASTER_RX_BUF_SIZE        160

#define SRXL_MASTER_STARTUP_DELAY_MS   50
#define SRXL_MASTER_FRAME_PERIOD_115200 11
#define SRXL_MASTER_FRAME_PERIOD_400000 6
#define SRXL_MASTER_RESPONSE_WAIT_115200 4
#define SRXL_MASTER_RESPONSE_WAIT_400000 2

#define SRXL_MASTER_SCAN_TABLE_SIZE 10

static const uint8_t default_scan_ids[SRXL_MASTER_SCAN_TABLE_SIZE] = {
    0x21,  /* Receiver */
    0x30,  /* Flight Controller */
    0x40,  /* ESC 0 */
    0x41,  /* ESC 1 */
    0x42,  /* ESC 2 */
    0x43,  /* ESC 3 */
    0x60,  /* Servo 1 */
    0x70,  /* Servo 2 */
    0x81,  /* VTX */
    0xB0,  /* Sensor */
};

/*---------------------------------------------------------------------------
 * CRC-16 (XMODEM, polynomial 0x1021, seed 0) -- vendored table, not shared
 * with spm_srxl.c's static srxlCrc16()/srxlCRCTable.
 *---------------------------------------------------------------------------*/

#ifdef SRXL_MASTER_CRC_EXTERN
/* User provides: uint16_t srxlMasterCrc16(const uint8_t *data, size_t len) */

#elif !defined(SRXL_MASTER_CRC_SMALL)

#include "srxl_master_crc_table.inc"

uint16_t srxlMasterCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t pos = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ srxl_master_crc_table[pos]);
    }
    return crc;
}

#else /* SRXL_MASTER_CRC_SMALL -- bitwise fallback */

uint16_t srxlMasterCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)(crc ^ ((uint16_t)data[i] << 8));
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

#endif

/* Append the big-endian CRC to a filled-in packet buffer and return its
 * total wire length (mirrors spm_srxl.c's srxlSend() CRC convention). */
static uint8_t finalize_packet(uint8_t *raw, uint8_t len)
{
    uint16_t crc = srxlMasterCrc16(raw, (size_t)(len - 2));
    raw[len - 2] = (uint8_t)((crc >> 8) & 0xFF);
    raw[len - 1] = (uint8_t)(crc & 0xFF);
    return len;
}

/*---------------------------------------------------------------------------
 * Peer entry
 *---------------------------------------------------------------------------*/

typedef struct SrxlMasterPeer
{
    uint8_t  device_id;
    uint8_t  priority;
    uint8_t  info;
    uint32_t uid;
    uint16_t telem_age;
    uint8_t  telem_payload[16];
    uint32_t telem_rx_ms;
    bool     telem_valid;
} SrxlMasterPeer;

/*---------------------------------------------------------------------------
 * TX pending flags
 *---------------------------------------------------------------------------*/

typedef struct SrxlMasterTxFlags
{
    bool enter_bind;
    bool set_bind;
    bool send_vtx;
    bool send_fwd_pgm;
} SrxlMasterTxFlags;

/*---------------------------------------------------------------------------
 * Context struct (definition -- opaque to callers via the header)
 *---------------------------------------------------------------------------*/

struct SrxlMasterCtx
{
    SrxlMasterConfig config;

    SrxlState state;
    uint32_t  state_entered_ms;
    uint32_t  last_rx_ms;
    uint32_t  last_tx_ms;

    SrxlMasterPeer peers[SRXL_MASTER_MAX_DEVICES];
    uint8_t        peer_count;
    uint16_t       peer_priority_sum;

    uint8_t negotiated_baud;

    /* Outgoing channel data */
    SrxlChannelData chan_out;
    bool             chan_out_failsafe;

    /* Handshake scan */
    uint8_t hs_scan_table[SRXL_MASTER_SCAN_TABLE_SIZE];
    uint8_t hs_scan_count;
    uint8_t hs_scan_idx;
    uint8_t hs_baud_and;

    /* Bind state */
    SrxlBindData bind_info;
    uint8_t      bind_target_id;

    /* VTX state */
    SrxlVtxData vtx_data;

    /* Forward programming */
    uint8_t fwd_pgm_buf[FWD_PGM_MAX_DATA_SIZE];
    uint8_t fwd_pgm_len;
    uint8_t fwd_pgm_target;

    SrxlMasterTxFlags tx_flags;

    /* Telemetry scheduling */
    uint8_t telem_poll_idx;

    /* RX ring buffer (ISR writes, tick reads) -- SPSC lock-free */
    uint8_t          rx_ring[SRXL_MASTER_RX_BUF_SIZE];
    volatile uint8_t rx_head;
    uint8_t          rx_tail;

    /* Frame assembly buffer */
    uint8_t frame_buf[SRXL_MAX_BUFFER_SIZE];
    uint8_t frame_len;
    bool    frame_synced;

    /* TX scratch packet */
    SrxlPacket tx_pkt;

    uint16_t frame_count;

    SrxlMasterEventCb event_cb;
    void              *event_cb_user;

    bool malloced;
};

/*---------------------------------------------------------------------------
 * Internal helpers
 *---------------------------------------------------------------------------*/

static uint32_t now_ms(SrxlMasterCtx *ctx)
{
    return ctx->config.hal.time_ms(ctx->config.hal.user);
}

static void hal_send(SrxlMasterCtx *ctx, const uint8_t *buf, uint8_t len)
{
    ctx->config.hal.uart_send(ctx->config.hal.user, buf, len);
    ctx->last_tx_ms = now_ms(ctx);
}

static void hal_set_baud(SrxlMasterCtx *ctx, uint32_t baud)
{
    ctx->config.hal.uart_set_baud(ctx->config.hal.user, baud);
}

static uint32_t baud_to_rate(uint8_t baud_flag)
{
    return (baud_flag & SRXL_BAUD_400000) ? 400000 : 115200;
}

static uint8_t frame_period_ms(SrxlMasterCtx *ctx)
{
    return (ctx->negotiated_baud & SRXL_BAUD_400000)
        ? SRXL_MASTER_FRAME_PERIOD_400000
        : SRXL_MASTER_FRAME_PERIOD_115200;
}

static uint8_t response_wait_ms(SrxlMasterCtx *ctx)
{
    return (ctx->negotiated_baud & SRXL_BAUD_400000)
        ? SRXL_MASTER_RESPONSE_WAIT_400000
        : SRXL_MASTER_RESPONSE_WAIT_115200;
}

static void fire_event(SrxlMasterCtx *ctx, const SrxlMasterEvent *evt)
{
    if (ctx->event_cb)
        ctx->event_cb(ctx, evt, ctx->event_cb_user);
}

static void enter_state(SrxlMasterCtx *ctx, SrxlState state)
{
    ctx->state = state;
    ctx->state_entered_ms = now_ms(ctx);
}

static uint32_t time_since_tx(SrxlMasterCtx *ctx)
{
    return now_ms(ctx) - ctx->last_tx_ms;
}

/*---------------------------------------------------------------------------
 * Peer management
 *---------------------------------------------------------------------------*/

static SrxlMasterPeer *find_peer(SrxlMasterCtx *ctx, uint8_t device_id)
{
    for (uint8_t i = 0; i < ctx->peer_count; i++) {
        if (ctx->peers[i].device_id == device_id)
            return &ctx->peers[i];
    }
    return NULL;
}

static SrxlMasterPeer *add_peer(SrxlMasterCtx *ctx, uint8_t device_id,
                                 uint8_t priority, uint8_t info, uint32_t uid)
{
    SrxlMasterPeer *p = find_peer(ctx, device_id);
    if (p) {
        p->priority = priority;
        p->info = info;
        p->uid = uid;
        return p;
    }

    if (ctx->peer_count >= SRXL_MASTER_MAX_DEVICES)
        return NULL;

    p = &ctx->peers[ctx->peer_count++];
    memset(p, 0, sizeof(*p));
    p->device_id = device_id;
    p->priority = priority;
    p->info = info;
    p->uid = uid;

    ctx->peer_priority_sum = 0;
    for (uint8_t i = 0; i < ctx->peer_count; i++)
        ctx->peer_priority_sum += ctx->peers[i].priority;

    return p;
}

/*---------------------------------------------------------------------------
 * Telemetry scheduling: priority * age scoring
 *---------------------------------------------------------------------------*/

static uint8_t select_telem_peer(SrxlMasterCtx *ctx)
{
    if (ctx->peer_count == 0 || ctx->peer_priority_sum == 0)
        return 0;

    uint32_t best_score = 0;
    uint8_t best_idx = 0;

    for (uint8_t i = 0; i < ctx->peer_count; i++) {
        uint32_t score = (uint32_t)ctx->peers[i].priority *
                         (ctx->peers[i].telem_age + 1);
        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}

static void age_telem_counters(SrxlMasterCtx *ctx)
{
    for (uint8_t i = 0; i < ctx->peer_count; i++) {
        if (ctx->peers[i].telem_age < 0xFFFF)
            ctx->peers[i].telem_age++;
    }
}

/*---------------------------------------------------------------------------
 * Ring buffer helpers (SPSC: feed() produces, tick() consumes)
 *---------------------------------------------------------------------------*/

static uint8_t ring_used(const SrxlMasterCtx *ctx)
{
    return (uint8_t)((ctx->rx_head - ctx->rx_tail) % SRXL_MASTER_RX_BUF_SIZE);
}

static uint8_t ring_pop(SrxlMasterCtx *ctx)
{
    uint8_t val = ctx->rx_ring[ctx->rx_tail];
    ctx->rx_tail = (uint8_t)((ctx->rx_tail + 1) % SRXL_MASTER_RX_BUF_SIZE);
    return val;
}

/* Assemble one complete frame from the ring into frame_buf[]. */
static bool try_assemble_frame(SrxlMasterCtx *ctx)
{
    while (ring_used(ctx) > 0) {
        uint8_t byte = ring_pop(ctx);

        if (!ctx->frame_synced) {
            if (byte == SPEKTRUM_SRXL_ID) {
                ctx->frame_buf[0] = byte;
                ctx->frame_len = 1;
                ctx->frame_synced = true;
            }
            continue;
        }

        if (ctx->frame_len < SRXL_MAX_BUFFER_SIZE) {
            ctx->frame_buf[ctx->frame_len++] = byte;
        } else {
            ctx->frame_synced = false;
            ctx->frame_len = 0;
            continue;
        }

        if (ctx->frame_len >= 3) {
            uint8_t expected = ctx->frame_buf[2];
            if (expected < 5 || expected > SRXL_MAX_BUFFER_SIZE) {
                ctx->frame_synced = false;
                ctx->frame_len = 0;
                continue;
            }
            if (ctx->frame_len >= expected) {
                ctx->frame_synced = false;
                return true;
            }
        }
    }
    return false;
}

/*---------------------------------------------------------------------------
 * Packet builders -- write directly into the shared SrxlPacket scratch
 * struct fields (spm_srxl.h types), then append the CRC as raw bytes.
 *---------------------------------------------------------------------------*/

static uint8_t build_handshake(SrxlMasterCtx *ctx, uint8_t dest_id,
                                uint8_t baud_supported)
{
    SrxlPacket *pkt = &ctx->tx_pkt;
    uint8_t len = (uint8_t)sizeof(SrxlHandshakePacket);

    pkt->header.srxlID = SPEKTRUM_SRXL_ID;
    pkt->header.packetType = SRXL_HANDSHAKE_ID;
    pkt->header.length = len;
    pkt->handshake.payload.srcDevID = ctx->config.device.device_id;
    pkt->handshake.payload.destDevID = dest_id;
    pkt->handshake.payload.priority = ctx->config.device.priority;
    pkt->handshake.payload.baudSupported = baud_supported;
    pkt->handshake.payload.info = ctx->config.device.info;
    pkt->handshake.payload.uid = ctx->config.device.uid;

    return finalize_packet(pkt->raw, len);
}

static uint8_t build_channel(SrxlMasterCtx *ctx, uint8_t cmd, uint8_t reply_id)
{
    SrxlPacket *pkt = &ctx->tx_pkt;

    uint8_t num_ch = 0;
    for (int i = 0; i < 32; i++) {
        if (ctx->chan_out.mask & (1u << i))
            num_ch++;
    }

    /* SrxlControlData(cmd+replyID) + SrxlChannelData(rssi+frameLosses+mask+values[N]) */
    uint8_t len = (uint8_t)(3 /* hdr */ + 2 /* cmd+replyID */ +
                            1 /* rssi */ + 2 /* frameLosses */ + 4 /* mask */ +
                            num_ch * 2 + 2 /* crc */);

    pkt->header.srxlID = SPEKTRUM_SRXL_ID;
    pkt->header.packetType = SRXL_CTRL_ID;
    pkt->header.length = len;
    pkt->control.payload.cmd = cmd;
    pkt->control.payload.replyID = reply_id;
    pkt->control.payload.channelData.rssi = ctx->chan_out.rssi;
    pkt->control.payload.channelData.frameLosses = ctx->chan_out.frameLosses;
    pkt->control.payload.channelData.mask = ctx->chan_out.mask;

    uint8_t ch_idx = 0;
    for (int i = 0; i < 32; i++) {
        if (ctx->chan_out.mask & (1u << i))
            pkt->control.payload.channelData.values[ch_idx++] = ctx->chan_out.values[i];
    }

    return finalize_packet(pkt->raw, len);
}

static uint8_t build_bind(SrxlMasterCtx *ctx, uint8_t request, uint8_t device_id)
{
    SrxlPacket *pkt = &ctx->tx_pkt;
    uint8_t len = (uint8_t)sizeof(SrxlBindPacket);

    pkt->header.srxlID = SPEKTRUM_SRXL_ID;
    pkt->header.packetType = SRXL_BIND_ID;
    pkt->header.length = len;
    pkt->bind.request = request;
    pkt->bind.deviceID = device_id;
    pkt->bind.data = ctx->bind_info;

    return finalize_packet(pkt->raw, len);
}

static uint8_t build_vtx(SrxlMasterCtx *ctx, uint8_t reply_id)
{
    SrxlPacket *pkt = &ctx->tx_pkt;
    uint8_t len = (uint8_t)(3 + 2 + sizeof(SrxlVtxData) + 2);

    pkt->header.srxlID = SPEKTRUM_SRXL_ID;
    pkt->header.packetType = SRXL_CTRL_ID;
    pkt->header.length = len;
    pkt->control.payload.cmd = SRXL_CTRL_CMD_VTX;
    pkt->control.payload.replyID = reply_id;
    pkt->control.payload.vtxData = ctx->vtx_data;

    return finalize_packet(pkt->raw, len);
}

static uint8_t build_fwd_pgm(SrxlMasterCtx *ctx, uint8_t reply_id)
{
    SrxlPacket *pkt = &ctx->tx_pkt;
    uint8_t len = (uint8_t)(3 + 2 + 3 + ctx->fwd_pgm_len + 2);

    pkt->header.srxlID = SPEKTRUM_SRXL_ID;
    pkt->header.packetType = SRXL_CTRL_ID;
    pkt->header.length = len;
    pkt->control.payload.cmd = SRXL_CTRL_CMD_FWDPGM;
    pkt->control.payload.replyID = reply_id;
    pkt->control.payload.fpData.rssi = 0;
    pkt->control.payload.fpData.rfu[0] = 0;
    pkt->control.payload.fpData.rfu[1] = 0;
    memcpy(pkt->control.payload.fpData.data, ctx->fwd_pgm_buf, ctx->fwd_pgm_len);

    return finalize_packet(pkt->raw, len);
}

/*---------------------------------------------------------------------------
 * Dispatch received packets
 *---------------------------------------------------------------------------*/

static void on_handshake(SrxlMasterCtx *ctx, const SrxlHandshakeData *hs)
{
    ctx->last_rx_ms = now_ms(ctx);

    if (hs->srcDevID == ctx->config.device.device_id)
        return;

    add_peer(ctx, hs->srcDevID, hs->priority, hs->info, hs->uid);
    ctx->hs_baud_and &= hs->baudSupported;

    /* Late join: unprompted handshake (dest=0) while running -- rebroadcast
     * so the new device transitions straight to Running. */
    if (hs->destDevID == 0x00 && ctx->state == SrxlState_Running) {
        uint8_t len = build_handshake(ctx, 0xFF, ctx->hs_baud_and);
        hal_send(ctx, ctx->tx_pkt.raw, len);
    }
}

static void on_telemetry(SrxlMasterCtx *ctx, const SrxlTelemetryPacket *tm)
{
    ctx->last_rx_ms = now_ms(ctx);

    /* destDevID == 0xFF means the sender wants a re-handshake. */
    if (tm->destDevID == 0xFF) {
        enter_state(ctx, SrxlState_SendHandshake);
        ctx->hs_scan_idx = 0;
        ctx->hs_baud_and = ctx->config.baud_supported;
        return;
    }

    if (ctx->peer_count == 0)
        return;

    uint8_t idx = ctx->telem_poll_idx;
    if (idx >= ctx->peer_count)
        return;

    SrxlMasterPeer *p = &ctx->peers[idx];
    memcpy(p->telem_payload, tm->payload.raw, 16);
    p->telem_rx_ms = now_ms(ctx);
    p->telem_valid = true;

    SrxlMasterEvent evt = {0};
    evt.type = SRXL_MASTER_EVT_TELEMETRY;
    evt.telemetry.device_id = p->device_id;
    evt.telemetry.data = &tm->payload;
    fire_event(ctx, &evt);
}

static void on_bind(SrxlMasterCtx *ctx, const SrxlBindPacket *bd)
{
    ctx->last_rx_ms = now_ms(ctx);

    SrxlMasterEvent evt = {0};
    evt.type = SRXL_MASTER_EVT_BIND;
    evt.bind.request = bd->request;
    evt.bind.device_id = bd->deviceID;
    evt.bind.data = &bd->data;
    fire_event(ctx, &evt);
}

static void on_vtx(SrxlMasterCtx *ctx, const SrxlVtxData *vtx)
{
    ctx->last_rx_ms = now_ms(ctx);
    ctx->vtx_data = *vtx;

    SrxlMasterEvent evt = {0};
    evt.type = SRXL_MASTER_EVT_VTX;
    evt.vtx.data = &ctx->vtx_data;
    fire_event(ctx, &evt);
}

static void dispatch_frame(SrxlMasterCtx *ctx, const uint8_t *raw, uint8_t len)
{
    if (len < 5)
        return;

    /* Validate CRC before interpreting the union as any packet type. */
    uint16_t computed = srxlMasterCrc16(raw, (size_t)(len - 2));
    uint16_t wire_crc = (uint16_t)(((uint16_t)raw[len - 2] << 8) | raw[len - 1]);
    if (computed != wire_crc)
        return;

    const SrxlPacket *pkt = (const SrxlPacket *)raw;

    switch (raw[1]) {
    case SRXL_HANDSHAKE_ID:
        if (len >= sizeof(SrxlHandshakePacket))
            on_handshake(ctx, &pkt->handshake.payload);
        break;
    case SRXL_TELEM_ID:
        if (len >= sizeof(SrxlTelemetryPacket))
            on_telemetry(ctx, &pkt->telemetry);
        break;
    case SRXL_BIND_ID:
        if (len >= sizeof(SrxlBindPacket))
            on_bind(ctx, &pkt->bind);
        break;
    case SRXL_CTRL_ID:
        if (len >= SRXL_CTRL_BASE_LENGTH && pkt->control.payload.cmd == SRXL_CTRL_CMD_VTX)
            on_vtx(ctx, &pkt->control.payload.vtxData);
        break;
    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * TX scheduling
 *---------------------------------------------------------------------------*/

static void send_handshake_to(SrxlMasterCtx *ctx, uint8_t dest_id)
{
    uint8_t len = build_handshake(ctx, dest_id, ctx->hs_baud_and);
    hal_send(ctx, ctx->tx_pkt.raw, len);
}

static void send_channel(SrxlMasterCtx *ctx)
{
    uint8_t poll_idx = select_telem_peer(ctx);
    uint8_t reply_id = 0;

    if (!ctx->chan_out_failsafe && poll_idx < ctx->peer_count) {
        reply_id = ctx->peers[poll_idx].device_id;
        ctx->telem_poll_idx = poll_idx;
    }

    uint8_t cmd = ctx->chan_out_failsafe ? SRXL_CTRL_CMD_CHANNEL_FS : SRXL_CTRL_CMD_CHANNEL;
    uint8_t len = build_channel(ctx, cmd, reply_id);
    hal_send(ctx, ctx->tx_pkt.raw, len);

    age_telem_counters(ctx);
    if (!ctx->chan_out_failsafe && poll_idx < ctx->peer_count)
        ctx->peers[poll_idx].telem_age = 0;

    ctx->chan_out.mask = 0;
    ctx->frame_count++;
}

static bool send_pending_tx(SrxlMasterCtx *ctx, uint8_t reply_id)
{
    if (ctx->tx_flags.enter_bind) {
        ctx->tx_flags.enter_bind = false;
        uint8_t len = build_bind(ctx, SRXL_BIND_REQ_ENTER, ctx->bind_target_id);
        hal_send(ctx, ctx->tx_pkt.raw, len);
        return true;
    }
    if (ctx->tx_flags.set_bind) {
        ctx->tx_flags.set_bind = false;
        uint8_t len = build_bind(ctx, SRXL_BIND_REQ_SET_BIND, ctx->bind_target_id);
        hal_send(ctx, ctx->tx_pkt.raw, len);
        return true;
    }
    if (ctx->tx_flags.send_vtx) {
        ctx->tx_flags.send_vtx = false;
        uint8_t len = build_vtx(ctx, reply_id);
        hal_send(ctx, ctx->tx_pkt.raw, len);
        return true;
    }
    if (ctx->tx_flags.send_fwd_pgm) {
        ctx->tx_flags.send_fwd_pgm = false;
        uint8_t len = build_fwd_pgm(ctx, reply_id);
        hal_send(ctx, ctx->tx_pkt.raw, len);
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------
 * State machine
 *---------------------------------------------------------------------------*/

static uint32_t time_in_state(SrxlMasterCtx *ctx)
{
    return now_ms(ctx) - ctx->state_entered_ms;
}

static void tick_startup(SrxlMasterCtx *ctx)
{
    if (time_in_state(ctx) < SRXL_MASTER_STARTUP_DELAY_MS)
        return;

    enter_state(ctx, SrxlState_SendHandshake);
    ctx->hs_scan_idx = 0;
    ctx->hs_baud_and = ctx->config.baud_supported;
    ctx->peer_count = 0;
    ctx->peer_priority_sum = 0;
}

static void tick_handshake(SrxlMasterCtx *ctx)
{
    if (ctx->hs_scan_idx < ctx->hs_scan_count) {
        uint8_t dest = ctx->hs_scan_table[ctx->hs_scan_idx++];

        if (dest == ctx->config.device.device_id) {
            if (ctx->hs_scan_idx < ctx->hs_scan_count)
                dest = ctx->hs_scan_table[ctx->hs_scan_idx++];
            else
                dest = 0xFF;
        }

        if (ctx->hs_scan_idx <= ctx->hs_scan_count && dest != 0xFF) {
            send_handshake_to(ctx, dest);
            return;
        }
    }

    /* Scan complete: broadcast, negotiate baud, enter Running. */
    send_handshake_to(ctx, 0xFF);
    ctx->negotiated_baud = ctx->hs_baud_and;
    hal_set_baud(ctx, baud_to_rate(ctx->negotiated_baud));
    enter_state(ctx, SrxlState_Running);

    SrxlMasterEvent evt = {0};
    evt.type = SRXL_MASTER_EVT_HANDSHAKE_COMPLETE;
    evt.handshake.peer_count = ctx->peer_count;
    fire_event(ctx, &evt);
}

static void tick_running(SrxlMasterCtx *ctx)
{
    uint32_t since_tx = time_since_tx(ctx);

    if (since_tx > 0 && since_tx < response_wait_ms(ctx))
        return;

    if (send_pending_tx(ctx, 0))
        return;

    bool should_send = (since_tx >= frame_period_ms(ctx)) ||
                        (ctx->chan_out.mask != 0) ||
                        ctx->chan_out_failsafe;

    if (should_send)
        send_channel(ctx);
}

/*---------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

static void init_ctx(SrxlMasterCtx *ctx, const SrxlMasterConfig *config)
{
    ctx->config = *config;
    ctx->state = SrxlState_ListenOnStartup;
    ctx->negotiated_baud = SRXL_BAUD_115200;
    ctx->hs_baud_and = config->baud_supported;

    memcpy(ctx->hs_scan_table, default_scan_ids, SRXL_MASTER_SCAN_TABLE_SIZE);
    ctx->hs_scan_count = SRXL_MASTER_SCAN_TABLE_SIZE;
    ctx->hs_scan_idx = 0;

    uint32_t t = now_ms(ctx);
    ctx->state_entered_ms = t;
    ctx->last_rx_ms = t;
    ctx->last_tx_ms = t;
}

size_t srxlMasterCtxSize(void)
{
    return sizeof(SrxlMasterCtx);
}

SrxlMasterCtx *srxlMasterInitStatic(uint8_t *buf, size_t buf_size,
                                     const SrxlMasterConfig *config)
{
    if (!buf || !config || buf_size < sizeof(SrxlMasterCtx))
        return NULL;

    SrxlMasterCtx *ctx = (SrxlMasterCtx *)buf;
    memset(ctx, 0, sizeof(*ctx));
    ctx->malloced = false;
    init_ctx(ctx, config);
    return ctx;
}

SrxlMasterCtx *srxlMasterInit(const SrxlMasterConfig *config)
{
    if (!config)
        return NULL;

    SrxlMasterCtx *ctx = (SrxlMasterCtx *)calloc(1, sizeof(SrxlMasterCtx));
    if (!ctx)
        return NULL;

    ctx->malloced = true;
    init_ctx(ctx, config);
    return ctx;
}

void srxlMasterDestroy(SrxlMasterCtx *ctx)
{
    if (ctx && ctx->malloced)
        free(ctx);
}

/*---------------------------------------------------------------------------
 * Core loop
 *---------------------------------------------------------------------------*/

void srxlMasterFeed(SrxlMasterCtx *ctx, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t next = (uint8_t)((ctx->rx_head + 1) % SRXL_MASTER_RX_BUF_SIZE);
        if (next == ctx->rx_tail)
            break;  /* ring full, drop */
        ctx->rx_ring[ctx->rx_head] = data[i];
        ctx->rx_head = next;
    }
}

void srxlMasterTick(SrxlMasterCtx *ctx)
{
    while (try_assemble_frame(ctx)) {
        dispatch_frame(ctx, ctx->frame_buf, ctx->frame_len);
        ctx->frame_len = 0;
    }

    switch (ctx->state) {
    case SrxlState_ListenOnStartup:
        tick_startup(ctx);
        break;
    case SrxlState_SendHandshake:
        tick_handshake(ctx);
        break;
    case SrxlState_Running:
        tick_running(ctx);
        break;
    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Master control API
 *---------------------------------------------------------------------------*/

void srxlMasterSetChannels(SrxlMasterCtx *ctx, const uint16_t *values,
                            uint32_t mask)
{
    ctx->chan_out.mask |= mask;
    for (int i = 0; i < 32; i++) {
        if (mask & (1u << i))
            ctx->chan_out.values[i] = values[i];
    }
}

void srxlMasterSetFailsafe(SrxlMasterCtx *ctx, bool failsafe)
{
    ctx->chan_out_failsafe = failsafe;
}

bool srxlMasterGetTelemetry(SrxlMasterCtx *ctx, uint8_t device_id,
                             SrxlTelemetryData *data_out, uint32_t *age_ms_out)
{
    SrxlMasterPeer *p = find_peer(ctx, device_id);
    if (!p || !p->telem_valid)
        return false;

    memcpy(data_out->raw, p->telem_payload, 16);
    if (age_ms_out)
        *age_ms_out = now_ms(ctx) - p->telem_rx_ms;
    return true;
}

void srxlMasterSetVtx(SrxlMasterCtx *ctx, const SrxlVtxData *vtx)
{
    ctx->vtx_data = *vtx;
    ctx->tx_flags.send_vtx = true;
}

void srxlMasterSendFwdPgm(SrxlMasterCtx *ctx, uint8_t device_id,
                           const uint8_t *data, uint8_t len)
{
    if (len > FWD_PGM_MAX_DATA_SIZE)
        len = FWD_PGM_MAX_DATA_SIZE;
    memcpy(ctx->fwd_pgm_buf, data, len);
    ctx->fwd_pgm_len = len;
    ctx->fwd_pgm_target = device_id;
    ctx->tx_flags.send_fwd_pgm = true;
}

void srxlMasterEnterBind(SrxlMasterCtx *ctx, uint8_t bind_type, bool broadcast)
{
    ctx->bind_info.type = bind_type;
    ctx->bind_target_id = broadcast ? 0xFF : ctx->config.device.device_id;
    ctx->tx_flags.enter_bind = true;
}

void srxlMasterSetBindInfo(SrxlMasterCtx *ctx, const SrxlBindData *data)
{
    ctx->bind_info = *data;
    ctx->tx_flags.set_bind = true;
}

/*---------------------------------------------------------------------------
 * Query
 *---------------------------------------------------------------------------*/

bool srxlMasterIsConnected(const SrxlMasterCtx *ctx)
{
    return ctx->state == SrxlState_Running;
}

uint8_t srxlMasterPeerCount(const SrxlMasterCtx *ctx)
{
    return ctx->peer_count;
}

uint32_t srxlMasterGetBaud(const SrxlMasterCtx *ctx)
{
    return baud_to_rate(ctx->negotiated_baud);
}

const char *srxlMasterGetState(const SrxlMasterCtx *ctx)
{
    switch (ctx->state) {
    case SrxlState_ListenOnStartup: return "STARTUP";
    case SrxlState_SendHandshake:   return "HANDSHAKE";
    case SrxlState_Running:         return "RUNNING";
    default:                        return "UNKNOWN";
    }
}

/*---------------------------------------------------------------------------
 * Event registration
 *---------------------------------------------------------------------------*/

void srxlMasterOnEvent(SrxlMasterCtx *ctx, SrxlMasterEventCb cb, void *user)
{
    ctx->event_cb = cb;
    ctx->event_cb_user = user;
}
