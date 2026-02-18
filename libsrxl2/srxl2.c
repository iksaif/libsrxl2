/*
 * libsrxl2 -- State machine
 *
 * Context-based SRXL2 protocol stack supporting both master and slave roles.
 * No globals, no packed struct aliasing, explicit byte-order conversion.
 *
 * MIT License
 */

#include "srxl2_internal.h"
#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Default handshake scan table (master)
 *---------------------------------------------------------------------------*/

static const uint8_t default_scan_ids[SRXL2_SCAN_TABLE_SIZE] = {
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
 * Internal helpers
 *---------------------------------------------------------------------------*/

static uint32_t now_ms(srxl2_ctx_t *ctx)
{
    return ctx->config.hal.time_ms(ctx->config.hal.user);
}

static void hal_send(srxl2_ctx_t *ctx, const uint8_t *buf, uint8_t len)
{
    ctx->config.hal.uart_send(ctx->config.hal.user, buf, len);
    ctx->last_tx_ms = now_ms(ctx);
}

static void hal_set_baud(srxl2_ctx_t *ctx, uint32_t baud)
{
    ctx->config.hal.uart_set_baud(ctx->config.hal.user, baud);
}

static uint32_t baud_to_rate(uint8_t baud_flag)
{
    return (baud_flag & SRXL2_BAUD_400000) ? 400000 : 115200;
}

static uint8_t frame_period_ms(srxl2_ctx_t *ctx)
{
    return (ctx->negotiated_baud & SRXL2_BAUD_400000)
        ? SRXL2_FRAME_PERIOD_400000
        : SRXL2_FRAME_PERIOD_115200;
}

static uint8_t response_wait_ms(srxl2_ctx_t *ctx)
{
    return (ctx->negotiated_baud & SRXL2_BAUD_400000)
        ? SRXL2_RESPONSE_WAIT_400000
        : SRXL2_RESPONSE_WAIT_115200;
}

static void fire_event(srxl2_ctx_t *ctx, const srxl2_event_t *evt)
{
    if (ctx->event_cb)
        ctx->event_cb(ctx, evt, ctx->event_cb_user);
}

static void enter_state(srxl2_ctx_t *ctx, srxl2_state_t state)
{
    ctx->state = state;
    ctx->state_entered_ms = now_ms(ctx);
}

static uint32_t time_in_state(srxl2_ctx_t *ctx)
{
    return now_ms(ctx) - ctx->state_entered_ms;
}

static uint32_t time_since_rx(srxl2_ctx_t *ctx)
{
    return now_ms(ctx) - ctx->last_rx_ms;
}

static uint32_t time_since_tx(srxl2_ctx_t *ctx)
{
    return now_ms(ctx) - ctx->last_tx_ms;
}

/*---------------------------------------------------------------------------
 * Peer management
 *---------------------------------------------------------------------------*/

static srxl2_peer_t *find_peer(srxl2_ctx_t *ctx, uint8_t device_id)
{
    for (uint8_t i = 0; i < ctx->peer_count; i++) {
        if (ctx->peers[i].device_id == device_id)
            return &ctx->peers[i];
    }
    return NULL;
}

static srxl2_peer_t *add_peer(srxl2_ctx_t *ctx, uint8_t device_id,
                                uint8_t priority, uint8_t info, uint32_t uid)
{
    srxl2_peer_t *p = find_peer(ctx, device_id);
    if (p) {
        /* Update existing peer */
        p->priority = priority;
        p->info = info;
        p->uid = uid;
        return p;
    }

    if (ctx->peer_count >= SRXL2_MAX_PEERS)
        return NULL;

    p = &ctx->peers[ctx->peer_count++];
    memset(p, 0, sizeof(*p));
    p->device_id = device_id;
    p->priority = priority;
    p->info = info;
    p->uid = uid;

    /* Recompute priority sum */
    ctx->peer_priority_sum = 0;
    for (uint8_t i = 0; i < ctx->peer_count; i++)
        ctx->peer_priority_sum += ctx->peers[i].priority;

    return p;
}

/*---------------------------------------------------------------------------
 * Telemetry scheduling (master): priority * age scoring
 *---------------------------------------------------------------------------*/

static uint8_t select_telem_device(srxl2_ctx_t *ctx)
{
    if (ctx->peer_count == 0 || ctx->peer_priority_sum == 0)
        return 0;  /* no valid device */

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

static void age_telem_counters(srxl2_ctx_t *ctx)
{
    for (uint8_t i = 0; i < ctx->peer_count; i++) {
        if (ctx->peers[i].telem_age < 0xFFFF)
            ctx->peers[i].telem_age++;
    }
}

/*---------------------------------------------------------------------------
 * Ring buffer helpers (SPSC: ISR produces, tick consumes)
 *---------------------------------------------------------------------------*/

static uint8_t ring_used(const srxl2_ctx_t *ctx)
{
    return (uint8_t)((ctx->rx_head - ctx->rx_tail) % SRXL2_RX_BUF_SIZE);
}

static uint8_t ring_pop(srxl2_ctx_t *ctx)
{
    uint8_t val = ctx->rx_ring[ctx->rx_tail];
    ctx->rx_tail = (uint8_t)((ctx->rx_tail + 1) % SRXL2_RX_BUF_SIZE);
    return val;
}

/*---------------------------------------------------------------------------
 * Frame assembly: pull bytes from ring, assemble complete packets
 * Returns true if a complete packet is in frame_buf[0..frame_len-1]
 *---------------------------------------------------------------------------*/

static bool try_assemble_frame(srxl2_ctx_t *ctx)
{
    while (ring_used(ctx) > 0) {
        uint8_t byte = ring_pop(ctx);

        if (!ctx->frame_synced) {
            if (byte == SRXL2_MAGIC) {
                ctx->frame_buf[0] = byte;
                ctx->frame_len = 1;
                ctx->frame_synced = true;
            }
            continue;
        }

        if (ctx->frame_len < SRXL2_MAX_PACKET_SIZE) {
            ctx->frame_buf[ctx->frame_len++] = byte;
        } else {
            /* Overflow -- reset */
            ctx->frame_synced = false;
            ctx->frame_len = 0;
            continue;
        }

        /* Once we have at least 3 bytes, we know the expected length */
        if (ctx->frame_len >= 3) {
            uint8_t expected = ctx->frame_buf[2];
            if (expected < 5 || expected > SRXL2_MAX_PACKET_SIZE) {
                /* Invalid length -- resync */
                ctx->frame_synced = false;
                ctx->frame_len = 0;
                continue;
            }
            if (ctx->frame_len >= expected) {
                /* Complete packet */
                ctx->frame_synced = false;
                return true;
            }
        }
    }
    return false;
}

/* Forward declaration (needed by on_handshake for late-join broadcast) */
static void master_send_handshake(srxl2_ctx_t *ctx, uint8_t dest_id);

/*---------------------------------------------------------------------------
 * Packet dispatch (called from tick for each parsed packet)
 *---------------------------------------------------------------------------*/

static void on_handshake(srxl2_ctx_t *ctx, const srxl2_pkt_handshake_t *hs)
{
    ctx->last_rx_ms = now_ms(ctx);

    if (ctx->is_master) {
        /* Master: register peer and AND baud */
        if (hs->src_id != ctx->config.device.device_id) {
            add_peer(ctx, hs->src_id, hs->priority, hs->info, hs->uid);
            ctx->hs_baud_and &= hs->baud_supported;
        }

        /* Master election: if we see a lower device ID, yield */
        if (hs->src_id < ctx->config.device.device_id) {
            ctx->is_master = false;
        }

        /* Late join: unprompted handshake (dest=0) while running --
           send broadcast so the new device transitions to Running */
        if (hs->dest_id == 0x00 && ctx->state == SRXL2_STATE_RUNNING) {
            master_send_handshake(ctx, 0xFF);
        }
    } else {
        /* Slave: reply if addressed to us */
        if (hs->dest_id == ctx->config.device.device_id) {
            uint8_t len = srxl2_pkt_handshake(
                ctx->tx_buf,
                ctx->config.device.device_id,
                hs->src_id,
                ctx->config.device.priority,
                ctx->config.baud_supported,
                ctx->config.device.info,
                ctx->config.device.uid);
            hal_send(ctx, ctx->tx_buf, len);
            ctx->master_rcvr_id = hs->src_id;
        }

        /* Broadcast: switch baud and enter running */
        if (hs->dest_id == 0xFF) {
            ctx->negotiated_baud = hs->baud_supported;
            hal_set_baud(ctx, baud_to_rate(ctx->negotiated_baud));
            ctx->master_rcvr_id = hs->src_id;
            enter_state(ctx, SRXL2_STATE_RUNNING);

            srxl2_event_t evt = {0};
            evt.type = SRXL2_EVT_HANDSHAKE_COMPLETE;
            evt.handshake.peer_count = ctx->peer_count;
            fire_event(ctx, &evt);
        }
    }
}

static void on_channel(srxl2_ctx_t *ctx, const srxl2_pkt_control_t *ctrl)
{
    ctx->last_rx_ms = now_ms(ctx);

    /* Update incoming channel data */
    ctx->chan_in.mask = ctrl->channel.mask;
    ctx->chan_in.rssi = ctrl->channel.rssi;
    ctx->chan_in.frame_losses = ctrl->channel.frame_losses;
    ctx->chan_in.is_failsafe = (ctrl->cmd == SRXL2_CMD_CHANNEL_FS);
    memcpy(ctx->chan_in.values, ctrl->channel.values,
           sizeof(ctx->chan_in.values));
    ctx->chan_in_valid = true;

    /* Fire channel event */
    srxl2_event_t evt = {0};
    evt.type = SRXL2_EVT_CHANNEL;
    evt.channel.data = &ctx->chan_in;
    fire_event(ctx, &evt);

    /* Slave: check if we should reply */
    if (!ctx->is_master &&
        ctrl->reply_id == ctx->config.device.device_id) {
        ctx->reply_pending = true;
        ctx->reply_to_id = ctrl->reply_id;
    }
}

static void on_telemetry(srxl2_ctx_t *ctx, const srxl2_pkt_telemetry_t *tm)
{
    ctx->last_rx_ms = now_ms(ctx);

    /* dest=0xFF means device wants re-handshake */
    if (tm->dest_id == 0xFF) {
        enter_state(ctx, SRXL2_STATE_HANDSHAKE);
        ctx->hs_scan_idx = 0;
        ctx->hs_baud_and = ctx->config.baud_supported;
        return;
    }

    /* Store in peer telemetry */
    /* Find peer by looking at who this telemetry is "from" -- it's the device
       that was polled. We identify it by the previous reply_id we sent. */
    /* For simplicity, iterate peers and find the one whose telem_age was
       just reset (the one we polled). Actually, the telemetry packet dest
       field tells us who it's addressed TO (the master), not FROM.
       The FROM device is identified by the fact that it's the device we
       polled last. Use telem_poll_idx. */
    if (ctx->is_master && ctx->peer_count > 0) {
        uint8_t idx = ctx->telem_poll_idx;
        if (idx < ctx->peer_count) {
            srxl2_peer_t *p = &ctx->peers[idx];
            memcpy(p->telem_payload, tm->payload, 16);
            p->telem_rx_ms = now_ms(ctx);
            p->telem_valid = true;

            srxl2_event_t evt = {0};
            evt.type = SRXL2_EVT_TELEMETRY;
            evt.telemetry.device_id = p->device_id;
            evt.telemetry.payload = p->telem_payload;
            fire_event(ctx, &evt);
        }
    }
}

static void on_bind(srxl2_ctx_t *ctx, const srxl2_pkt_bind_t *bd)
{
    ctx->last_rx_ms = now_ms(ctx);

    srxl2_event_t evt = {0};
    evt.type = SRXL2_EVT_BIND;
    evt.bind.request = bd->request;
    evt.bind.device_id = bd->device_id;
    evt.bind.data = &bd->data;
    fire_event(ctx, &evt);
}

static void on_vtx(srxl2_ctx_t *ctx, const srxl2_pkt_control_t *ctrl)
{
    ctx->last_rx_ms = now_ms(ctx);
    ctx->vtx_data = ctrl->vtx;

    srxl2_event_t evt = {0};
    evt.type = SRXL2_EVT_VTX;
    evt.vtx.data = &ctx->vtx_data;
    fire_event(ctx, &evt);
}

static void on_fwd_pgm(srxl2_ctx_t *ctx, const srxl2_pkt_control_t *ctrl)
{
    ctx->last_rx_ms = now_ms(ctx);

    srxl2_event_t evt = {0};
    evt.type = SRXL2_EVT_FWD_PGM;
    evt.fwd_pgm.data = ctrl->fwd_pgm.data;
    evt.fwd_pgm.len = ctrl->fwd_pgm.len;
    fire_event(ctx, &evt);
}

static void dispatch_packet(srxl2_ctx_t *ctx, const srxl2_decoded_pkt_t *pkt)
{
    switch (pkt->packet_type) {
    case SRXL2_PKT_HANDSHAKE:
        on_handshake(ctx, &pkt->handshake);
        break;
    case SRXL2_PKT_CONTROL:
        switch (pkt->control.cmd) {
        case SRXL2_CMD_CHANNEL:
        case SRXL2_CMD_CHANNEL_FS:
            on_channel(ctx, &pkt->control);
            break;
        case SRXL2_CMD_VTX:
            on_vtx(ctx, &pkt->control);
            break;
        case SRXL2_CMD_FWDPGM:
            on_fwd_pgm(ctx, &pkt->control);
            break;
        }
        break;
    case SRXL2_PKT_TELEMETRY:
        on_telemetry(ctx, &pkt->telemetry);
        break;
    case SRXL2_PKT_BIND:
        on_bind(ctx, &pkt->bind);
        break;
    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * Master: send channel data packet
 *---------------------------------------------------------------------------*/

static void master_send_channel(srxl2_ctx_t *ctx)
{
    /* Select device to request telemetry from */
    uint8_t poll_idx = select_telem_device(ctx);
    uint8_t reply_id = 0;

    if (poll_idx < ctx->peer_count) {
        reply_id = ctx->peers[poll_idx].device_id;
        ctx->telem_poll_idx = poll_idx;
    }

    uint8_t cmd = ctx->chan_out.is_failsafe
        ? SRXL2_CMD_CHANNEL_FS
        : SRXL2_CMD_CHANNEL;

    /* For failsafe, reply_id is 0 (no device should reply) */
    if (ctx->chan_out.is_failsafe)
        reply_id = 0;

    uint8_t len = srxl2_pkt_channel(
        ctx->tx_buf, cmd, reply_id,
        ctx->chan_out.rssi,
        ctx->chan_out.frame_losses,
        ctx->chan_out.mask,
        ctx->chan_out.values);
    hal_send(ctx, ctx->tx_buf, len);

    /* Age counters, reset polled device */
    age_telem_counters(ctx);
    if (poll_idx < ctx->peer_count)
        ctx->peers[poll_idx].telem_age = 0;

    /* Clear channel mask after send */
    ctx->chan_out.mask = 0;
    ctx->frame_count++;
}

/*---------------------------------------------------------------------------
 * Master: send handshake to a specific device
 *---------------------------------------------------------------------------*/

static void master_send_handshake(srxl2_ctx_t *ctx, uint8_t dest_id)
{
    uint8_t len = srxl2_pkt_handshake(
        ctx->tx_buf,
        ctx->config.device.device_id,
        dest_id,
        ctx->config.device.priority,
        ctx->hs_baud_and,
        ctx->config.device.info,
        ctx->config.device.uid);
    hal_send(ctx, ctx->tx_buf, len);
}

/*---------------------------------------------------------------------------
 * Slave: send telemetry reply
 *---------------------------------------------------------------------------*/

static void slave_send_telemetry(srxl2_ctx_t *ctx)
{
    uint8_t dest = ctx->master_rcvr_id;
    uint8_t len = srxl2_pkt_telemetry(ctx->tx_buf, dest, ctx->telem_out);
    hal_send(ctx, ctx->tx_buf, len);
}

/*---------------------------------------------------------------------------
 * Send pending bind/vtx/fwdpgm if flagged
 *---------------------------------------------------------------------------*/

static bool send_pending_tx(srxl2_ctx_t *ctx, uint8_t reply_id)
{
    if (ctx->tx_flags.enter_bind) {
        ctx->tx_flags.enter_bind = false;
        uint8_t len = srxl2_pkt_bind(ctx->tx_buf,
            SRXL2_BIND_REQ_ENTER, ctx->bind_target_id, &ctx->bind_info);
        hal_send(ctx, ctx->tx_buf, len);
        return true;
    }
    if (ctx->tx_flags.set_bind) {
        ctx->tx_flags.set_bind = false;
        uint8_t len = srxl2_pkt_bind(ctx->tx_buf,
            SRXL2_BIND_REQ_SET_BIND, ctx->bind_target_id, &ctx->bind_info);
        hal_send(ctx, ctx->tx_buf, len);
        return true;
    }
    if (ctx->tx_flags.send_vtx) {
        ctx->tx_flags.send_vtx = false;
        uint8_t len = srxl2_pkt_vtx(ctx->tx_buf, reply_id, &ctx->vtx_data);
        hal_send(ctx, ctx->tx_buf, len);
        return true;
    }
    if (ctx->tx_flags.send_fwd_pgm) {
        ctx->tx_flags.send_fwd_pgm = false;
        uint8_t len = srxl2_pkt_fwd_pgm(ctx->tx_buf, reply_id, 0,
            ctx->fwd_pgm_buf, ctx->fwd_pgm_len);
        hal_send(ctx, ctx->tx_buf, len);
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------
 * State machine: STARTUP
 *---------------------------------------------------------------------------*/

static void tick_startup(srxl2_ctx_t *ctx)
{
    if (time_in_state(ctx) < SRXL2_STARTUP_DELAY_MS)
        return;

    if (ctx->config.role == SRXL2_ROLE_MASTER) {
        ctx->is_master = true;
        enter_state(ctx, SRXL2_STATE_HANDSHAKE);
        ctx->hs_scan_idx = 0;
        ctx->hs_baud_and = ctx->config.baud_supported;
        ctx->peer_count = 0;
        ctx->peer_priority_sum = 0;
    } else {
        /* Slave: unit_id=0 means lower nibble of device_id is 0 */
        uint8_t unit_id = ctx->config.device.device_id & 0x0F;
        if (unit_id == 0) {
            /* Send unprompted handshake */
            uint8_t len = srxl2_pkt_handshake(
                ctx->tx_buf,
                ctx->config.device.device_id,
                0x00,
                ctx->config.device.priority,
                ctx->config.baud_supported,
                ctx->config.device.info,
                ctx->config.device.uid);
            hal_send(ctx, ctx->tx_buf, len);
        }
        enter_state(ctx, SRXL2_STATE_HANDSHAKE);
    }
}

/*---------------------------------------------------------------------------
 * State machine: HANDSHAKE (master)
 *---------------------------------------------------------------------------*/

static void tick_handshake_master(srxl2_ctx_t *ctx)
{
    /* Send next handshake from scan table */
    if (ctx->hs_scan_idx < ctx->hs_scan_count) {
        uint8_t dest = ctx->hs_scan_table[ctx->hs_scan_idx++];

        /* Don't scan our own ID */
        if (dest == ctx->config.device.device_id) {
            if (ctx->hs_scan_idx < ctx->hs_scan_count)
                dest = ctx->hs_scan_table[ctx->hs_scan_idx++];
            else
                dest = 0xFF; /* go to broadcast */
        }

        if (ctx->hs_scan_idx <= ctx->hs_scan_count && dest != 0xFF) {
            master_send_handshake(ctx, dest);
            return;
        }
    }

    /* Scan complete: send broadcast handshake, switch baud, enter running */
    master_send_handshake(ctx, 0xFF);
    ctx->negotiated_baud = ctx->hs_baud_and;
    hal_set_baud(ctx, baud_to_rate(ctx->negotiated_baud));
    enter_state(ctx, SRXL2_STATE_RUNNING);

    srxl2_event_t evt = {0};
    evt.type = SRXL2_EVT_HANDSHAKE_COMPLETE;
    evt.handshake.peer_count = ctx->peer_count;
    fire_event(ctx, &evt);
}

/*---------------------------------------------------------------------------
 * State machine: HANDSHAKE (slave)
 *---------------------------------------------------------------------------*/

static void tick_handshake_slave(srxl2_ctx_t *ctx)
{
    /* Timeout: go back to startup */
    if (time_in_state(ctx) >= SRXL2_HANDSHAKE_TIMEOUT_MS) {
        ctx->negotiated_baud = SRXL2_BAUD_115200;
        hal_set_baud(ctx, 115200);
        enter_state(ctx, SRXL2_STATE_STARTUP);
    }
    /* Packet handling is done in dispatch_packet / on_handshake */
}

/*---------------------------------------------------------------------------
 * State machine: RUNNING (master)
 *---------------------------------------------------------------------------*/

static void tick_running_master(srxl2_ctx_t *ctx)
{
    uint32_t since_tx = time_since_tx(ctx);

    /* Response wait: don't send too early after previous frame */
    if (since_tx > 0 && since_tx < response_wait_ms(ctx))
        return;

    /* Check pending TX flags first */
    if (send_pending_tx(ctx, 0))
        return;

    /* Time to send channel data? */
    bool should_send = (since_tx >= frame_period_ms(ctx)) ||
                       (ctx->chan_out.mask != 0) ||
                       ctx->chan_out.is_failsafe;

    if (should_send) {
        master_send_channel(ctx);
    }
}

/*---------------------------------------------------------------------------
 * State machine: RUNNING (slave)
 *---------------------------------------------------------------------------*/

static void tick_running_slave(srxl2_ctx_t *ctx)
{
    /* Timeout: reset baud and go to startup */
    if (time_since_rx(ctx) >= SRXL2_RUNNING_TIMEOUT_MS) {
        ctx->negotiated_baud = SRXL2_BAUD_115200;
        hal_set_baud(ctx, 115200);
        enter_state(ctx, SRXL2_STATE_STARTUP);

        srxl2_event_t evt = {0};
        evt.type = SRXL2_EVT_TIMEOUT;
        fire_event(ctx, &evt);
        return;
    }

    /* If polled, send reply */
    if (ctx->reply_pending) {
        ctx->reply_pending = false;

        /* Check pending TX flags first */
        if (!send_pending_tx(ctx, ctx->reply_to_id)) {
            slave_send_telemetry(ctx);
        }
    }
}

/*---------------------------------------------------------------------------
 * Init context (shared between static and dynamic)
 *---------------------------------------------------------------------------*/

static void init_ctx(srxl2_ctx_t *ctx, const srxl2_config_t *config)
{
    ctx->config = *config;
    ctx->state = SRXL2_STATE_STARTUP;
    ctx->is_master = (config->role == SRXL2_ROLE_MASTER);
    ctx->negotiated_baud = SRXL2_BAUD_115200;
    ctx->hs_baud_and = config->baud_supported;

    /* Populate default handshake scan table */
    memcpy(ctx->hs_scan_table, default_scan_ids, SRXL2_SCAN_TABLE_SIZE);
    ctx->hs_scan_count = SRXL2_SCAN_TABLE_SIZE;
    ctx->hs_scan_idx = 0;

    uint32_t t = now_ms(ctx);
    ctx->state_entered_ms = t;
    ctx->last_rx_ms = t;
    ctx->last_tx_ms = t;
}

/*---------------------------------------------------------------------------
 * Public API: Lifecycle
 *---------------------------------------------------------------------------*/

size_t srxl2_ctx_size(void)
{
    return sizeof(srxl2_ctx_t);
}

srxl2_ctx_t *srxl2_init_static(uint8_t *buf, size_t buf_size,
                                const srxl2_config_t *config)
{
    if (!buf || !config || buf_size < sizeof(srxl2_ctx_t))
        return NULL;

    srxl2_ctx_t *ctx = (srxl2_ctx_t *)buf;
    memset(ctx, 0, sizeof(*ctx));
    ctx->malloced = false;
    init_ctx(ctx, config);
    return ctx;
}

srxl2_ctx_t *srxl2_init(const srxl2_config_t *config)
{
    if (!config)
        return NULL;

    srxl2_ctx_t *ctx = (srxl2_ctx_t *)calloc(1, sizeof(srxl2_ctx_t));
    if (!ctx)
        return NULL;

    ctx->malloced = true;
    init_ctx(ctx, config);
    return ctx;
}

void srxl2_destroy(srxl2_ctx_t *ctx)
{
    if (ctx && ctx->malloced)
        free(ctx);
}

/*---------------------------------------------------------------------------
 * Public API: Core loop
 *---------------------------------------------------------------------------*/

void srxl2_feed(srxl2_ctx_t *ctx, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t next = (uint8_t)((ctx->rx_head + 1) % SRXL2_RX_BUF_SIZE);
        if (next == ctx->rx_tail)
            break;  /* ring full, drop */
        ctx->rx_ring[ctx->rx_head] = data[i];
        ctx->rx_head = next;
    }
}

void srxl2_tick(srxl2_ctx_t *ctx)
{
    /* Drain ring buffer: assemble and dispatch complete packets */
    while (try_assemble_frame(ctx)) {
        srxl2_decoded_pkt_t pkt;
        if (srxl2_pkt_parse(ctx->frame_buf, ctx->frame_len, &pkt) ==
            SRXL2_PARSE_OK) {
            dispatch_packet(ctx, &pkt);
        }
        ctx->frame_len = 0;
    }

    /* Run state machine */
    switch (ctx->state) {
    case SRXL2_STATE_STARTUP:
        tick_startup(ctx);
        break;
    case SRXL2_STATE_HANDSHAKE:
        if (ctx->is_master)
            tick_handshake_master(ctx);
        else
            tick_handshake_slave(ctx);
        break;
    case SRXL2_STATE_RUNNING:
        if (ctx->is_master)
            tick_running_master(ctx);
        else
            tick_running_slave(ctx);
        break;
    }
}

/*---------------------------------------------------------------------------
 * Public API: Master
 *---------------------------------------------------------------------------*/

void srxl2_set_channels(srxl2_ctx_t *ctx, const uint16_t *values,
                         uint32_t mask)
{
    ctx->chan_out.mask |= mask;
    for (int i = 0; i < 32; i++) {
        if (mask & (1u << i))
            ctx->chan_out.values[i] = values[i];
    }
}

void srxl2_set_failsafe(srxl2_ctx_t *ctx, bool failsafe)
{
    ctx->chan_out.is_failsafe = failsafe;
}

bool srxl2_get_telemetry(srxl2_ctx_t *ctx, uint8_t device_id,
                          uint8_t payload_out[16], uint32_t *age_ms_out)
{
    srxl2_peer_t *p = find_peer(ctx, device_id);
    if (!p || !p->telem_valid)
        return false;

    memcpy(payload_out, p->telem_payload, 16);
    if (age_ms_out)
        *age_ms_out = now_ms(ctx) - p->telem_rx_ms;
    return true;
}

void srxl2_set_vtx(srxl2_ctx_t *ctx, const srxl2_vtx_data_t *vtx)
{
    ctx->vtx_data = *vtx;
    ctx->tx_flags.send_vtx = true;
}

void srxl2_send_fwd_pgm(srxl2_ctx_t *ctx, uint8_t device_id,
                          const uint8_t *data, uint8_t len)
{
    if (len > SRXL2_FWD_PGM_MAX)
        len = SRXL2_FWD_PGM_MAX;
    memcpy(ctx->fwd_pgm_buf, data, len);
    ctx->fwd_pgm_len = len;
    ctx->fwd_pgm_target = device_id;
    ctx->tx_flags.send_fwd_pgm = true;
}

void srxl2_enter_bind(srxl2_ctx_t *ctx, uint8_t bind_type, bool broadcast)
{
    ctx->bind_info.type = bind_type;
    ctx->bind_broadcast = broadcast;
    ctx->bind_target_id = broadcast ? 0xFF : ctx->master_rcvr_id;
    ctx->tx_flags.enter_bind = true;
}

void srxl2_set_bind_info(srxl2_ctx_t *ctx, const srxl2_bind_data_t *data)
{
    ctx->bind_info = *data;
    ctx->tx_flags.set_bind = true;
}

/*---------------------------------------------------------------------------
 * Public API: Slave
 *---------------------------------------------------------------------------*/

bool srxl2_get_channels(srxl2_ctx_t *ctx, srxl2_channel_data_t *data_out)
{
    if (!ctx->chan_in_valid)
        return false;
    *data_out = ctx->chan_in;
    return true;
}

void srxl2_set_telemetry(srxl2_ctx_t *ctx, const uint8_t payload[16])
{
    memcpy(ctx->telem_out, payload, 16);
    ctx->telem_out_valid = true;
}

/*---------------------------------------------------------------------------
 * Public API: Query
 *---------------------------------------------------------------------------*/

bool srxl2_is_connected(const srxl2_ctx_t *ctx)
{
    return ctx->state == SRXL2_STATE_RUNNING;
}

uint8_t srxl2_peer_count(const srxl2_ctx_t *ctx)
{
    return ctx->peer_count;
}

uint32_t srxl2_get_baud(const srxl2_ctx_t *ctx)
{
    return baud_to_rate(ctx->negotiated_baud);
}

const char *srxl2_get_state(const srxl2_ctx_t *ctx)
{
    switch (ctx->state) {
    case SRXL2_STATE_STARTUP:    return "STARTUP";
    case SRXL2_STATE_HANDSHAKE:  return "HANDSHAKE";
    case SRXL2_STATE_RUNNING:    return "RUNNING";
    default:                     return "UNKNOWN";
    }
}

/*---------------------------------------------------------------------------
 * Public API: Event registration
 *---------------------------------------------------------------------------*/

void srxl2_on_event(srxl2_ctx_t *ctx, srxl2_event_cb_t cb, void *user)
{
    ctx->event_cb = cb;
    ctx->event_cb_user = user;
}
