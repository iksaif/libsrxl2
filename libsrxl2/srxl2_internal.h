/*
 * libsrxl2 -- Internal types and context struct
 *
 * NOT for user inclusion. Include srxl2.h instead.
 *
 * MIT License
 */

#ifndef SRXL2_INTERNAL_H
#define SRXL2_INTERNAL_H

#include "srxl2.h"
#include "srxl2_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Internal constants
 *---------------------------------------------------------------------------*/

#define SRXL2_RX_BUF_SIZE      160  /* SPSC ring buffer for ISR->task */

/* Timing (milliseconds) */
#define SRXL2_STARTUP_DELAY_MS      50
#define SRXL2_HANDSHAKE_TIMEOUT_MS  200
#define SRXL2_RUNNING_TIMEOUT_MS    50
#define SRXL2_FRAME_PERIOD_115200   11  /* ms between frames at 115200 */
#define SRXL2_FRAME_PERIOD_400000   6   /* ms between frames at 400000 */
#define SRXL2_RESPONSE_WAIT_115200  4   /* ms to wait for slave reply */
#define SRXL2_RESPONSE_WAIT_400000  2

/* Default handshake scan IDs */
#define SRXL2_SCAN_TABLE_SIZE   10

/*---------------------------------------------------------------------------
 * State machine
 *---------------------------------------------------------------------------*/

typedef enum {
    SRXL2_STATE_STARTUP,    /* Wait 50ms on init */
    SRXL2_STATE_HANDSHAKE,  /* Discovering devices or waiting */
    SRXL2_STATE_RUNNING,    /* Normal operation */
} srxl2_state_t;

/*---------------------------------------------------------------------------
 * Peer entry
 *---------------------------------------------------------------------------*/

typedef struct {
    uint8_t  device_id;
    uint8_t  priority;
    uint8_t  info;
    uint8_t  rfu;
    uint32_t uid;
    uint16_t telem_age;         /* frames since last poll */
    uint8_t  telem_payload[16]; /* last received telemetry */
    uint32_t telem_rx_ms;       /* timestamp of last telem reception */
    bool     telem_valid;       /* true if payload has been received */
} srxl2_peer_t;

/*---------------------------------------------------------------------------
 * TX pending flags
 *---------------------------------------------------------------------------*/

typedef struct {
    bool enter_bind;
    bool set_bind;
    bool report_bind;
    bool send_vtx;
    bool send_fwd_pgm;
} srxl2_tx_flags_t;

/*---------------------------------------------------------------------------
 * Context struct
 *---------------------------------------------------------------------------*/

struct srxl2_ctx {
    /* Configuration (copy from init) */
    srxl2_config_t  config;

    /* State machine */
    srxl2_state_t   state;
    uint32_t        last_rx_ms;
    uint32_t        last_tx_ms;
    uint32_t        state_entered_ms;

    /* Peers */
    srxl2_peer_t    peers[SRXL2_MAX_PEERS];
    uint8_t         peer_count;
    uint16_t        peer_priority_sum;

    /* Negotiated baud */
    uint8_t         negotiated_baud;  /* SRXL2_BAUD_* */
    bool            is_master;

    /* Channel data (master outgoing) */
    srxl2_channel_data_t chan_out;

    /* Channel data (slave incoming) */
    srxl2_channel_data_t chan_in;
    bool            chan_in_valid;

    /* Telemetry (slave: what to send when polled) */
    uint8_t         telem_out[16];
    bool            telem_out_valid;

    /* Handshake scan (master) */
    uint8_t         hs_scan_table[SRXL2_SCAN_TABLE_SIZE];
    uint8_t         hs_scan_count;
    uint8_t         hs_scan_idx;
    uint8_t         hs_baud_and;  /* baud rates ANDed from all peers */

    /* Bind state */
    srxl2_bind_data_t bind_info;
    uint8_t         bind_target_id;
    bool            bind_broadcast;

    /* VTX state */
    srxl2_vtx_data_t vtx_data;

    /* Forward programming */
    uint8_t         fwd_pgm_buf[SRXL2_FWD_PGM_MAX];
    uint8_t         fwd_pgm_len;
    uint8_t         fwd_pgm_target;

    /* TX pending flags */
    srxl2_tx_flags_t tx_flags;

    /* RX ring buffer (ISR writes, tick reads) -- SPSC lock-free */
    uint8_t         rx_ring[SRXL2_RX_BUF_SIZE];
    volatile uint8_t rx_head;  /* written by producer (ISR) */
    uint8_t         rx_tail;   /* read by consumer (tick) */

    /* Frame assembly buffer (used during tick) */
    uint8_t         frame_buf[SRXL2_MAX_PACKET_SIZE];
    uint8_t         frame_len;
    bool            frame_synced;

    /* TX scratch buffer */
    uint8_t         tx_buf[SRXL2_MAX_PACKET_SIZE];

    /* Telemetry scheduling (master) */
    uint8_t         telem_poll_idx;

    /* Event callback */
    srxl2_event_cb_t event_cb;
    void            *event_cb_user;

    /* Master receiver tracking */
    uint8_t         master_rcvr_id;

    /* Frame counter */
    uint16_t        frame_count;

    /* Reply state (slave: should we reply this tick?) */
    bool            reply_pending;
    uint8_t         reply_to_id;  /* who asked us to reply */

    /* Was this context allocated with malloc? */
    bool            malloced;
};

#ifdef __cplusplus
}
#endif

#endif /* SRXL2_INTERNAL_H */
