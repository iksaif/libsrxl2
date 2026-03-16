/*
 * libsrxl2 -- Modern SRXL2 Protocol Stack
 *
 * Public API. This is the only header users include.
 * Context-based, RTOS-friendly, fully portable C11 library supporting
 * both master and slave roles.
 *
 * MIT License
 */

#ifndef SRXL2_H
#define SRXL2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Compile-time configuration
 *---------------------------------------------------------------------------*/

#ifndef SRXL2_MAX_PEERS
#define SRXL2_MAX_PEERS 16
#endif

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

/* Baud rate flags (bitmask, ANDed during handshake negotiation) */
#define SRXL2_BAUD_115200       0x00
#define SRXL2_BAUD_400000       0x01

/* Device info bits (handshake info byte) */
#define SRXL2_DEVINFO_NO_RF             0x00
#define SRXL2_DEVINFO_TELEM_TX_ENABLED  0x01
#define SRXL2_DEVINFO_TELEM_FULL_RANGE  0x02
#define SRXL2_DEVINFO_FWD_PROG_SUPPORT  0x04

/* Device type upper nibbles */
#define SRXL2_DEVTYPE_NONE              0x00
#define SRXL2_DEVTYPE_REMOTE_RX         0x10
#define SRXL2_DEVTYPE_RECEIVER          0x20
#define SRXL2_DEVTYPE_FC                0x30
#define SRXL2_DEVTYPE_ESC               0x40
#define SRXL2_DEVTYPE_SERVO1            0x60
#define SRXL2_DEVTYPE_SERVO2            0x70
#define SRXL2_DEVTYPE_VTX               0x80
#define SRXL2_DEVTYPE_EXT_RF            0x90
#define SRXL2_DEVTYPE_REMOTE_ID         0xA0
#define SRXL2_DEVTYPE_SENSOR            0xB0
#define SRXL2_DEVTYPE_BROADCAST         0xF0

/* Packet type IDs */
#define SRXL2_PKT_HANDSHAKE     0x21
#define SRXL2_PKT_BIND          0x41
#define SRXL2_PKT_RSSI          0x55
#define SRXL2_PKT_TELEMETRY     0x80
#define SRXL2_PKT_CONTROL       0xCD

/* Control sub-commands */
#define SRXL2_CMD_CHANNEL       0x00
#define SRXL2_CMD_CHANNEL_FS    0x01
#define SRXL2_CMD_VTX           0x02
#define SRXL2_CMD_FWDPGM        0x03

/* Bind request types */
#define SRXL2_BIND_REQ_ENTER        0xEB
#define SRXL2_BIND_REQ_STATUS       0xB5
#define SRXL2_BIND_REQ_BOUND_DATA   0xDB
#define SRXL2_BIND_REQ_SET_BIND     0x5B

/* Wire magic */
#define SRXL2_MAGIC             0xA6

/*---------------------------------------------------------------------------
 * Inline ID helpers
 *---------------------------------------------------------------------------*/

static inline uint8_t srxl2_device_type(uint8_t device_id)
{
    return (device_id >> 4) & 0x0F;
}

static inline uint8_t srxl2_unit_id(uint8_t device_id)
{
    return device_id & 0x0F;
}

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/* Opaque context -- one per bus */
typedef struct srxl2_ctx srxl2_ctx_t;

/* Role */
typedef enum {
    SRXL2_ROLE_MASTER,
    SRXL2_ROLE_SLAVE,
} srxl2_role_t;

/* Platform callbacks (function pointers, set at init) */
typedef struct {
    void     (*uart_send)(void *user, const uint8_t *buf, uint8_t len);
    void     (*uart_set_baud)(void *user, uint32_t baud);
    uint32_t (*time_ms)(void *user);
    void     *user;
} srxl2_hal_t;

/* Device descriptor */
typedef struct {
    uint8_t  device_id;     /* e.g. 0x10 for RemoteRx master */
    uint8_t  priority;      /* telemetry priority (1-100) */
    uint8_t  info;          /* SRXL2_DEVINFO_* bits */
    uint32_t uid;           /* unique ID for collision detection */
} srxl2_device_t;

/* Init configuration */
typedef struct {
    srxl2_role_t   role;
    srxl2_device_t device;
    srxl2_hal_t    hal;
    uint8_t        baud_supported;  /* SRXL2_BAUD_* flags */
} srxl2_config_t;

/* Channel data */
typedef struct {
    uint16_t values[32];
    uint32_t mask;
    int8_t   rssi;
    uint16_t frame_losses;
    bool     is_failsafe;
} srxl2_channel_data_t;

/* VTX data */
typedef struct {
    uint8_t  band;
    uint8_t  channel;
    uint8_t  pit;
    uint8_t  power;
    uint16_t power_mw;
    uint8_t  region;
} srxl2_vtx_data_t;

/* Bind data */
typedef struct {
    uint8_t  type;
    uint8_t  options;
    uint64_t guid;
    uint32_t uid;
} srxl2_bind_data_t;

/* Event types */
typedef enum {
    SRXL2_EVT_CHANNEL,              /* channel data received (slave) */
    SRXL2_EVT_TELEMETRY,            /* telemetry received (master) */
    SRXL2_EVT_BIND,                 /* bind event */
    SRXL2_EVT_VTX,                  /* VTX data */
    SRXL2_EVT_FWD_PGM,             /* forward programming data */
    SRXL2_EVT_HANDSHAKE_COMPLETE,   /* handshake phase done */
    SRXL2_EVT_TIMEOUT,             /* connection lost */
} srxl2_event_type_t;

/* Event structure (tagged union) */
typedef struct {
    srxl2_event_type_t type;
    union {
        struct {
            const srxl2_channel_data_t *data;
        } channel;
        struct {
            uint8_t device_id;
            const uint8_t *payload;  /* 16 bytes */
        } telemetry;
        struct {
            uint8_t request;
            uint8_t device_id;
            const srxl2_bind_data_t *data;
        } bind;
        struct {
            const srxl2_vtx_data_t *data;
        } vtx;
        struct {
            const uint8_t *data;
            uint8_t len;
        } fwd_pgm;
        struct {
            uint8_t peer_count;
        } handshake;
    };
} srxl2_event_t;

/* Event callback */
typedef void (*srxl2_event_cb_t)(srxl2_ctx_t *ctx, const srxl2_event_t *evt,
                                  void *user);

/*---------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

/* Return size needed for static allocation buffer */
size_t srxl2_ctx_size(void);

/* Init with user-provided buffer (no malloc). Returns NULL if buf too small. */
srxl2_ctx_t *srxl2_init_static(uint8_t *buf, size_t buf_size,
                                const srxl2_config_t *config);

/* Init with malloc. Returns NULL on failure. */
srxl2_ctx_t *srxl2_init(const srxl2_config_t *config);

/* Destroy (frees if malloc'd) */
void srxl2_destroy(srxl2_ctx_t *ctx);

/*---------------------------------------------------------------------------
 * Core loop
 *---------------------------------------------------------------------------*/

/* Buffer received UART bytes (ISR-safe, single producer) */
void srxl2_feed(srxl2_ctx_t *ctx, const uint8_t *data, size_t len);

/* Advance state machine (call from task context) */
void srxl2_tick(srxl2_ctx_t *ctx);

/*---------------------------------------------------------------------------
 * Master API
 *---------------------------------------------------------------------------*/

void srxl2_set_channels(srxl2_ctx_t *ctx, const uint16_t *values,
                         uint32_t mask);
void srxl2_set_failsafe(srxl2_ctx_t *ctx, bool failsafe);
bool srxl2_get_telemetry(srxl2_ctx_t *ctx, uint8_t device_id,
                          uint8_t payload_out[16], uint32_t *age_ms_out);

void srxl2_set_vtx(srxl2_ctx_t *ctx, const srxl2_vtx_data_t *vtx);
void srxl2_send_fwd_pgm(srxl2_ctx_t *ctx, uint8_t device_id,
                          const uint8_t *data, uint8_t len);
void srxl2_enter_bind(srxl2_ctx_t *ctx, uint8_t bind_type, bool broadcast);
void srxl2_set_bind_info(srxl2_ctx_t *ctx, const srxl2_bind_data_t *data);

/*---------------------------------------------------------------------------
 * Slave API
 *---------------------------------------------------------------------------*/

bool srxl2_get_channels(srxl2_ctx_t *ctx, srxl2_channel_data_t *data_out);
void srxl2_set_telemetry(srxl2_ctx_t *ctx, const uint8_t payload[16]);

/*---------------------------------------------------------------------------
 * Query
 *---------------------------------------------------------------------------*/

bool        srxl2_is_connected(const srxl2_ctx_t *ctx);
uint8_t     srxl2_peer_count(const srxl2_ctx_t *ctx);
uint32_t    srxl2_get_baud(const srxl2_ctx_t *ctx);
const char *srxl2_get_state(const srxl2_ctx_t *ctx);

/*---------------------------------------------------------------------------
 * Event registration
 *---------------------------------------------------------------------------*/

void srxl2_on_event(srxl2_ctx_t *ctx, srxl2_event_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* SRXL2_H */
