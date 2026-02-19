/*
 * libsrxl2 -- Internal packet codec prototypes
 *
 * Pure functions: decode raw bytes, encode wire format, compute CRC.
 * No state machine access, no side effects.
 *
 * MIT License
 */

#ifndef SRXL2_PACKET_H
#define SRXL2_PACKET_H

#include "srxl2.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Wire constants
 *---------------------------------------------------------------------------*/

#define SRXL2_FWD_PGM_MAX      64
#define SRXL2_MAX_PACKET_SIZE   80

/*---------------------------------------------------------------------------
 * Byte-order helpers (inline, portable)
 *---------------------------------------------------------------------------*/

static inline uint16_t srxl2_rd_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t srxl2_rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t srxl2_rd_le64(const uint8_t *p)
{
    uint64_t lo = srxl2_rd_le32(p);
    uint64_t hi = srxl2_rd_le32(p + 4);
    return lo | (hi << 32);
}

static inline uint16_t srxl2_rd_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline void srxl2_wr_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void srxl2_wr_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void srxl2_wr_le64(uint8_t *p, uint64_t v)
{
    srxl2_wr_le32(p, (uint32_t)(v & 0xFFFFFFFF));
    srxl2_wr_le32(p + 4, (uint32_t)(v >> 32));
}

static inline uint32_t srxl2_rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static inline void srxl2_wr_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static inline void srxl2_wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}

/*---------------------------------------------------------------------------
 * Parse result
 *---------------------------------------------------------------------------*/

typedef enum {
    SRXL2_PARSE_OK = 0,
    SRXL2_PARSE_ERR_MAGIC,      /* first byte != 0xA6 */
    SRXL2_PARSE_ERR_LENGTH,     /* length field invalid or truncated */
    SRXL2_PARSE_ERR_CRC,        /* CRC mismatch */
    SRXL2_PARSE_ERR_UNKNOWN,    /* unknown packet type */
} srxl2_parse_result_t;

/*---------------------------------------------------------------------------
 * Decoded packet types
 *---------------------------------------------------------------------------*/

typedef struct {
    uint8_t  src_id;
    uint8_t  dest_id;
    uint8_t  priority;
    uint8_t  baud_supported;
    uint8_t  info;
    uint32_t uid;
} srxl2_pkt_handshake_t;

typedef struct {
    uint8_t  cmd;       /* SRXL2_CMD_CHANNEL / CHANNEL_FS / VTX / FWDPGM */
    uint8_t  reply_id;
    union {
        struct {
            int8_t   rssi;
            uint16_t frame_losses;
            uint32_t mask;
            uint16_t values[32];
            uint8_t  num_channels;
        } channel;
        srxl2_vtx_data_t vtx;
        struct {
            int8_t   rssi;
            uint8_t  data[64];
            uint8_t  len;
        } fwd_pgm;
    };
} srxl2_pkt_control_t;

typedef struct {
    uint8_t dest_id;
    uint8_t payload[16];
} srxl2_pkt_telemetry_t;

typedef struct {
    uint8_t  request;
    uint8_t  device_id;
    srxl2_bind_data_t data;
} srxl2_pkt_bind_t;

typedef struct {
    uint8_t request;
    int8_t  antenna_a;
    int8_t  antenna_b;
    int8_t  antenna_c;
    int8_t  antenna_d;
} srxl2_pkt_rssi_t;

/* Decoded packet: header + typed union */
typedef struct {
    uint8_t packet_type;
    uint8_t length;
    union {
        srxl2_pkt_handshake_t handshake;
        srxl2_pkt_control_t   control;
        srxl2_pkt_telemetry_t telemetry;
        srxl2_pkt_bind_t      bind;
        srxl2_pkt_rssi_t      rssi;
    };
} srxl2_decoded_pkt_t;

/*---------------------------------------------------------------------------
 * String helpers
 *---------------------------------------------------------------------------*/

const char *srxl2_packet_type_name(uint8_t pkt_type);
const char *srxl2_device_type_name(uint8_t device_type);
const char *srxl2_parse_result_name(srxl2_parse_result_t r);

/*---------------------------------------------------------------------------
 * CRC validation
 *---------------------------------------------------------------------------*/

bool srxl2_validate_crc(const uint8_t *data, uint8_t len);

/*---------------------------------------------------------------------------
 * Decode
 *---------------------------------------------------------------------------*/

srxl2_parse_result_t srxl2_pkt_parse(const uint8_t *raw, uint8_t len,
                                      srxl2_decoded_pkt_t *out);

/*---------------------------------------------------------------------------
 * Encode (each returns wire length written to buf)
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_handshake(uint8_t *buf, uint8_t src, uint8_t dest,
                              uint8_t priority, uint8_t baud, uint8_t info,
                              uint32_t uid);

uint8_t srxl2_pkt_channel(uint8_t *buf, uint8_t cmd, uint8_t reply_id,
                            int8_t rssi, uint16_t frame_losses,
                            uint32_t mask, const uint16_t *values);

uint8_t srxl2_pkt_telemetry(uint8_t *buf, uint8_t dest_id,
                              const uint8_t payload[16]);

uint8_t srxl2_pkt_bind(uint8_t *buf, uint8_t request, uint8_t device_id,
                         const srxl2_bind_data_t *data);

uint8_t srxl2_pkt_vtx(uint8_t *buf, uint8_t reply_id,
                        const srxl2_vtx_data_t *vtx);

uint8_t srxl2_pkt_rssi(uint8_t *buf, uint8_t request, int8_t a, int8_t b,
                         int8_t c, int8_t d);

uint8_t srxl2_pkt_fwd_pgm(uint8_t *buf, uint8_t reply_id, int8_t rssi,
                            const uint8_t *data, uint8_t data_len);

/*---------------------------------------------------------------------------
 * CRC
 *---------------------------------------------------------------------------*/

uint16_t srxl2_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* SRXL2_PACKET_H */
