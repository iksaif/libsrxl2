/*
 * libsrxl2 -- Packet codec + CRC
 *
 * Pure functions: no state, no side effects. Decode raw wire bytes into
 * typed structs, encode typed structs into wire bytes with CRC.
 * All multi-byte wire fields use explicit byte-order helpers.
 *
 * MIT License
 */

#include "srxl2_packet.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * CRC-16 (XMODEM, polynomial 0x1021, seed 0)
 *
 * Three modes (compile-time):
 *   default          -- table-based lookup (512 bytes flash, fast)
 *   SRXL2_CRC_SMALL  -- bitwise computation (0 extra flash, slow)
 *   SRXL2_CRC_EXTERN -- user provides srxl2_crc16() externally
 *                        (e.g. STM32F7/H7 hardware CRC unit)
 *---------------------------------------------------------------------------*/

#ifdef SRXL2_CRC_EXTERN
/* User provides: uint16_t srxl2_crc16(const uint8_t *data, size_t len) */

#elif !defined(SRXL2_CRC_SMALL)

#include "srxl2_crc_table.inc"

uint16_t srxl2_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t pos = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ srxl2_crc_table[pos]);
    }
    return crc;
}

#else /* SRXL2_CRC_SMALL -- bitwise fallback */

uint16_t srxl2_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

#endif /* SRXL2_CRC */

/*---------------------------------------------------------------------------
 * CRC validation
 *---------------------------------------------------------------------------*/

bool srxl2_validate_crc(const uint8_t *data, uint8_t len)
{
    if (!data || len < 5)
        return false;
    uint16_t computed = srxl2_crc16(data, len - 2);
    uint16_t wire_crc = srxl2_rd_be16(&data[len - 2]);
    return computed == wire_crc;
}

/*---------------------------------------------------------------------------
 * String helpers
 *---------------------------------------------------------------------------*/

const char *srxl2_packet_type_name(uint8_t pkt_type)
{
    switch (pkt_type) {
    case SRXL2_PKT_HANDSHAKE: return "Handshake";
    case SRXL2_PKT_BIND:      return "Bind";
    case SRXL2_PKT_RSSI:      return "RSSI";
    case SRXL2_PKT_TELEMETRY:  return "Telemetry";
    case SRXL2_PKT_CONTROL:    return "Control Data";
    case 0x50:                 return "Parameter";
    case 0x99:                 return "Internal";
    default:                   return "Unknown";
    }
}

const char *srxl2_device_type_name(uint8_t device_type)
{
    switch (device_type) {
    case 0x0: return "None";
    case 0x1: return "Remote Receiver";
    case 0x2: return "Receiver";
    case 0x3: return "Flight Controller";
    case 0x4: return "ESC";
    case 0x6: return "Servo (Type 1)";
    case 0x7: return "Servo (Type 2)";
    case 0x8: return "VTX";
    case 0x9: return "External RF";
    case 0xA: return "Remote ID";
    case 0xB: return "Sensor";
    case 0xF: return "Broadcast";
    default:  return "Unknown";
    }
}

const char *srxl2_parse_result_name(srxl2_parse_result_t r)
{
    switch (r) {
    case SRXL2_PARSE_OK:          return "Success";
    case SRXL2_PARSE_ERR_MAGIC:   return "Invalid magic ID";
    case SRXL2_PARSE_ERR_LENGTH:  return "Invalid length";
    case SRXL2_PARSE_ERR_CRC:     return "CRC mismatch";
    case SRXL2_PARSE_ERR_UNKNOWN: return "Unknown packet type";
    default:                      return "Unknown error";
    }
}

/*---------------------------------------------------------------------------
 * Internal: append CRC and return total length
 *---------------------------------------------------------------------------*/

static uint8_t finalize_packet(uint8_t *buf, uint8_t len)
{
    uint16_t crc = srxl2_crc16(buf, len - 2);
    srxl2_wr_be16(&buf[len - 2], crc);
    return len;
}

/*---------------------------------------------------------------------------
 * Decode
 *---------------------------------------------------------------------------*/

srxl2_parse_result_t srxl2_pkt_parse(const uint8_t *raw, uint8_t len,
                                      srxl2_decoded_pkt_t *out)
{
    if (len < 5)
        return SRXL2_PARSE_ERR_LENGTH;

    if (raw[0] != SRXL2_MAGIC)
        return SRXL2_PARSE_ERR_MAGIC;

    uint8_t pkt_len = raw[2];
    if (pkt_len < 5 || pkt_len > len)
        return SRXL2_PARSE_ERR_LENGTH;

    /* Verify CRC over bytes [0..pkt_len-3], compare with last 2 bytes */
    uint16_t computed = srxl2_crc16(raw, pkt_len - 2);
    uint16_t wire_crc = srxl2_rd_be16(&raw[pkt_len - 2]);
    if (computed != wire_crc)
        return SRXL2_PARSE_ERR_CRC;

    memset(out, 0, sizeof(*out));
    out->packet_type = raw[1];
    out->length = pkt_len;

    switch (raw[1]) {
    case SRXL2_PKT_HANDSHAKE: {
        if (pkt_len < 14)
            return SRXL2_PARSE_ERR_LENGTH;
        srxl2_pkt_handshake_t *hs = &out->handshake;
        hs->src_id         = raw[3];
        hs->dest_id        = raw[4];
        hs->priority       = raw[5];
        hs->baud_supported = raw[6];
        hs->info           = raw[7];
        hs->uid            = srxl2_rd_le32(&raw[8]);
        break;
    }

    case SRXL2_PKT_CONTROL: {
        if (pkt_len < 7)
            return SRXL2_PARSE_ERR_LENGTH;
        srxl2_pkt_control_t *ctrl = &out->control;
        ctrl->cmd      = raw[3];
        ctrl->reply_id = raw[4];

        switch (ctrl->cmd) {
        case SRXL2_CMD_CHANNEL:
        case SRXL2_CMD_CHANNEL_FS: {
            if (pkt_len < 14)
                return SRXL2_PARSE_ERR_LENGTH;
            ctrl->channel.rssi         = (int8_t)raw[5];
            ctrl->channel.frame_losses = srxl2_rd_le16(&raw[6]);
            ctrl->channel.mask         = srxl2_rd_le32(&raw[8]);

            /* Count channels from mask */
            uint32_t m = ctrl->channel.mask;
            uint8_t ch_idx = 0;
            for (int i = 0; i < 32 && ch_idx < 32; i++) {
                if (m & (1u << i)) {
                    uint8_t off = (uint8_t)(12 + ch_idx * 2);
                    if (off + 2 > pkt_len - 2)
                        break;
                    ctrl->channel.values[i] = srxl2_rd_le16(&raw[off]);
                    ch_idx++;
                }
            }
            ctrl->channel.num_channels = ch_idx;
            break;
        }

        case SRXL2_CMD_VTX: {
            if (pkt_len < 14)
                return SRXL2_PARSE_ERR_LENGTH;
            ctrl->vtx.band     = raw[5];
            ctrl->vtx.channel  = raw[6];
            ctrl->vtx.pit      = raw[7];
            ctrl->vtx.power    = raw[8];
            ctrl->vtx.power_mw = srxl2_rd_le16(&raw[9]);
            ctrl->vtx.region   = raw[11];
            break;
        }

        case SRXL2_CMD_FWDPGM: {
            if (pkt_len < 10)
                return SRXL2_PARSE_ERR_LENGTH;
            ctrl->fwd_pgm.rssi = (int8_t)raw[5];
            /* raw[6..7] are RFU (reserved) */
            uint8_t data_len = (uint8_t)(pkt_len - 10);
            if (data_len > SRXL2_FWD_PGM_MAX)
                data_len = SRXL2_FWD_PGM_MAX;
            memcpy(ctrl->fwd_pgm.data, &raw[8], data_len);
            ctrl->fwd_pgm.len = data_len;
            break;
        }

        default:
            return SRXL2_PARSE_ERR_UNKNOWN;
        }
        break;
    }

    case SRXL2_PKT_TELEMETRY: {
        if (pkt_len < 22)
            return SRXL2_PARSE_ERR_LENGTH;
        out->telemetry.dest_id = raw[3];
        memcpy(out->telemetry.payload, &raw[4], 16);
        break;
    }

    case SRXL2_PKT_BIND: {
        if (pkt_len < 19)
            return SRXL2_PARSE_ERR_LENGTH;
        srxl2_pkt_bind_t *bd = &out->bind;
        bd->request       = raw[3];
        bd->device_id     = raw[4];
        bd->data.type     = raw[5];
        bd->data.options  = raw[6];
        bd->data.guid     = srxl2_rd_le64(&raw[7]);
        bd->data.uid      = srxl2_rd_le32(&raw[15]);
        break;
    }

    case SRXL2_PKT_RSSI: {
        if (pkt_len < 10)
            return SRXL2_PARSE_ERR_LENGTH;
        out->rssi.request   = raw[3];
        out->rssi.antenna_a = (int8_t)raw[4];
        out->rssi.antenna_b = (int8_t)raw[5];
        out->rssi.antenna_c = (int8_t)raw[6];
        out->rssi.antenna_d = (int8_t)raw[7];
        break;
    }

    default:
        return SRXL2_PARSE_ERR_UNKNOWN;
    }

    return SRXL2_PARSE_OK;
}

/*---------------------------------------------------------------------------
 * Encode: Handshake (0x21)
 * Wire: A6 21 0E src dest priority baud info uid[4] crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_handshake(uint8_t *buf, uint8_t src, uint8_t dest,
                              uint8_t priority, uint8_t baud, uint8_t info,
                              uint32_t uid)
{
    uint8_t len = 14;
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_HANDSHAKE;
    buf[2] = len;
    buf[3] = src;
    buf[4] = dest;
    buf[5] = priority;
    buf[6] = baud;
    buf[7] = info;
    srxl2_wr_le32(&buf[8], uid);
    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: Channel Data (0xCD, cmd=0x00 or 0x01)
 * Wire: A6 CD len cmd replyID rssi frameLosses[2] mask[4] values[2*N] crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_channel(uint8_t *buf, uint8_t cmd, uint8_t reply_id,
                            int8_t rssi, uint16_t frame_losses,
                            uint32_t mask, const uint16_t *values)
{
    /* Count channels in mask */
    uint8_t num_ch = 0;
    for (int i = 0; i < 32; i++) {
        if (mask & (1u << i))
            num_ch++;
    }

    /* header(3) + cmd(1) + replyID(1) + rssi(1) + frameLosses(2) + mask(4) + values(2*N) + crc(2) */
    uint8_t len = (uint8_t)(14 + num_ch * 2);
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_CONTROL;
    buf[2] = len;
    buf[3] = cmd;
    buf[4] = reply_id;
    buf[5] = (uint8_t)rssi;
    srxl2_wr_le16(&buf[6], frame_losses);
    srxl2_wr_le32(&buf[8], mask);

    uint8_t ch_idx = 0;
    for (int i = 0; i < 32; i++) {
        if (mask & (1u << i)) {
            srxl2_wr_le16(&buf[12 + ch_idx * 2], values[i]);
            ch_idx++;
        }
    }

    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: Telemetry (0x80)
 * Wire: A6 80 16 destID payload[16] crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_telemetry(uint8_t *buf, uint8_t dest_id,
                              const uint8_t payload[16])
{
    uint8_t len = 22;
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_TELEMETRY;
    buf[2] = len;
    buf[3] = dest_id;
    memcpy(&buf[4], payload, 16);
    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: Bind (0x41)
 * Wire: A6 41 13 request deviceID type options guid[8] uid[4] crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_bind(uint8_t *buf, uint8_t request, uint8_t device_id,
                         const srxl2_bind_data_t *data)
{
    uint8_t len = 21;
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_BIND;
    buf[2] = len;
    buf[3] = request;
    buf[4] = device_id;
    buf[5] = data->type;
    buf[6] = data->options;
    srxl2_wr_le64(&buf[7], data->guid);
    srxl2_wr_le32(&buf[15], data->uid);
    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: VTX (0xCD, cmd=0x02)
 * Wire: A6 CD len 02 replyID band channel pit power powerDec[2] region crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_vtx(uint8_t *buf, uint8_t reply_id,
                        const srxl2_vtx_data_t *vtx)
{
    uint8_t len = 14;
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_CONTROL;
    buf[2] = len;
    buf[3] = SRXL2_CMD_VTX;
    buf[4] = reply_id;
    buf[5] = vtx->band;
    buf[6] = vtx->channel;
    buf[7] = vtx->pit;
    buf[8] = vtx->power;
    srxl2_wr_le16(&buf[9], vtx->power_mw);
    buf[11] = vtx->region;
    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: RSSI (0x55)
 * Wire: A6 55 09 request antA antB antC antD crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_rssi(uint8_t *buf, uint8_t request, int8_t a, int8_t b,
                         int8_t c, int8_t d)
{
    uint8_t len = 10;
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_RSSI;
    buf[2] = len;
    buf[3] = request;
    buf[4] = (uint8_t)a;
    buf[5] = (uint8_t)b;
    buf[6] = (uint8_t)c;
    buf[7] = (uint8_t)d;
    return finalize_packet(buf, len);
}

/*---------------------------------------------------------------------------
 * Encode: Forward Programming (0xCD, cmd=0x03)
 * Wire: A6 CD len 03 replyID rssi rfu[2] data[N] crc[2]
 *---------------------------------------------------------------------------*/

uint8_t srxl2_pkt_fwd_pgm(uint8_t *buf, uint8_t reply_id, int8_t rssi,
                            const uint8_t *data, uint8_t data_len)
{
    if (data_len > SRXL2_FWD_PGM_MAX)
        data_len = SRXL2_FWD_PGM_MAX;

    uint8_t len = (uint8_t)(10 + data_len);
    buf[0] = SRXL2_MAGIC;
    buf[1] = SRXL2_PKT_CONTROL;
    buf[2] = len;
    buf[3] = SRXL2_CMD_FWDPGM;
    buf[4] = reply_id;
    buf[5] = (uint8_t)rssi;
    buf[6] = 0;  /* RFU */
    buf[7] = 0;  /* RFU */
    memcpy(&buf[8], data, data_len);
    return finalize_packet(buf, len);
}
