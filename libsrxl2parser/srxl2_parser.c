/*
 * SRXL2 Parser Library - Modern C Implementation
 *
 * A standalone, dependency-free SRXL2 packet parser library.
 * This library provides read-only parsing of SRXL2 protocol packets
 * with no side effects or external dependencies.
 *
 * MIT License
 */

#include "srxl2_parser.h"
#include <string.h>

//=============================================================================
// CRC-16 Implementation (XMODEM/ZMODEM)
//=============================================================================

uint16_t srxl2_compute_crc(const uint8_t *data, size_t length)
{
    if (!data || length == 0)
        return 0;

    uint16_t crc = 0;

    for (size_t i = 0; i < length; i++)
    {
        crc = crc ^ ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }

    return crc;
}

bool srxl2_validate_crc(const uint8_t *data, size_t length)
{
    if (!data || length < SRXL2_MIN_PACKET_SIZE)
        return false;

    // Extract CRC from last 2 bytes (big-endian)
    uint16_t packet_crc = ((uint16_t)data[length - 2] << 8) | data[length - 1];

    // Compute CRC over all bytes except last 2
    uint16_t computed_crc = srxl2_compute_crc(data, length - SRXL2_CRC_SIZE);

    return packet_crc == computed_crc;
}

//=============================================================================
// Packet Parsing Helper Functions
//=============================================================================

/**
 * @brief Parse handshake packet payload
 */
static srxl2_parse_result_t parse_handshake(const uint8_t *data, size_t length, srxl2_packet_t *packet)
{
    // Handshake packet: 3 header + 9 payload + 2 CRC = 14 bytes
    if (length != 14)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    const uint8_t *payload = data + SRXL2_HEADER_SIZE;

    packet->payload.handshake.src_device_id = payload[0];
    packet->payload.handshake.dest_device_id = payload[1];
    packet->payload.handshake.priority = payload[2];
    packet->payload.handshake.baud_supported = payload[3];
    packet->payload.handshake.info = payload[4];

    // UID is 4 bytes, big-endian
    packet->payload.handshake.uid = ((uint32_t)payload[5] << 24) |
                                    ((uint32_t)payload[6] << 16) |
                                    ((uint32_t)payload[7] << 8) |
                                    ((uint32_t)payload[8]);

    return SRXL2_PARSE_OK;
}

/**
 * @brief Parse channel data from control packet
 */
static void parse_channel_data(const uint8_t *data, srxl2_channel_data_t *channel_data)
{
    channel_data->rssi = (int8_t)data[0];
    channel_data->frame_losses = ((uint16_t)data[1] << 8) | data[2];

    // Channel mask is 4 bytes, big-endian
    channel_data->channel_mask = ((uint32_t)data[3] << 24) |
                                  ((uint32_t)data[4] << 16) |
                                  ((uint32_t)data[5] << 8) |
                                  ((uint32_t)data[6]);

    // Parse channel values (2 bytes each, big-endian)
    channel_data->num_channels = 0;
    const uint8_t *ch_ptr = data + 7;

    for (uint8_t i = 0; i < 32; i++)
    {
        if (channel_data->channel_mask & (1UL << i))
        {
            channel_data->values[i] = ((uint16_t)ch_ptr[0] << 8) | ch_ptr[1];
            ch_ptr += 2;
            channel_data->num_channels++;
        }
        else
        {
            channel_data->values[i] = 0;
        }
    }
}

/**
 * @brief Parse control packet payload
 */
static srxl2_parse_result_t parse_control(const uint8_t *data, size_t length, srxl2_packet_t *packet)
{
    // Control packet: 3 header + 2 cmd/reply + 7+ channel data + 2 CRC
    // Minimum: 3 + 2 + 7 + 2 = 14 bytes
    if (length < 14)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    const uint8_t *payload = data + SRXL2_HEADER_SIZE;

    packet->payload.control.cmd = payload[0];
    packet->payload.control.reply_id = payload[1];

    // Parse channel data
    parse_channel_data(payload + 2, &packet->payload.control.channel_data);

    return SRXL2_PARSE_OK;
}

/**
 * @brief Parse telemetry packet payload
 */
static srxl2_parse_result_t parse_telemetry(const uint8_t *data, size_t length, srxl2_packet_t *packet)
{
    // Telemetry packet: 3 header + 1 dest + 16 data + 2 CRC = 22 bytes
    if (length != 22)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    const uint8_t *payload = data + SRXL2_HEADER_SIZE;

    packet->payload.telemetry.dest_device_id = payload[0];
    memcpy(packet->payload.telemetry.raw, payload + 1, 16);

    return SRXL2_PARSE_OK;
}

/**
 * @brief Parse bind packet payload
 */
static srxl2_parse_result_t parse_bind(const uint8_t *data, size_t length, srxl2_packet_t *packet)
{
    // Bind packet: 3 header + 16 payload + 2 CRC = 21 bytes
    if (length != 21)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    const uint8_t *payload = data + SRXL2_HEADER_SIZE;

    packet->payload.bind.request = payload[0];
    packet->payload.bind.device_id = payload[1];
    packet->payload.bind.bind_type = payload[2];
    packet->payload.bind.options = payload[3];

    // GUID is 8 bytes, big-endian
    packet->payload.bind.guid = ((uint64_t)payload[4] << 56) |
                                ((uint64_t)payload[5] << 48) |
                                ((uint64_t)payload[6] << 40) |
                                ((uint64_t)payload[7] << 32) |
                                ((uint64_t)payload[8] << 24) |
                                ((uint64_t)payload[9] << 16) |
                                ((uint64_t)payload[10] << 8) |
                                ((uint64_t)payload[11]);

    // UID is 4 bytes, big-endian
    packet->payload.bind.uid = ((uint32_t)payload[12] << 24) |
                               ((uint32_t)payload[13] << 16) |
                               ((uint32_t)payload[14] << 8) |
                               ((uint32_t)payload[15]);

    return SRXL2_PARSE_OK;
}

/**
 * @brief Parse internal packet payload
 */
static srxl2_parse_result_t parse_internal(const uint8_t *data, size_t length, srxl2_packet_t *packet)
{
    // Internal packet: 3 header + 7 payload + 2 CRC = 12 bytes
    if (length != 12)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    const uint8_t *payload = data + SRXL2_HEADER_SIZE;

    packet->payload.internal.src_device_id = payload[0];
    packet->payload.internal.dest_device_id = payload[1];
    packet->payload.internal.test = payload[2];

    // Key is 4 bytes, big-endian
    packet->payload.internal.key = ((uint32_t)payload[3] << 24) |
                                   ((uint32_t)payload[4] << 16) |
                                   ((uint32_t)payload[5] << 8) |
                                   ((uint32_t)payload[6]);

    return SRXL2_PARSE_OK;
}

//=============================================================================
// Main Parsing Function
//=============================================================================

srxl2_parse_result_t srxl2_parse_packet(
    const uint8_t *data,
    size_t length,
    srxl2_packet_t *packet)
{
    // Validate input parameters
    if (!data || !packet)
        return SRXL2_PARSE_ERR_NULL_PTR;

    if (length < SRXL2_MIN_PACKET_SIZE)
        return SRXL2_PARSE_ERR_TOO_SHORT;

    if (length > SRXL2_MAX_PACKET_SIZE)
        return SRXL2_PARSE_ERR_TOO_LONG;

    // Validate magic ID
    if (data[0] != SRXL2_MAGIC_ID)
        return SRXL2_PARSE_ERR_BAD_MAGIC;

    // Validate length field
    uint8_t packet_length = data[2];
    if (packet_length != length)
        return SRXL2_PARSE_ERR_BAD_LENGTH;

    // Validate CRC
    if (!srxl2_validate_crc(data, length))
        return SRXL2_PARSE_ERR_BAD_CRC;

    // Parse header
    packet->header.magic_id = data[0];
    packet->header.packet_type = data[1];
    packet->header.length = data[2];

    // Parse payload based on packet type
    srxl2_parse_result_t result;

    switch (packet->header.packet_type)
    {
    case SRXL2_PKT_HANDSHAKE:
        result = parse_handshake(data, length, packet);
        break;

    case SRXL2_PKT_CONTROL:
        result = parse_control(data, length, packet);
        break;

    case SRXL2_PKT_TELEMETRY:
        result = parse_telemetry(data, length, packet);
        break;

    case SRXL2_PKT_BIND:
        result = parse_bind(data, length, packet);
        break;

    case SRXL2_PKT_SPM_INTERNAL:
        result = parse_internal(data, length, packet);
        break;

    case SRXL2_PKT_RSSI:
    case SRXL2_PKT_PARAMETER:
        // These packet types are not fully implemented yet
        // Just copy raw payload
        {
            size_t payload_len = length - SRXL2_HEADER_SIZE - SRXL2_CRC_SIZE;
            memcpy(packet->payload.raw, data + SRXL2_HEADER_SIZE, payload_len);
            result = SRXL2_PARSE_OK;
        }
        break;

    default:
        return SRXL2_PARSE_ERR_UNKNOWN_TYPE;
    }

    return result;
}

//=============================================================================
// String Conversion Functions
//=============================================================================

const char* srxl2_device_type_name(srxl2_device_type_t device_type)
{
    switch (device_type)
    {
    case SRXL2_DEV_NONE:              return "None";
    case SRXL2_DEV_REMOTE_RECEIVER:   return "Remote Receiver";
    case SRXL2_DEV_RECEIVER:          return "Receiver";
    case SRXL2_DEV_FLIGHT_CONTROLLER: return "Flight Controller";
    case SRXL2_DEV_ESC:               return "ESC";
    case SRXL2_DEV_SERVO_1:           return "Servo (Type 1)";
    case SRXL2_DEV_SERVO_2:           return "Servo (Type 2)";
    case SRXL2_DEV_VTX:               return "VTX";
    case SRXL2_DEV_EXTERNAL_RF:       return "External RF";
    case SRXL2_DEV_REMOTE_ID:         return "Remote ID";
    case SRXL2_DEV_SENSOR:            return "Sensor";
    case SRXL2_DEV_BROADCAST:         return "Broadcast";
    default:                          return "Unknown";
    }
}

const char* srxl2_packet_type_name(srxl2_packet_type_t packet_type)
{
    switch (packet_type)
    {
    case SRXL2_PKT_HANDSHAKE:    return "Handshake";
    case SRXL2_PKT_BIND:         return "Bind";
    case SRXL2_PKT_PARAMETER:    return "Parameter";
    case SRXL2_PKT_RSSI:         return "RSSI";
    case SRXL2_PKT_TELEMETRY:    return "Telemetry";
    case SRXL2_PKT_SPM_INTERNAL: return "Internal";
    case SRXL2_PKT_CONTROL:      return "Control Data";
    default:                     return "Unknown";
    }
}

const char* srxl2_parse_result_str(srxl2_parse_result_t result)
{
    switch (result)
    {
    case SRXL2_PARSE_OK:              return "Success";
    case SRXL2_PARSE_ERR_NULL_PTR:    return "Null pointer";
    case SRXL2_PARSE_ERR_TOO_SHORT:   return "Packet too short";
    case SRXL2_PARSE_ERR_TOO_LONG:    return "Packet too long";
    case SRXL2_PARSE_ERR_BAD_MAGIC:   return "Invalid magic ID";
    case SRXL2_PARSE_ERR_BAD_LENGTH:  return "Invalid length field";
    case SRXL2_PARSE_ERR_BAD_CRC:     return "CRC mismatch";
    case SRXL2_PARSE_ERR_UNKNOWN_TYPE: return "Unknown packet type";
    default:                          return "Unknown error";
    }
}
