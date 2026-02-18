/*
 * Shared Test Infrastructure for SRXL2 State Machine Tests
 *
 * Provides init/reset/packet-building helpers and implements all 13
 * master hooks plus the 3 user callbacks as no-op trackers.
 *
 * MIT License
 */

#include "test_helpers.h"
#include "uart_adapter.h"
#include <string.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// UART capture variable definitions (declared extern in uart_adapter.h)
///////////////////////////////////////////////////////////////////////////////

uint8_t g_tx_buf[CAPTURE_TX_BUF_SIZE];
size_t g_tx_len = 0;
capture_packet_entry_t g_tx_packets[CAPTURE_MAX_PACKETS];
size_t g_tx_count = 0;
uint32_t g_baud_rate = 115200;

///////////////////////////////////////////////////////////////////////////////
// Hook call counters
///////////////////////////////////////////////////////////////////////////////

int g_hook_run_start_count = 0;
int g_hook_run_end_count = 0;
int g_hook_handshake_start_count = 0;
int g_hook_handshake_complete_count = 0;
int g_hook_handshake_complete_dev_count = 0;
int g_hook_frame_count = 0;
int g_hook_channel_sent_count = 0;
uint8_t g_hook_channel_sent_telem_id = 0;
int g_hook_select_telem_count = 0;
uint8_t g_hook_select_telem_default_id = 0;
int g_hook_set_telem_tx_count = 0;
bool g_hook_set_telem_tx_enabled = false;
int g_hook_telem_sent_count = 0;
int g_hook_suppress_telem_count = 0;
int g_hook_bind_count = 0;
int g_hook_parse_internal_count = 0;
int g_hook_fill_internal_count = 0;

bool g_hook_frame_skip = false;
bool g_hook_select_telem_passthrough = true;
uint8_t g_hook_select_telem_override = 0;

// Callback tracking
int g_cb_fill_telem_count = 0;
int g_cb_recv_channel_count = 0;
bool g_cb_recv_channel_is_failsafe = false;
int g_cb_vtx_count = 0;

///////////////////////////////////////////////////////////////////////////////
// Master hook implementations (all 13)
///////////////////////////////////////////////////////////////////////////////

void srxlOnMasterRunStart(uint8_t busIndex)
{
    (void)busIndex;
    g_hook_run_start_count++;
}

void srxlOnMasterRunEnd(uint8_t busIndex)
{
    (void)busIndex;
    g_hook_run_end_count++;
}

void srxlOnMasterHandshakeStart(uint8_t busIndex)
{
    (void)busIndex;
    g_hook_handshake_start_count++;
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    (void)busIndex;
    g_hook_handshake_complete_count++;
    g_hook_handshake_complete_dev_count = deviceCount;
}

bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount)
{
    (void)busIndex;
    (void)frameCount;
    g_hook_frame_count++;
    return g_hook_frame_skip;
}

void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID)
{
    (void)busIndex;
    g_hook_channel_sent_count++;
    g_hook_channel_sent_telem_id = telemDeviceID;
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    (void)busIndex;
    g_hook_select_telem_count++;
    g_hook_select_telem_default_id = defaultDeviceID;
    if (g_hook_select_telem_passthrough)
        return defaultDeviceID;
    return g_hook_select_telem_override;
}

void srxlOnMasterSetTelemTx(bool enabled)
{
    g_hook_set_telem_tx_count++;
    g_hook_set_telem_tx_enabled = enabled;
}

void srxlOnMasterTelemSent(void)
{
    g_hook_telem_sent_count++;
}

void srxlOnMasterSuppressTelem(void *pTelemetryData)
{
    (void)pTelemetryData;
    g_hook_suppress_telem_count++;
}

bool srxlOnMasterBind(void *pBindInfo)
{
    (void)pBindInfo;
    g_hook_bind_count++;
    return true;
}

bool srxlOnMasterParseInternal(void *pInternal)
{
    (void)pInternal;
    g_hook_parse_internal_count++;
    return true;
}

uint8_t srxlOnMasterFillInternal(void *pInternal)
{
    (void)pInternal;
    g_hook_fill_internal_count++;
    return 10; // minimal length
}

///////////////////////////////////////////////////////////////////////////////
// User callback implementations
///////////////////////////////////////////////////////////////////////////////

void userProvidedFillSrxlTelemetry(SrxlTelemetryData *pTelemetry)
{
    (void)pTelemetry;
    g_cb_fill_telem_count++;
}

void userProvidedReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafeData)
{
    (void)pChannelData;
    g_cb_recv_channel_count++;
    g_cb_recv_channel_is_failsafe = isFailsafeData;
}

void userProvidedHandleVtxData(SrxlVtxData *pVtxData)
{
    (void)pVtxData;
    g_cb_vtx_count++;
}

///////////////////////////////////////////////////////////////////////////////
// CRC-16 (XMODEM, poly 0x1021, seed 0)
///////////////////////////////////////////////////////////////////////////////

uint16_t test_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

///////////////////////////////////////////////////////////////////////////////
// Reset helpers
///////////////////////////////////////////////////////////////////////////////

static void reset_hook_counters(void)
{
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

void test_reset_capture(void)
{
    capture_reset();
    reset_hook_counters();
}

void test_reset_all(void)
{
    memset(&srxlThisDev, 0, sizeof(srxlThisDev));
    memset(srxlBus, 0, sizeof(SrxlBus) * SRXL_NUM_OF_BUSES);
    memset(&srxlChData, 0, sizeof(srxlChData));
    memset(&srxlTelemData, 0, sizeof(srxlTelemData));
    memset(&srxlRx, 0, sizeof(srxlRx));
    srxlChDataIsFailsafe = false;
    srxlTelemetryPhase = false;
    srxlFailsafeChMask = 0;
    g_baud_rate = 115200;
    test_reset_capture();
}

///////////////////////////////////////////////////////////////////////////////
// Init helpers
///////////////////////////////////////////////////////////////////////////////

void test_init_master(uint8_t deviceID)
{
    test_reset_all();
    srxlInitDevice(deviceID, 20, SRXL_DEVINFO_TELEM_TX_ENABLED | SRXL_DEVINFO_TELEM_FULL_RANGE, 0x12345678);
    srxlInitBus(0, 0, SRXL_BAUD_400000);
    // Force master mode if the lib didn't set it (only 0x10 gets auto-master)
    srxlBus[0].master = true;
    capture_reset();
}

void test_init_slave(uint8_t deviceID)
{
    test_reset_all();
    srxlInitDevice(deviceID, 10, SRXL_DEVINFO_NO_RF, 0xAABBCCDD);
    srxlInitBus(0, 0, SRXL_BAUD_400000);
    srxlBus[0].master = false;
    capture_reset();
}

///////////////////////////////////////////////////////////////////////////////
// Packet builders
///////////////////////////////////////////////////////////////////////////////

uint8_t test_build_handshake(uint8_t *buf, uint8_t src, uint8_t dest,
                              uint8_t priority, uint8_t baud,
                              uint8_t info, uint32_t uid)
{
    uint8_t len = 14; // sizeof(SrxlHandshakePacket) = 3 + 9 + 2 = 14
    buf[0] = 0xA6;    // SPEKTRUM_SRXL_ID
    buf[1] = 0x21;    // SRXL_HANDSHAKE_ID
    buf[2] = len;
    buf[3] = src;
    buf[4] = dest;
    buf[5] = priority;
    buf[6] = baud;
    buf[7] = info;
    // UID is stored as little-endian in the packed struct
    memcpy(&buf[8], &uid, 4);

    uint16_t crc = test_crc16(buf, len - 2);
    buf[len - 2] = (crc >> 8) & 0xFF;
    buf[len - 1] = crc & 0xFF;
    return len;
}

uint8_t test_build_channel_data(uint8_t *buf, uint8_t cmd, uint8_t reply_id,
                                 uint32_t mask, const uint16_t *values,
                                 int8_t rssi)
{
    // Count channels in mask
    uint8_t num_channels = 0;
    for (int i = 0; i < 32; i++)
    {
        if (mask & (1u << i))
            num_channels++;
    }

    // Packet: header(3) + cmd(1) + replyID(1) + rssi(1) + frameLosses(2) + mask(4) + channels(2*N) + crc(2)
    uint8_t len = 3 + 1 + 1 + 1 + 2 + 4 + (2 * num_channels) + 2;
    buf[0] = 0xA6;
    buf[1] = 0xCD; // SRXL_CTRL_ID
    buf[2] = len;
    buf[3] = cmd;
    buf[4] = reply_id;
    buf[5] = (uint8_t)rssi;
    buf[6] = 0; // frameLosses low
    buf[7] = 0; // frameLosses high

    // Channel mask (little-endian as packed struct)
    memcpy(&buf[8], &mask, 4);

    // Channel values (packed in order of set bits)
    uint8_t ch_idx = 0;
    uint8_t val_idx = 0;
    for (int i = 0; i < 32; i++)
    {
        if (mask & (1u << i))
        {
            uint16_t v = values ? values[val_idx++] : 0;
            memcpy(&buf[12 + ch_idx * 2], &v, 2);
            ch_idx++;
        }
    }

    uint16_t crc = test_crc16(buf, len - 2);
    buf[len - 2] = (crc >> 8) & 0xFF;
    buf[len - 1] = crc & 0xFF;
    return len;
}

uint8_t test_build_telemetry(uint8_t *buf, uint8_t dest_id,
                              const uint8_t payload[16])
{
    uint8_t len = 22; // sizeof(SrxlTelemetryPacket) = 3 + 1 + 16 + 2 = 22
    buf[0] = 0xA6;
    buf[1] = 0x80; // SRXL_TELEM_ID
    buf[2] = len;
    buf[3] = dest_id;
    memcpy(&buf[4], payload, 16);

    uint16_t crc = test_crc16(buf, len - 2);
    buf[len - 2] = (crc >> 8) & 0xFF;
    buf[len - 1] = crc & 0xFF;
    return len;
}

///////////////////////////////////////////////////////////////////////////////
// Injection / advance wrappers
///////////////////////////////////////////////////////////////////////////////

bool test_inject_packet(uint8_t bus, uint8_t *packet, uint8_t len)
{
    return srxlParsePacket(bus, packet, len);
}

void test_advance_ms(uint8_t bus, int16_t ms)
{
    srxlRun(bus, ms);
}
