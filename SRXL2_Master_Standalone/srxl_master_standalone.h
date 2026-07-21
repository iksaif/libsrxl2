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

#ifndef __SRXL_MASTER_STANDALONE_H__
#define __SRXL_MASTER_STANDALONE_H__

/**
 * @file srxl_master_standalone.h
 * @brief Standalone SRXL2 bus master -- wire-compatible with the official
 *        Spektrum types, zero shared global state.
 *
 * Unlike SRXL2_Master/ (which drives Spektrum's own spm_srxl.c engine and
 * therefore inherits its single process-wide device identity via
 * srxlThisDev/srxlBus[]), this module never includes or links spm_srxl.c.
 * It only uses spm_srxl.h for struct/enum *types*
 * (SrxlHandshakeData, SrxlChannelData, SrxlBindData, SrxlVtxData,
 * SrxlTelemetryData, SrxlDevEntry, SrxlFullID, SrxlState, ...) so that code
 * which already vendors the official headers can build packets and interact
 * with this master without adding a second, unrelated type system.
 *
 * All state lives in a caller-owned SrxlMasterCtx. Multiple instances (e.g.
 * one running this master while another process component runs the
 * upstream spm_srxl.c engine as a slave with a different device ID) can
 * coexist in the same process with zero shared globals.
 */

#include "spm_srxl.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Maximum number of peer devices tracked per master instance. */
#ifndef SRXL_MASTER_MAX_DEVICES
#define SRXL_MASTER_MAX_DEVICES SRXL_MAX_DEVICES
#endif

/* Opaque context -- one per bus master instance. */
typedef struct SrxlMasterCtx SrxlMasterCtx;

/* Platform callbacks (function pointers, set at init). */
typedef struct SrxlMasterHal
{
    void     (*uart_send)(void *user, const uint8_t *buf, uint8_t len);
    void     (*uart_set_baud)(void *user, uint32_t baud);
    uint32_t (*time_ms)(void *user);
    void     *user;
} SrxlMasterHal;

/* This device's identity (what today lives in the global srxlThisDev). */
typedef struct SrxlMasterDevice
{
    uint8_t  device_id;      /* e.g. 0x21 for a Receiver-role master */
    uint8_t  priority;       /* telemetry priority (1-100) */
    uint8_t  info;           /* SRXL_DEVINFO_xxx bits */
    uint32_t uid;            /* unique id for collision detection */
} SrxlMasterDevice;

/* Init configuration. */
typedef struct SrxlMasterConfig
{
    SrxlMasterDevice device;
    SrxlMasterHal    hal;
    uint8_t          baud_supported;  /* SRXL_BAUD_* flags */
} SrxlMasterConfig;

/* Event types fired to the application. */
typedef enum
{
    SRXL_MASTER_EVT_HANDSHAKE_COMPLETE, /* handshake scan finished, now Running */
    SRXL_MASTER_EVT_TELEMETRY,          /* telemetry received from a peer */
    SRXL_MASTER_EVT_BIND,               /* bind-related packet received */
    SRXL_MASTER_EVT_VTX,                /* VTX data received */
} SrxlMasterEventType;

typedef struct SrxlMasterEvent
{
    SrxlMasterEventType type;
    union
    {
        struct
        {
            uint8_t peer_count;
        } handshake;
        struct
        {
            uint8_t                  device_id;
            const SrxlTelemetryData *data;
        } telemetry;
        struct
        {
            uint8_t             request;
            uint8_t             device_id;
            const SrxlBindData *data;
        } bind;
        struct
        {
            const SrxlVtxData *data;
        } vtx;
    };
} SrxlMasterEvent;

typedef void (*SrxlMasterEventCb)(SrxlMasterCtx *ctx, const SrxlMasterEvent *evt,
                                   void *user);

/*---------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

/* Size needed for a static allocation buffer, for srxlMasterInitStatic(). */
size_t srxlMasterCtxSize(void);

/* Init with a caller-provided buffer (no malloc). NULL if buf too small. */
SrxlMasterCtx *srxlMasterInitStatic(uint8_t *buf, size_t buf_size,
                                     const SrxlMasterConfig *config);

/* Init with malloc(). NULL on failure. */
SrxlMasterCtx *srxlMasterInit(const SrxlMasterConfig *config);

/* Destroy (frees if malloc'd by srxlMasterInit()). */
void srxlMasterDestroy(SrxlMasterCtx *ctx);

/*---------------------------------------------------------------------------
 * Core loop
 *---------------------------------------------------------------------------*/

/* Buffer received UART bytes (ISR-safe, single producer). */
void srxlMasterFeed(SrxlMasterCtx *ctx, const uint8_t *data, size_t len);

/* Advance the state machine (call regularly from task context). */
void srxlMasterTick(SrxlMasterCtx *ctx);

/*---------------------------------------------------------------------------
 * Master control API
 *---------------------------------------------------------------------------*/

/* Set outgoing channel values for the given mask (merged into pending). */
void srxlMasterSetChannels(SrxlMasterCtx *ctx, const uint16_t *values,
                            uint32_t mask);

/* Enable/disable failsafe channel data output. */
void srxlMasterSetFailsafe(SrxlMasterCtx *ctx, bool failsafe);

/* Retrieve last known telemetry payload for a device, if any. */
bool srxlMasterGetTelemetry(SrxlMasterCtx *ctx, uint8_t device_id,
                             SrxlTelemetryData *data_out, uint32_t *age_ms_out);

void srxlMasterSetVtx(SrxlMasterCtx *ctx, const SrxlVtxData *vtx);

void srxlMasterSendFwdPgm(SrxlMasterCtx *ctx, uint8_t device_id,
                           const uint8_t *data, uint8_t len);

void srxlMasterEnterBind(SrxlMasterCtx *ctx, uint8_t bind_type, bool broadcast);

void srxlMasterSetBindInfo(SrxlMasterCtx *ctx, const SrxlBindData *data);

/*---------------------------------------------------------------------------
 * Query
 *---------------------------------------------------------------------------*/

bool        srxlMasterIsConnected(const SrxlMasterCtx *ctx);
uint8_t     srxlMasterPeerCount(const SrxlMasterCtx *ctx);
uint32_t    srxlMasterGetBaud(const SrxlMasterCtx *ctx);
const char *srxlMasterGetState(const SrxlMasterCtx *ctx);

/*---------------------------------------------------------------------------
 * Event registration
 *---------------------------------------------------------------------------*/

void srxlMasterOnEvent(SrxlMasterCtx *ctx, SrxlMasterEventCb cb, void *user);

/*---------------------------------------------------------------------------
 * CRC-16 (XMODEM, polynomial 0x1021, seed 0) -- vendored, not shared with
 * spm_srxl.c's static srxlCrc16().
 *
 * Compile-time modes:
 *   (default)               -- 256-entry table lookup
 *   SRXL_MASTER_CRC_SMALL   -- bitwise loop (0B extra flash)
 *   SRXL_MASTER_CRC_EXTERN  -- user provides this function externally;
 *                               see srxl_master_crc_stm32.c for an
 *                               STM32F7/H7 hardware-CRC example.
 *---------------------------------------------------------------------------*/

uint16_t srxlMasterCrc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SRXL_MASTER_STANDALONE_H__ */
