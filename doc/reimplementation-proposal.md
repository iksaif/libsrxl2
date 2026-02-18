# Proposal: Modern SRXL2 Stack for Flight Controllers

## Why Rewrite

The official Spektrum `spm_srxl.c` library was written for bare-metal firmware
running inside a receiver. It has a single-process, cooperative-scheduling
mindset that clashes with how iNav (ChibiOS) and ArduPilot (ChibiOS/NuttX)
actually work. Rather than wrapping it with ever-thicker adapter layers, a
purpose-built stack would be smaller, safer, and easier to maintain.

### Concrete problems with the current library

| Category | Issue |
|----------|-------|
| **Global state** | 17 mutable globals/statics shared across modules. `srxlChData`, `srxlTelemData`, `srxlBus[]`, `srxlRx`, `srxlThisDev`, `srxlChDataIsFailsafe`, etc. are all file-scope without `static`, accessed via raw `extern` from the master code. No two bus instances can exist independently. |
| **Thread safety** | Zero synchronization. `srxlParsePacket()` runs from UART ISR context, `srxlRun()` from a task context; both mutate the same bus struct, channel data, receiver stats, and bitfield flags. On any preemptive RTOS this produces data races. `srxlEnterCriticalSection()` exists but is only used in the STM32 HW-CRC path. |
| **Config-via-include** | `spm_srxl.h` includes `spm_srxl_config.h` mid-file, expecting the user to provide 9 `static inline` callback implementations plus compile-time constants. A second config header adds 13 more hooks for master mode. There is no interface spec; you discover what to implement by reading the source. |
| **Monolithic functions** | `srxlSend()` is 190 lines, `srxlParsePacket()` is 430 lines. They both parse/build *and* mutate state machine + receiver tracking + bind logic. No separation between decoding and acting on a packet. |
| **Circular coupling** | `spm_srxl.c` calls `srxlRunMaster()` from `spm_srxl_master.c`; the master calls `srxlSend()` (an internal function not in any header) and reads globals via `extern` declarations inside function bodies. Neither module compiles alone when master mode is enabled. |
| **`#ifdef` spaghetti** | `SRXL_INCLUDE_MASTER_CODE` appears in 12 sites *inside function bodies* of `spm_srxl.c`, creating two interleaved code paths through the same 430-line switch statement. |
| **Re-entrancy** | `srxlParsePacket()` calls `srxlRun()` for slaves; the handshake handler inside `srxlParsePacket` calls `srxlRun()` recursively. |
| **Packed struct aliasing** | All protocol types are packed, overlaid in a union. `SrxlHandshakeData.uid` sits at offset 5 (unaligned `uint32_t`). ARM Cortex-M0 will HardFault. |
| **Implicit endianness** | Protocol struct fields are accessed directly from the union buffer with no byte swap. Assumes little-endian host everywhere except CRC and telemetry payloads. |
| **Fixed array sizes** | `SRXL_NUM_OF_BUSES` and `SRXL_MAX_DEVICES` are compile-time constants baked into array dimensions. Cannot adapt at runtime. |
| **Unmaintained** | Last upstream commit was 2023. The master code was never open-sourced by Spektrum; our `SRXL2_Master/` is a from-scratch implementation. ArduPilot's PR #13923 drew criticism for "cut&paste" integration. iNav rewrote the slave from scratch in `src/main/rx/srxl2.c`. |

### What iNav and ArduPilot do today

**iNav** reimplemented the slave protocol from scratch (~600 lines in
`rx/srxl2.c`). It registers as a FlightController, does its own packet
building, manages half-duplex, and never touches `spm_srxl.c`. No master
support.

**ArduPilot** wraps the official `spm_srxl.c` behind a C++ `AP_RCProtocol_SRXL2`
class. It had to work around the globals and config-header pattern. Tridge
flagged architectural issues during review (PR #13923). Slave only; Smart ESC
support is an open feature request (issue #27603) with no implementation.

Neither project can act as a bus master. Both need it to control Spektrum
Smart ESCs and read Smart Battery telemetry.


## Design Goals

1. **Context-based API** -- no globals. Every function takes a context pointer. Multiple independent bus instances coexist.
2. **Separation of parsing and state** -- pure decode functions (no side effects) + state machine that consumes decoded events.
3. **RTOS-friendly** -- well-defined ownership of data. ISR-safe packet ingestion, task-level state machine processing. No re-entrancy.
4. **Both master and slave** in the same binary, selected per-bus at init.
5. **Portable** -- no packed struct aliasing, explicit byte-order conversion, no platform-specific `#ifdef` in the core.
6. **Small** -- target ~1500 lines for the full stack (iNav's slave alone is ~600; MSRC's master is ~300). The official lib + our master is ~2000 lines plus config boilerplate.


## Proposed Architecture

```
                       User code (FC firmware)
                              |
              +---------------+---------------+
              |                               |
         srxl2_init()                    srxl2_init()
         role=MASTER                     role=SLAVE
         bus 0, UART A                   bus 1, UART B
              |                               |
              v                               v
    +-------------------+          +-------------------+
    |   srxl2_ctx_t     |          |   srxl2_ctx_t     |
    |   (master state)  |          |   (slave state)   |
    +-------------------+          +-------------------+
              |                               |
    +---------+---------+          +---------+---------+
    | srxl2_feed()      |          | srxl2_feed()      |
    | srxl2_tick()      |          | srxl2_tick()      |
    | srxl2_set_chan()   |          | srxl2_get_chan()   |
    | srxl2_get_telem() |          | srxl2_set_telem() |
    +---------+---------+          +---------+---------+
              |                               |
              v                               v
    +-------------------+          +-------------------+
    |   HAL callbacks   |          |   HAL callbacks   |
    |   (fn pointers)   |          |   (fn pointers)   |
    +-------------------+          +-------------------+
    | .uart_send()      |          | .uart_send()      |
    | .uart_set_baud()  |          | .uart_set_baud()  |
    | .time_ms()        |          | .time_ms()        |
    +-------------------+          +-------------------+
```

### Core types

```c
// Opaque context -- one per bus. All state lives here, no globals.
typedef struct srxl2_ctx srxl2_ctx_t;

// Role
typedef enum { SRXL2_ROLE_MASTER, SRXL2_ROLE_SLAVE } srxl2_role_t;

// Platform callbacks (function pointers, set at init)
typedef struct {
    void (*uart_send)(void *user, const uint8_t *buf, uint8_t len);
    void (*uart_set_baud)(void *user, uint32_t baud);
    uint32_t (*time_ms)(void *user);
    void *user;  // passed back to every callback
} srxl2_hal_t;

// Device descriptor
typedef struct {
    uint8_t  device_id;    // e.g. 0x10 for master, 0x31 for slave FC
    uint8_t  priority;     // telemetry priority (1-100)
    uint8_t  info;         // SRXL_DEVINFO_* bits
    uint32_t uid;          // unique ID for collision detection
} srxl2_device_t;

// Init config
typedef struct {
    srxl2_role_t   role;
    srxl2_device_t device;
    srxl2_hal_t    hal;
    uint8_t        baud_supported;  // SRXL2_BAUD_115200 | SRXL2_BAUD_400000
} srxl2_config_t;

// Telemetry callback (master receives from slaves, slave fills for master)
typedef void (*srxl2_telem_cb_t)(srxl2_ctx_t *ctx, uint8_t device_id,
                                  const uint8_t payload[16], void *user);

// Channel data callback (slave receives from master)
typedef void (*srxl2_channel_cb_t)(srxl2_ctx_t *ctx, const uint16_t *values,
                                    uint32_t mask, bool is_failsafe, void *user);
```

### Public API (~15 functions)

```c
// Lifecycle
srxl2_ctx_t *srxl2_init(const srxl2_config_t *config);
void         srxl2_destroy(srxl2_ctx_t *ctx);

// Feed raw UART bytes (call from ISR or DMA callback -- just buffers data)
void srxl2_feed(srxl2_ctx_t *ctx, const uint8_t *data, size_t len);

// Advance state machine (call from task context, typically every 1ms)
void srxl2_tick(srxl2_ctx_t *ctx);

// Master: set outgoing channel data
void srxl2_set_channels(srxl2_ctx_t *ctx, const uint16_t *values,
                         uint32_t mask);

// Master: enter failsafe (sends CHANNEL_FS with current values)
void srxl2_set_failsafe(srxl2_ctx_t *ctx, bool failsafe);

// Master: get latest telemetry from a device (returns false if stale)
bool srxl2_get_telemetry(srxl2_ctx_t *ctx, uint8_t device_id,
                          uint8_t payload_out[16], uint32_t *age_ms);

// Slave: get latest channel data (returns false if no data yet)
bool srxl2_get_channels(srxl2_ctx_t *ctx, uint16_t *values_out,
                         uint32_t *mask_out, bool *is_failsafe);

// Slave: set telemetry payload (called before master polls us)
void srxl2_set_telemetry(srxl2_ctx_t *ctx, const uint8_t payload[16]);

// Register callbacks (optional, for event-driven usage)
void srxl2_on_telemetry(srxl2_ctx_t *ctx, srxl2_telem_cb_t cb, void *user);
void srxl2_on_channels(srxl2_ctx_t *ctx, srxl2_channel_cb_t cb, void *user);

// Query state
bool     srxl2_is_connected(srxl2_ctx_t *ctx);
uint8_t  srxl2_device_count(srxl2_ctx_t *ctx);
uint32_t srxl2_get_baud(srxl2_ctx_t *ctx);
```

### What is NOT in the public API

- CRC computation (internal)
- Packet construction (internal)
- State enum (internal -- user checks `srxl2_is_connected()`, not `state == Running`)
- Handshake details (automatic)
- Telemetry scheduling (automatic, based on device priorities)


## Internal Design

### Packet layer (pure functions, no state)

```c
// srxl2_packet.c -- ~200 lines
// Decode raw bytes into typed structs. No side effects.
srxl2_parse_result_t srxl2_pkt_decode(const uint8_t *raw, uint8_t len,
                                       srxl2_decoded_t *out);

// Encode typed structs into wire bytes with CRC.
uint8_t srxl2_pkt_encode_handshake(uint8_t *buf, const srxl2_handshake_t *hs);
uint8_t srxl2_pkt_encode_channel(uint8_t *buf, const srxl2_channel_t *ch);
uint8_t srxl2_pkt_encode_telemetry(uint8_t *buf, const srxl2_telemetry_t *tm);

// CRC
uint16_t srxl2_crc16(const uint8_t *data, size_t len);
```

All multi-byte wire fields are read/written with explicit helpers:

```c
static inline uint16_t rd_be16(const uint8_t *p) { return (p[0]<<8)|p[1]; }
static inline uint16_t rd_le16(const uint8_t *p) { return p[0]|(p[1]<<8); }
static inline void wr_be16(uint8_t *p, uint16_t v) { p[0]=v>>8; p[1]=v; }
```

No packed structs in the wire path. Decode into plain host-endian structs.

### State machine (~400 lines)

```c
// srxl2_sm.c
// Single entry point per tick. Returns actions for the caller to execute.

typedef enum {
    SRXL2_STATE_STARTUP,      // waiting 50ms
    SRXL2_STATE_HANDSHAKE,    // discovering devices (master) or waiting (slave)
    SRXL2_STATE_RUNNING,      // normal operation
} srxl2_state_t;

// Internal to ctx:
struct srxl2_ctx {
    srxl2_config_t  config;
    srxl2_state_t   state;
    uint32_t        last_rx_ms;
    uint32_t        last_tx_ms;
    uint32_t        state_entered_ms;

    // Device table (discovered peers)
    srxl2_peer_t    peers[SRXL2_MAX_PEERS];  // default 8, configurable
    uint8_t         peer_count;

    // Channel data (double-buffered for ISR safety)
    srxl2_channel_buf_t  chan_tx;  // master: outgoing
    srxl2_channel_buf_t  chan_rx;  // slave: incoming

    // Telemetry (ring per peer)
    uint8_t         telem_payload[SRXL2_MAX_PEERS][16];
    uint32_t        telem_age_ms[SRXL2_MAX_PEERS];

    // Handshake state (master)
    uint8_t         hs_scan_idx;
    uint8_t         hs_scan_table[12]; // default IDs to scan
    uint8_t         hs_scan_count;
    uint8_t         negotiated_baud;

    // RX buffer (fed from ISR, consumed in tick)
    uint8_t         rx_buf[SRXL2_MAX_PACKET_SIZE];
    uint8_t         rx_len;
    bool            rx_ready;  // set by feed(), cleared by tick()

    // TX buffer
    uint8_t         tx_buf[SRXL2_MAX_PACKET_SIZE];

    // Telemetry scheduling (master)
    uint8_t         telem_poll_idx;

    // Callbacks
    srxl2_telem_cb_t   telem_cb;
    void              *telem_cb_user;
    srxl2_channel_cb_t channel_cb;
    void              *channel_cb_user;
};
```

The state machine is a flat switch in `srxl2_tick()`:

```
STARTUP:
  if time_since_init >= 50ms:
    if role == MASTER:
      -> HANDSHAKE (begin scan)
    else if unit_id == 0:
      send unprompted handshake, -> HANDSHAKE
    else:
      -> HANDSHAKE (listen)

HANDSHAKE (master):
  if pending_rx: process handshake response, register peer
  if scan not done: send next handshake, advance scan index
  if scan done: send broadcast handshake, switch baud, -> RUNNING

HANDSHAKE (slave):
  if pending_rx with our dest: reply with handshake
  if broadcast received: switch baud, -> RUNNING
  if timeout 200ms: -> STARTUP

RUNNING (master):
  if pending_rx: process telemetry / rehandshake-request
  if time_since_tx >= frame_period:
    select telem_reply_id (priority-weighted round-robin)
    send channel data (or failsafe)
    age telem counters

RUNNING (slave):
  if pending_rx:
    if channel data: update chan_rx, invoke callback
    if reply_id == us: send telemetry
  if timeout 50ms: reset baud, -> STARTUP
```

### ISR / task boundary

The only function safe to call from ISR context is `srxl2_feed()`, which
copies bytes into `rx_buf` and sets `rx_ready`. Everything else runs in
`srxl2_tick()` from task context.

If the platform needs the received-channel callback to fire from task context
too, that happens naturally since `srxl2_tick()` is the one invoking it.

For the UART transmit side, `hal.uart_send()` is called from `srxl2_tick()`
which is always task context. The HAL can use DMA/blocking/whatever.


## Handshake scan (master)

Instead of scanning all 240 IDs (0x10..0xFE), scan only the default ID per
device type, matching what real Spektrum receivers do:

```c
static const uint8_t default_scan_ids[] = {
    0x21,  // Receiver
    0x30,  // Flight Controller
    0x40,  // ESC 0
    0x41,  // ESC 1
    0x42,  // ESC 2
    0x43,  // ESC 3
    0x60,  // Servo 1
    0x70,  // Servo 2
    0x81,  // VTX
    0xB0,  // Sensor
};
```

Send one handshake per ID with a ~10ms wait for response. Total handshake
time: ~100ms (vs. ~12 seconds for the 240-ID scan).


## Telemetry scheduling

Priority-weighted aging, same algorithm as our current `spm_srxl_master.c` but
cleaner:

```
For each frame:
  score[i] = peer[i].priority * peer[i].age
  pick peer with highest score
  reset that peer's age to 0
  increment all others' age
```

A device with priority 30 gets polled ~3x more often than one with priority 10.


## File layout

```
libsrxl2/
  srxl2.h              -- public API (types, init, tick, set/get)
  srxl2.c              -- state machine (~400 lines)
  srxl2_packet.h       -- internal: packet encode/decode protos
  srxl2_packet.c       -- internal: packet codec + CRC (~200 lines)
  srxl2_internal.h     -- internal: context struct, constants
```

Plus the existing standalone libraries that don't change:

```
libsrxl2parser/        -- read-only packet parser (for sniffer)
  srxl2_parser.c/h
  srxl2_telemetry.c/h  -- telemetry payload decoder
```


## Migration path

### Phase 1: Build `libsrxl2` with master support

Write the new stack. Test it against the existing test suite (adapt
`tests/test_master.c` and `tests/test_slave.c` to the new API). The test
harness and packet builders from `test_helpers.c` are reusable.

Validate against a real Smart ESC using the sniffer to compare traffic.

### Phase 2: Replace the simulator

Rewrite `srxl2_master_sim.c` to use `libsrxl2` instead of `spm_srxl.c` +
`spm_srxl_master.c`. This is the proof that the new stack works end-to-end.

### Phase 3: iNav integration

iNav already has a custom SRXL2 slave in `rx/srxl2.c`. The integration would
add a second bus instance with `SRXL2_ROLE_MASTER` for ESC control:

```c
// In esc_srxl2.c (new file)
static srxl2_ctx_t *esc_ctx;

void escSrxl2Init(serialPort_t *port) {
    srxl2_config_t cfg = {
        .role = SRXL2_ROLE_MASTER,
        .device = { .device_id = 0x10, .priority = 20 },
        .hal = {
            .uart_send = inav_uart_send,
            .uart_set_baud = inav_uart_set_baud,
            .time_ms = inav_time_ms,
            .user = port,
        },
        .baud_supported = SRXL2_BAUD_400000,
    };
    esc_ctx = srxl2_init(&cfg);
}

// Called every 1ms from scheduler
void escSrxl2Process(void) {
    // Feed any received bytes
    uint8_t buf[80];
    int n = serialRxBytesWaiting(port);
    if (n > 0) {
        serialRead(port, buf, n);
        srxl2_feed(esc_ctx, buf, n);
    }
    srxl2_tick(esc_ctx);
}

// Called from mixer at output time
void escSrxl2WriteMotors(uint16_t *values, uint8_t count) {
    uint32_t mask = (1u << count) - 1;
    srxl2_set_channels(esc_ctx, values, mask);
}
```

### Phase 4: ArduPilot integration

ArduPilot wraps everything in C++ classes. The new stack would sit behind an
`AP_SRXL2_Master` class similar to existing `AP_FETtecOneWire`:

```cpp
class AP_SRXL2_Master : public AP_ESC_Telem_Backend {
    srxl2_ctx_t *ctx;
    void update() override;          // calls srxl2_tick()
    void write_motor(uint8_t idx, uint16_t pwm) override;
    // telemetry forwarded to AP_ESC_Telem
};
```


## Size estimates

| Component | Lines (est.) | vs. current |
|-----------|-------------|-------------|
| `srxl2.c` (state machine) | ~400 | `spm_srxl.c` is 1563 + `spm_srxl_master.c` is 439 |
| `srxl2_packet.c` (codec + CRC) | ~200 | embedded in `spm_srxl.c` |
| `srxl2.h` (public API) | ~100 | `spm_srxl.h` is 581 + 2 config headers |
| `srxl2_internal.h` | ~80 | n/a |
| **Total** | **~780** | **~2600** (official lib + master + configs) |

The reduction comes from: no `#ifdef` variants, no bind/VTX/forward-programming
(can be added later behind the same API), no multi-bus hub logic (each ctx is
independent), no receiver-sorting machinery (FCs don't need it).


## What we intentionally omit (for now)

These features exist in the Spektrum library but are not needed for the
FC-as-bus-master use case:

- **Bind protocol** (0x41) -- ESCs don't bind, receivers do
- **VTX control** (0x02) -- separate VTX protocol, not relevant
- **Forward Programming** (0x03) -- ESC configuration tool, not runtime
- **Parameter Config** (0x50) -- device parameter query, not runtime
- **RSSI packets** (0x55) -- receiver-to-transmitter, not relevant
- **Multi-bus hub** -- each bus is an independent `srxl2_ctx_t`
- **STM32 HW-CRC** -- software CRC is fast enough on F4/F7/H7

All of these can be added later as optional features without changing the core
API, since they're just additional packet types handled in the state machine.


## References

- [SpektrumRC/SRXL2](https://github.com/SpektrumRC/SRXL2) -- official library (MIT license)
- [dgatf/msrc](https://github.com/dgatf/msrc) -- RP2040 SRXL2 slave + Smart ESC master in ~300 lines
- [ArduPilot PR #13923](https://github.com/ArduPilot/ardupilot/pull/13923) -- SRXL2 slave integration (Tridge's review comments on architecture)
- [ArduPilot issue #27603](https://github.com/ArduPilot/ardupilot/issues/27603) -- Smart ESC feature request
- [iNav `rx/srxl2.c`](https://github.com/iNavFlight/inav/blob/master/src/main/rx/srxl2.c) -- custom slave reimplementation
- [ArduPilot forum: Smart ESCs and SRXL2](https://discuss.ardupilot.org/t/spektrum-smart-escs-and-srxl2/95104)
- `doc/master-analysis.md` -- our bug analysis and roadmap for the current stack
- `doc/msrc-analysis.md` -- how MSRC implements SRXL2 master
