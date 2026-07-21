# SRXL2 Master Standalone

A self-contained SRXL2 **bus master** that is wire- and type-compatible with
Spektrum's official `spm_srxl.h`, has **zero shared global state**, and never
links `spm_srxl.c`.

## Why this exists

The obvious way to add a master to a project that already vendors Spektrum's
library is `SRXL2_Master/` (the sibling module in this repo): it plugs
`srxlRunMaster()` into the official `spm_srxl.c` engine. That works great when
the master is the *only* SRXL2 role in your process.

It does **not** work when you need a master **alongside** an already-running
`spm_srxl.c` engine that is acting as a **slave** — because `spm_srxl.c` keeps
its device identity in a single process-wide global:

```c
// spm_srxl.c
SrxlDevice srxlThisDev;          // one identity for the whole process
...
// srxlInitBus():
pBus->fullID.deviceID = srxlThisDev.devEntry.deviceID;   // every bus gets it
```

This is exactly ArduPilot's situation (issue
[ardupilot#27603](https://github.com/ArduPilot/ardupilot/issues/27603)): the
flight controller already runs `spm_srxl.c` as a **slave** (FlightController,
`0x30`/`0x31`) to talk to a Spektrum receiver, and separately needs to be a bus
**master** (Receiver, `0x21`) on another UART to drive a Smart ESC. One global
identity cannot be both at once, and they don't want to add a second, unrelated
SRXL2 type system next to the Spektrum headers they already use.

**This module solves both problems:**

1. **Zero shared global state.** All state lives in a caller-owned
   `SrxlMasterCtx`. Any number of instances — including an unrelated
   `spm_srxl.c` engine acting as a slave with a *different* device ID — can
   run in the same process on different UARTs with no interference.
2. **No new header dependency.** It `#include`s `spm_srxl.h` for the packet
   *types* only (`SrxlChannelData`, `SrxlBindData`, `SrxlVtxData`,
   `SrxlTelemetryData`, `SrxlHandshakeData`, `SrxlPacket`, `SrxlState`, …).
   Code that already uses those structs needs no new types. It never
   references `srxlThisDev`, `srxlBus[]`, `srxlSend()`, `srxlRun()`, or any
   `SRXL_INCLUDE_MASTER_CODE` path.

## What the bus master does

1. **Device discovery** — handshakes across the default device-type ID scan
   table, then broadcasts to move everyone to Running.
2. **Channel data** — sends RC channel values every ~11 ms (6 ms at 400 k baud).
3. **Telemetry polling** — asks one slave per frame to reply, using
   priority × age scheduling so higher-priority devices get polled more often
   without starving the rest.
4. **Baud negotiation** — ANDs every peer's supported baud during handshake,
   then switches the bus to 400000 if all peers agree.
5. **VTX / bind / forward-programming** — one-shot control packets on request.

## Files

| File | Description |
|------|-------------|
| `srxl_master_standalone.h` | Public API (context lifecycle, feed/tick, channel/telemetry/bind/VTX, events) |
| `srxl_master_standalone.c` | State machine + packet builders + vendored CRC |
| `srxl_master_crc_table.inc` | Vendored CRC-16/XMODEM table (own copy, no dependency on `spm_srxl.c`) |
| `srxl_master_crc_stm32.c` | Optional `SRXL_MASTER_CRC_EXTERN` hardware-CRC hook for STM32F7/H7 |
| `spm_srxl_config.h` | Minimal config so `spm_srxl.h` (types only) parses; defines no engine hooks |
| `examples/coexist_sim.c` | Runs this master **and** a real `spm_srxl.c` slave in one process |

## API sketch

```c
#include "srxl_master_standalone.h"

static void on_event(SrxlMasterCtx *ctx, const SrxlMasterEvent *e, void *u) {
    if (e->type == SRXL_MASTER_EVT_TELEMETRY)
        handle_telem(e->telemetry.device_id, e->telemetry.data);  // SrxlTelemetryData*
}

SrxlMasterConfig cfg = {
    .device = { .device_id = 0x21, .priority = 20,
                .info = SRXL_DEVINFO_TELEM_TX_ENABLED, .uid = my_uid },
    .hal = { .uart_send = my_uart_send, .uart_set_baud = my_set_baud,
             .time_ms = my_millis, .user = my_ctx },
    .baud_supported = SRXL_BAUD_400000,
};

SrxlMasterCtx *m = srxlMasterInit(&cfg);   // or srxlMasterInitStatic(buf, size, &cfg)
srxlMasterOnEvent(m, on_event, NULL);

// In your ISR / RX path:
srxlMasterFeed(m, rx_bytes, rx_len);       // ISR-safe, single producer

// In your task loop (regularly):
srxlMasterTick(m);
uint16_t values[4] = { 1000, 2000, 3000, 4000 };
srxlMasterSetChannels(m, values, 0x0F);
```

`srxlMasterInitStatic()` takes a caller-provided buffer (size from
`srxlMasterCtxSize()`) for no-malloc embedded targets.

## CRC

The module carries its own CRC-16/XMODEM (poly `0x1021`, seed 0) —
bit-for-bit identical to `spm_srxl.c`'s `srxlCRCTable`, but a separate copy so
there is no build-time dependency on `spm_srxl.c` (whose `srxlCrc16()` is
`static`). Compile-time modes:

| Mode | Effect |
|------|--------|
| *(default)* | 256-entry table lookup (~512 B flash) |
| `-DSRXL_MASTER_CRC_SMALL` | Bitwise loop, 0 B extra flash |
| `-DSRXL_MASTER_CRC_EXTERN` | You provide `srxlMasterCrc16()`; add `srxl_master_crc_stm32.c` for STM32F7/H7 hardware CRC |

## Building (as part of this repo)

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build

# Coexistence demo — standalone master (0x21) + real spm_srxl.c slave (0x40):
./build/srxl2_coexist_sim

# Tests:
ctest --test-dir build --output-on-failure -R "standalone|coexist"
```

- `test_scenarios_standalone` runs the shared master scenarios against this
  module's API.
- `test_master_standalone_coexist` links the **real** `spm_srxl.c` and proves,
  at the byte level, that (a) packets this master emits are decoded by the
  upstream parser, (b) this master decodes genuine upstream-format packets, and
  (c) two masters with different IDs — plus a live `spm_srxl.c` slave with a
  third ID — coexist in one process without touching each other's state.

## Endianness

Like `spm_srxl.c`'s own `srxlSend()`, packet fields are filled by direct
assignment into the packed `spm_srxl.h` structs, so correct wire bytes are
produced on little-endian hosts (every ARM Cortex-M/A and x86 target). Only the
CRC is written as explicit big-endian bytes, matching the wire spec.

## Relationship to the other modules

| Module | Role model | Global state | Links `spm_srxl.c`? |
|--------|-----------|--------------|---------------------|
| `SRXL2/` | Official slave engine | `srxlThisDev`, `srxlBus[]` | — (is the engine) |
| `SRXL2_Master/` | Master via official engine | shares the engine's globals | yes |
| `libsrxl2/` | Clean-room master **or** slave | none (context-based) | no — own type system |
| **`SRXL2_Master_Standalone/`** | **Master only** | **none (context-based)** | **no — reuses `spm_srxl.h` *types***|

Use this one when you need a master that coexists with the official engine and
speaks its types. Use `libsrxl2/` when you want a fresh, dependency-free stack
(master or slave) and are happy with its own type system.

## License

MIT — see [LICENSE](../LICENSE).

The official Spektrum SRXL2 headers (`SRXL2/Source/`) are separately licensed by
Horizon Hobby, LLC under MIT.
