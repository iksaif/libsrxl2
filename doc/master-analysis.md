# SRXL2 Master Implementation Analysis

## Goal

Build a flight controller (iNav/ArduPilot) that **is** the SRXL2 bus master — with full authority over its slave devices:
- Send throttle/channel data to Spektrum Smart ESCs
- Read telemetry from Smart ESCs (RPM, voltage, current, temperature)
- Read data from Smart Batteries
- The FC owns the bus. No Spektrum receiver in the loop. The FC directly controls ESCs the same way a Spektrum receiver would.

This is **not** about pretending or faking — the FC is the real bus master. The SRXL2 spec allows any device with the lowest ID on the bus to be master. The FC takes that role.

## Current State

ArduPilot and iNav only implement SRXL2 **slave** mode (receiving channels from a Spektrum receiver). Neither can act as a bus master to control ESCs directly. Our custom `SRXL2_Master/` library provides the missing `srxlRunMaster()` implementation that the official Spektrum library expects but doesn't ship.

## How SRXL2 Master-Slave Works (Spec Summary)

### Bus Topology
```
                     SRXL2 bus (single half-duplex UART wire)
                     |
Master (0x10)--------+--------ESC (0x40)--------Battery Sensor (0xB0)--------...
                     |
              All devices share one wire.
              Only one device talks at a time.
              Master controls the bus cadence.
```

### Normal Operation Cycle (every 11ms or 5.5ms)

```
Time 0ms:     Master sends Control Data packet (channel values + ReplyID)
              - ReplyID = device ID of slave that should reply with telemetry
              - ReplyID = 0x00 means no reply wanted
              - Channel data includes throttle for ESC

Time ~1-3ms:  Slave matching ReplyID sends Telemetry packet (16 bytes)
              - ESC sends STRU_TELE_ESC (RPM, voltage, current, temp...)
              - Battery sends STRU_TELE_FP_MAH or STRU_TELE_SMARTBATT

Time ~3-11ms: Bus idle until next frame

Time 11ms:    Master sends next Control Data packet
              - ReplyID cycles through devices based on priority
```

### Handshake Sequence (startup)

```
1. Master waits 50ms after reset (ListenOnStartup)
2. Master polls each device ID with Handshake packets:
   - Sends Handshake to 0x40 (ESC), waits for reply
   - Sends Handshake to 0x41 (ESC 2), waits for reply (or timeout)
   - ... etc for all expected device types
3. Master broadcasts final Handshake (dest=0xFF) with negotiated baud rate
4. All devices switch to negotiated baud rate
5. Normal operation begins
```

## Critical Issues in Current Implementation

### P0 - Frame Timing (BROKEN)

**Spec**: Master must send channel data every 11ms (or 5.5ms at 400kbaud).

**Current** (`srxl2_master_sim.c:296`): Channel data pushed every **10 seconds**. The state machine in `spm_srxl_master.c:277` sends when `timeoutCount_ms >= 10`, but without `channelOutMask` being set continuously, channel data isn't actually sent.

**Impact**: ESCs will enter failsafe/disarm within ~1 second of no channel data.

**Fix**: The main loop must call `srxlSetOutgoingChannelMask()` every 11ms, or the master state machine should send channel data unconditionally on a timer regardless of mask state.

### P0 - Debug printf in srxlSend (BROKEN)

**Location**: `spm_srxl.c:536-540`

```c
if (channelMask != 0) {
    printf("Channel %d: %d %x -> %x\n", i, channelMaskBit, channelMask & channelMaskBit, channelMask);
}
```

These printfs are in the hot path of the **original library's** channel data send. On real hardware, printf over UART will block for milliseconds and destroy bus timing completely. Must be removed or guarded by a debug flag.

### P0 - No Failsafe Handling

**Current**: No mechanism to enter failsafe mode (send `SRXL_CTRL_CMD_CHANNEL_FS` with throttle=0).

**Impact**: If the FC loses its own RC input, it has no way to signal ESCs to stop. This is a **safety issue**.

**Required**:
- Detect loss of control input (RC link)
- Set `srxlChDataIsFailsafe = true`
- Send failsafe channel data with throttle = 0, other channels at safe values
- ESCs should then use their own failsafe behavior (typically motor stop)

### P1 - Handshake Scans All 240 Device IDs

**Current** (`spm_srxl_master.c:136-154`): Scans from 0x10 to 0xFF sequentially.

**Impact**: At ~50ms per unanswered handshake, this takes ~12 seconds. A real receiver only scans default IDs per type.

**Fix**: Only scan known device types:
```
0x10       Remote Receiver (self)
0x21       Receiver
0x30       Flight Controller
0x40-0x43  ESCs (up to 4)
0x81       VTX
0xB0       Sensor
```

This reduces handshake to ~0.5 seconds.

### P1 - Handshake Doesn't Wait for Responses

**Current**: `continueHandshake()` is called immediately after sending, advancing `requestID` without waiting for the slave to reply.

**What should happen**: The `srxlParsePacket()` handler for handshake responses (`spm_srxl.c:919-926`) sets `pBus->state = SrxlState_SendHandshake` to continue. But the master code advances `requestID` in the same call that sends, so by the time the response arrives, `requestID` has already moved to the next device.

**In practice**: This mostly works because responses are parsed via `srxlParsePacket()` which calls `srxlAddDeviceEntry()` regardless of `requestID`. But it means the master doesn't properly handle the "poll, wait, advance" cadence the spec describes.

### P1 - Baud Rate Change Missing on Master Side

**Current**: After broadcasting the final handshake with the negotiated baud rate, the master never calls `srxlChangeBaudRate()` itself.

**Slave side** (`spm_srxl.c:944-948`): Slaves correctly switch on broadcast handshake.

**Master side**: Missing. If the negotiated rate is 400000, the master keeps transmitting at 115200.

**Impact**: Can't use 400kbaud, which means can't run at 5.5ms frame rate.

### P1 - No Telemetry Payload Parsing

**Current** (`srxl2_master_sim.c:252-259`): Telemetry packets are counted but the 16-byte payload is never decoded.

**Required for ESC control**: Parse `STRU_TELE_ESC` (sensor ID 0x20):
```c
typedef struct {
    UINT8   identifier;     // 0x20
    UINT8   sID;            // Secondary ID
    UINT16  RPM;            // 10 RPM/count, BIG-ENDIAN
    UINT16  voltsInput;     // 0.01V/count
    UINT16  tempFET;        // 0.1C/count
    UINT16  currentMotor;   // 10mA/count
    UINT16  tempBEC;        // 0.1C/count
    UINT8   currentBEC;     // 100mA/count
    UINT8   voltsBEC;       // 0.05V/count
    UINT8   throttle;       // 0.5%/count
    UINT8   powerOut;       // 0.5%/count
} STRU_TELE_ESC;           // 0xFFFF = no data for 16-bit, 0xFF for 8-bit
```

**Required for Smart Battery**: Parse `STRU_TELE_FP_MAH` (sensor ID 0x34):
```c
typedef struct {
    UINT8   identifier;     // 0x34
    UINT8   sID;
    INT16   current_A;      // 0.1A/count
    INT16   chargeUsed_A;   // 1mAh/count
    UINT16  temp_A;         // 0.1C/count (0x7FFF = not populated)
    INT16   current_B;
    INT16   chargeUsed_B;
    UINT16  temp_B;
    UINT16  spare;
} STRU_TELE_FP_MAH;
```

Also potentially: `STRU_TELE_SMARTBATT` (0x42), `STRU_TELE_LIPOMON` (0x3A), `STRU_TELE_RX_MAH` (0x18).

**Critical note**: ESC telemetry uses **big-endian** byte order. Must byte-swap all 16-bit fields when parsing on a little-endian host.

### P1 - srxlOnMasterFrame vs Main Loop Conflict

Both `srxlOnMasterFrame()` (called from state machine) and `main()` write to `srxlChData` and call `srxlSetOutgoingChannelMask()`. These conflict. Only one code path should manage channel data.

**Fix**: Remove channel management from `main()`. Use `srxlOnMasterFrame()` as the single point where channel data is updated from the FC's mixer output.

## Missing Features

### No Multi-ESC Channel Mapping

For a quadcopter, 4 ESCs need 4 throttle channels. Each ESC reads a specific channel from the channel data packet:
- ESC 0x40 reads Channel 1 (throttle)
- ESC 0x41 reads Channel 2
- etc.

The current implementation has no mapping from FC motor outputs to SRXL2 channel indices. This needs to be defined and configurable.

### No Forward Programming (SRXL_INCLUDE_FWD_PGM_CODE)

Forward Programming lets you configure ESC parameters (RPM limits, brake, timing, governor mode) through the SRXL2 bus. Not critical for basic operation, but useful for setup.

### No Parameter Configuration (Packet Type 0x50)

Allows querying/setting device parameters. The original library has `TODO: Add later` stubs. Useful for configuring ESC telemetry reporting units, but not blocking.

### Device ID Choice

Current: `0x10` (Remote Receiver). Per the spec, the bus master is "the RF telemetry receiver or remote receiver with the **lowest device ID** on the bus." The lowest ID wins master election automatically.

Using `0x10` is the right choice for our use case:
- It guarantees master election (lowest possible device type ID)
- The FC **is** the master — this is its dedicated ESC bus, no Spektrum receiver present
- Smart ESCs don't care about the device type of the master, they only care about receiving channel data and being polled for telemetry

If the FC also needs to be a slave on a **separate** SRXL2 bus (receiving RC from a Spektrum receiver), that would use a different UART with device ID `0x31` (FC slave). The two buses are independent.

### No RSSI Alternation

Per spec, the `rssi` field in channel data alternates between dBm (negative) and percent (positive). Some devices use this to determine telemetry phase. Current code always sends `-50`. Should alternate, e.g.: frame N sends -50 dBm, frame N+1 sends 75%.

## Telemetry Sensor Types Relevant to Our Goal

| Sensor ID | Type | Struct | Data Available |
|-----------|------|--------|---------------|
| 0x20 | ESC | `STRU_TELE_ESC` | RPM, Voltage, Current, FET/BEC Temp, Throttle, Power |
| 0x34 | Flight Battery | `STRU_TELE_FP_MAH` | Current (2ch), Charge Used, Temperature |
| 0x42 | Smart Battery | `STRU_TELE_SMARTBATT` | (Spektrum proprietary, struct not public) |
| 0x18 | Rx Pack Capacity | `STRU_TELE_RX_MAH` | Current (2ch), Charge, Voltage, Alerts |
| 0x3A | LiPo Cell Monitor | `STRU_TELE_LIPOMON` | 6 cell voltages, Temperature |
| 0x3F | LiPo 14S Monitor | `STRU_TELE_LIPOMON_14` | 14 cell voltages |
| 0x7E | RPM/Volts/Temp | `STRU_TELE_RPM` | RPM, Voltage, Temperature, RSSI |
| 0x7F | QoS | `STRU_TELE_QOS` | Fades (4 antenna), Frame losses, Holds, Rx Voltage |

## Implementation Roadmap

### Phase 1: Fix Core Master (simulator)
1. Fix 11ms frame timing
2. Remove debug printfs from spm_srxl.c
3. Target handshake scan (only default IDs)
4. Add baud rate switch on master side after broadcast handshake
5. Fix srxlOnMasterFrame to be the sole channel data path

### Phase 2: Add Telemetry Parsing
1. Build telemetry decoder for STRU_TELE_ESC
2. Add STRU_TELE_FP_MAH / STRU_TELE_RX_MAH decoder
3. Handle big-endian byte swapping
4. Print/log decoded telemetry in sniffer/simulator

### Phase 3: ESC Control
1. Map FC motor outputs to SRXL2 channels
2. Implement failsafe (throttle=0 on control loss)
3. Test with real Spektrum Smart ESC on serial bus
4. Validate timing with logic analyzer

### Phase 4: Integration
1. Port master code to ArduPilot/iNav HAL
2. Implement as second SRXL2 bus (slave on receiver bus, master on ESC bus)
3. Expose ESC telemetry to FC telemetry system
4. Add Smart Battery telemetry passthrough

## Reference: How ArduPilot/iNav Currently Handle SRXL2

Both act as **SRXL2 slaves only**:
- Receive RC channel data from a Spektrum receiver (the master)
- Send telemetry back to the receiver for display on the transmitter
- Device type: Flight Controller (0x30)
- Cannot control ESCs via SRXL2
- Cannot read Smart ESC/Battery telemetry

## Target Architecture

The FC has full control of the ESC bus as its master. Optionally, a second bus can receive RC input:

```
                                    FC
                                    │
            ┌───────────────────────┼───────────────────────┐
            │                       │                       │
    UART1 (SRXL2 bus 1)    Flight Controller        UART2 (SRXL2 bus 2)
    FC is MASTER (0x10)       core logic            FC is SLAVE (0x31)
            │                       │                       │
    ┌───────┴───────┐               │               Spektrum Receiver
    │               │               │               (bus master, 0x21)
 ESC (0x40)   Battery (0xB0)        │                       │
    │                               │               Transmitter [RF]
 Smart Battery                      │
 (reports via ESC)           Mixer / Failsafe
```

- **Bus 1 (master)**: FC owns this bus. Sends throttle, reads ESC/battery telemetry. No Spektrum receiver needed.
- **Bus 2 (slave, optional)**: FC receives RC input from a Spektrum receiver. Standard existing ArduPilot/iNav behavior.
- Each bus uses a separate UART. They are completely independent.
- The FC can also work standalone (bus 1 only) with its own control logic, no RC receiver at all.

---

## What Has Been Done

### Telemetry parsing library (`libsrxl2parser/srxl2_telemetry.{c,h}`)

Standalone decoder for the 16-byte X-Bus telemetry payloads inside SRXL2 telemetry packets. Added to the existing `libsrxl2parser` library.

**API**: `srxl2_decode_telemetry(raw[16], &decoded)` returns a tagged union (`srxl2_telem_decoded_t`) with `.type` discriminator.

**Supported sensor types**:
- ESC (0x20): RPM, voltage, current, FET/BEC temp, BEC voltage/current, throttle %, power %
- Flight Pack Capacity (0x34): current A/B, charge used A/B, temperature A/B
- LiPo Cell Monitor 6S (0x3A): 6 cell voltages, temperature
- RPM/Volts/Temp (0x7E): RPM from pulse period, voltage, temperature (F->C), RSSI
- Smart Battery (0x42) with sub-type dispatch:
  - Realtime (0x00): temp, current, consumption, min/max cell
  - Cells 1-6 (0x10), 7-12 (0x20), 13-18 (0x30): individual cell voltages
  - Battery ID (0x80): chemistry, cell count, mfg code, cycles
  - Limits (0x90): capacity, discharge rate, over-discharge, zero capacity, fully charged, temp range

**Implementation details**:
- All 16-bit fields are `swap16()`'d for big-endian → host conversion
- Sentinel values (`0xFFFF`, `0xFF`, `0x7FFF`) are detected and converted to `NAN`
- Wire-format packed structs + decoded float structs are separate (no aliasing issues)
- Smart Battery structs reverse-engineered from MSRC (`dgatf/msrc` `smart_esc.c`)
- Builds clean with no warnings

**Not yet integrated** into `srxl2_master_sim.c` — the simulator still just counts telemetry packets. Integration is Phase 2 step 4 in the roadmap.

### Analysis documents

- `doc/master-analysis.md` — this file
- `doc/msrc-analysis.md` — how MSRC implements both SRXL2 slave (telemetry bridge for third-party ESCs) and SRXL2 master (`smart_esc.c` — controlling Spektrum Smart ESCs + reading Smart Battery telemetry)

### Existing code inventory

| File | Role | Status |
|------|------|--------|
| `SRXL2/Source/spm_srxl.c` | Official Spektrum library | Has debug printfs in channel send path (P0 bug) |
| `SRXL2/Source/spm_srxl.h` | Official Spektrum library header | OK, defines all types and state machine |
| `SRXL2_Master/spm_srxl_master.c` | Custom master state machine | Handshake, channel send, telemetry polling — needs timing fixes |
| `SRXL2_Master/spm_srxl_master.h` | Master header | OK |
| `programs/srxl2_master_sim.c` | Simulator using fakeuart | 10s frame timing, no telemetry parsing, dual channel data paths |
| `programs/srxl2_battery_sim.c` | Battery slave simulator | For testing |
| `programs/srxl2_sniffer.c` | Bus sniffer | Uses libsrxl2parser, works with real serial |
| `programs/spm_srxl_config.h` | Config bridging official lib to UART adapter | OK |
| `programs/spm_srxl_master_config.h` | 13 extern hook declarations | OK |
| `programs/uart_adapter.h` | UART abstraction (fakeuart or real serial) | OK |
| `libsrxl2parser/srxl2_parser.{c,h}` | Standalone packet parser | OK, parses all packet types |
| `libsrxl2parser/srxl2_telemetry.{c,h}` | Telemetry payload decoder | **NEW** — decodes ESC, battery, smart battery, lipo, rpm |
| `libtransport/transport.{c,h}` | Transport abstraction | OK |
| `fakeuart/fakeuart.{c,h}` | Fake UART for simulation | OK |

---

## What Needs To Be Done Next

### Phase 1: Fix the core master (make it actually work)

These are all in `SRXL2_Master/spm_srxl_master.c` and `programs/srxl2_master_sim.c`.

**1a. Fix frame timing** — The master must send channel data every ~10ms unconditionally. Two approaches:

- **Simple (MSRC-style)**: Bypass the Spektrum library state machine entirely. Build and send `0xCD` packets directly on a 10ms timer, like `smart_esc.c` does. This is proven to work with real Smart ESCs.
- **Library-based**: Fix `srxl2_master_sim.c` main loop to call `srxlSetOutgoingChannelMask()` every 11ms instead of every 10 seconds. The state machine in `spm_srxl_master.c:277` already sends when `channelOutMask` is set.

The MSRC-style approach is simpler and proven. The library-based approach preserves the full protocol stack (bind, forward programming, multi-bus hub) but is more complex.

**1b. Remove debug printfs** — Delete lines 535-540 in `spm_srxl.c` (the `printf("Channel %d: ...")` in the channel data send path). These are in the official library code we modified.

**1c. Fix handshake scan** — In `spm_srxl_master.c:continueHandshake()`, replace the `0x10-0xFF` sequential scan with a table of default device IDs. The table from `spm_srxl.h:62-80` (`SRXL_DEFAULT_ID_OF_TYPE[]`) already exists.

**1d. Add baud rate switch on master side** — After sending the broadcast handshake (dest=0xFF), the master must call `srxlChangeBaudRate()` if the negotiated rate is 400000. This goes in `spm_srxl_master.c` after the broadcast send, or in `srxl2_master_sim.c` in the `srxlOnMasterHandshakeComplete` hook.

**1e. Single channel data path** — Remove the 10-second timer and channel writing from `srxl2_master_sim.c:main()`. All channel data should flow through the `srxlOnMasterFrame()` hook.

### Phase 2: Integrate telemetry parsing

**2a. Wire up the decoder** — In `srxl2_master_sim.c`, when a telemetry packet is received, call `srxl2_decode_telemetry()` on the 16-byte payload and print the decoded values.

**2b. Add telemetry to the sniffer** — The sniffer (`srxl2_sniffer.c`) already parses packets. Add telemetry payload decoding so it can display human-readable ESC/battery data in real time.

**2c. Store latest telemetry per device** — The master should maintain a `srxl2_telem_decoded_t` per discovered device (indexed by device ID). This is needed for the FC to read ESC status and battery state.

### Phase 3: ESC control

**3a. Channel-to-motor mapping** — Define which SRXL2 channel index maps to which ESC. For a single ESC: CH1 = throttle, CH7 = reverse (matching MSRC's smart_esc.c). For multi-motor: CH1-CH4 = motors 1-4.

**3b. Failsafe** — When the FC's own control source is lost (RC link timeout, autonomous mode abort, etc.):
- Set `srxlChDataIsFailsafe = true`
- Set throttle channel(s) to 0
- The state machine already handles sending `SRXL_CMD_CHANNEL_FS` when this flag is set

**3c. Test with real hardware** — Connect to a real Spektrum Smart ESC (e.g. Avian) via a USB-to-serial adapter with half-duplex support. Verify handshake, channel data delivery, and telemetry reception.

### Phase 4: Integration with ArduPilot/iNav

**4a. HAL abstraction** — Replace `fakeuart` / raw serial with ArduPilot `AP_HAL::UARTDriver` or iNav's serial API. The key interface points are:
- `srxlSendOnUart()` → HAL UART write
- `srxlChangeBaudRate()` → HAL baud rate change
- Receive path → HAL UART read with DMA

**4b. Dual-bus architecture** — Register two SRXL2 bus instances:
- Bus 0: Slave (existing ArduPilot/iNav SRXL2 driver, receives RC)
- Bus 1: Master (new code, controls ESCs)

**4c. Expose telemetry** — Map decoded ESC telemetry to ArduPilot's `AP_ESC_Telem` or iNav's ESC telemetry interface. Map Smart Battery data to the battery monitor backend.

**4d. Consider the simple approach** — Given that MSRC's `smart_esc.c` controls a Smart ESC in ~300 lines without the Spektrum library, a standalone master implementation (just building packets directly) may be easier to integrate than wrapping the full `spm_srxl.c` state machine. The Spektrum library is designed for firmware running inside a receiver — it has assumptions about being the only thing running, owning global state, etc. A lightweight direct implementation would be more FC-friendly.

### Open Questions

1. **Do Smart ESCs require the broadcast handshake (dest=0xFF)?** MSRC skips it and works. But if we want 400kbaud, we need it for baud negotiation.

2. **Can we use device ID 0x10 (remote receiver) or do ESCs only accept 0x21+ (receiver)?** MSRC uses 0x21. Need to test with 0x10.

3. **Multi-ESC on one bus**: Do all ESCs read from the same channel data packet (each reading their own channel by convention)? Or does each ESC need separate channel data addressed to it? The spec says channel data is broadcast — all devices see it. Each ESC reads the channel assigned during configuration.

4. **Smart Battery polling**: Does the ESC forward battery telemetry automatically when polled, or does the master need to poll a separate battery device ID? MSRC's code shows battery data comes from the ESC (same device ID 0x40), confirmed by the telemetry response containing sensor ID 0x42. The ESC alternates between its own telemetry (0x20) and forwarded battery data (0x42).

5. **Timing constraints for real-time FC integration**: At 11ms frame rate, the master has ~2-3ms to process a telemetry response before sending the next frame. On a modern FC MCU (STM32F4/F7/H7) this is plenty. On slower platforms, DMA-based UART is essential.
