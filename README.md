# SRXL2 Experiments

SRXL2 bus master and protocol stack for controlling Spektrum Smart ESCs and reading Smart Battery telemetry, targeting FC firmware (iNav/ArduPilot).

## Project Structure

```
.
├── libsrxl2/           Modern SRXL2 protocol stack (context-based, no globals)
├── SRXL2/              Official Spektrum library (submodule, read-only)
├── SRXL2_Master/       Legacy bus master hooks for the official library
├── fakeuart/           UDP multicast virtual UART (simulates half-duplex bus)
├── libtransport/       Abstraction over fakeuart and serial ports
├── programs/           Simulation programs
├── tests/              Test suite (11 tests)
├── doc/                Protocol specs and analysis docs
└── SpektrumDocumentation/  Spektrum telemetry definitions (submodule)
```

### libsrxl2 (new)

Context-based C11 library supporting both master and slave roles. No globals, no packed struct aliasing, explicit byte-order conversion. Includes:

- **srxl2.c/h** -- State machine (startup, handshake, running) with event callbacks
- **srxl2_packet.c/h** -- Packet codec (encode/decode all SRXL2 packet types)
- **srxl2_telemetry.c/h** -- Telemetry payload decoder (ESC, FP_MAH, LiPo monitor, Smart Battery, RPM)
- **srxl2_internal.h** -- Internal context struct and constants

### Legacy (SRXL2 + SRXL2_Master)

The official Spektrum library with a custom master implementation. Still works, but has known issues (see `doc/master-analysis.md`). Kept for reference and interop testing.

## Building

```bash
git clone --recursive <repository-url>
cd srxl2-experiments
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j
```

### Run Tests

```bash
ctest --output-on-failure
```

All 11 tests cover: CRC, packet codec, telemetry decoding, master/slave state machines, interop between old and new libraries, and scenario tests.

## Simulation Programs

Two sets of simulators exist -- legacy (using the official library) and new (using libsrxl2):

| Program | Library | Role | Device ID |
|---------|---------|------|-----------|
| `srxl2_new_master_sim` | libsrxl2 | Bus master | 0x10 |
| `srxl2_new_battery_sim` | libsrxl2 | Battery sensor | 0xB0 |
| `srxl2_master_sim` | legacy | Bus master | 0x10 |
| `srxl2_battery_sim` | legacy | Battery sensor | 0xB0 |
| `srxl2_sniffer` | libsrxl2 | Passive sniffer | -- |

### Quick Demo

Open 3 terminals:

```bash
# Terminal 1: sniffer
./build/srxl2_sniffer

# Terminal 2: master
./build/srxl2_new_master_sim

# Terminal 3: battery (can start after master -- late join works)
./build/srxl2_new_battery_sim
```

The master discovers the battery via handshake, sends channel data every 11ms, and polls for FP_MAH telemetry. The sniffer decodes all traffic in real time.

### Options

```
srxl2_new_master_sim  [--bus <name>] [--help]
srxl2_new_battery_sim [--bus <name>] [--id <0xB0-0xBF>] [--help]
srxl2_sniffer         [--bus <name>] [--format hex|state] [--help]
```

Different bus names create isolated virtual buses (UDP multicast groups).

## Protocol Overview

SRXL2 is a half-duplex UART protocol (115200 or 400000 baud):

1. **Startup** -- 50ms quiet period
2. **Handshake** -- Master scans default device IDs, slaves reply, master sends broadcast to enter Running
3. **Running** -- Master sends channel data every 11ms (5.5ms at 400k), one slave replies with telemetry per frame

Key packet types:
- `0x21` Handshake -- device discovery and baud negotiation
- `0xCD` Control Data -- RC channels + telemetry poll
- `0x80` Telemetry -- 16-byte X-Bus sensor payload
- `0x41` Bind -- bind mode management

## Documentation

- `doc/master-analysis.md` -- Analysis of known issues and roadmap
- `doc/msrc-analysis.md` -- How MSRC implements SRXL2 slave
- `doc/SRXL2 Specification.md` -- Protocol specification
- `doc/reimplementation-proposal.md` -- Design rationale for libsrxl2

## License

MIT License. See [LICENSE](LICENSE).

The official SRXL2 library (`SRXL2/`) is MIT licensed by Horizon Hobby, LLC.
