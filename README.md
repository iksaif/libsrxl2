<p align="center">
  <img src="logo.svg" alt="SRXL2" width="420">
</p>

<p align="center">
  <a href="https://github.com/iksaif/libsrxl2/actions/workflows/ci.yml"><img src="https://github.com/iksaif/libsrxl2/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <img src="https://img.shields.io/badge/language-C11-blue" alt="C11">
  <img src="https://img.shields.io/badge/license-MIT-green" alt="MIT License">
  <img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS%20%7C%20Pico%20%7C%20Arduino-orange" alt="Platforms">
  <img src="https://img.shields.io/badge/protocol-SRXL2-blueviolet" alt="SRXL2">
</p>

# SRXL2 Experiments

Open-source SRXL2 protocol library for third-party integrations (flight controllers, sensors, ESCs). Includes a bus master implementation, telemetry decoders/encoders, embedded targets (Pico, Arduino), and a real-time sniffer for decoding SRXL2 traffic on the bench.

**This is a community project** -- not affiliated with Spektrum or Horizon Hobby. Contributions, bug reports, and hardware testing are all welcome. If you have a Spektrum Smart ESC, receiver, or battery, your feedback is invaluable.

> **Hardware status:** Pico embedded targets (FC slave, sniffer) validated on
> an Arduino Nano RP2040 Connect with a Spektrum receiver. Channel data,
> RSSI, and telemetry work correctly over PIO half-duplex single-pin UART.
> Bus Pirate 5 passive sniffing also validated. Complex receivers
> (AR10360T) require Forward Programming configuration and need more
> investigation.

## Project Structure

```
.
├── libsrxl2/           Modern SRXL2 protocol stack (context-based, no globals)
├── SRXL2/              Official Spektrum library (submodule, read-only)
├── SRXL2_Master/       Legacy bus master hooks for the official library
├── fakeuart/           UDP multicast virtual UART (simulates half-duplex bus)
├── libtransport/       Abstraction over fakeuart and serial ports
├── programs/           Simulation programs
├── embedded/           Embedded targets (Pico, Arduino)
├── tests/              Test suite (11 tests)
└── SpektrumDocumentation/  Spektrum telemetry definitions (submodule)
```

### libsrxl2 (new)

Context-based C11 library supporting both master and slave roles. No globals, no packed struct aliasing, explicit byte-order conversion. Includes:

- **srxl2.c/h** -- State machine (startup, handshake, running) with event callbacks
- **srxl2_packet.c/h** -- Packet codec (encode/decode all SRXL2 packet types)
- **srxl2_telemetry.c/h** -- Telemetry payload decoder (ESC, FP_MAH, LiPo monitor, Smart Battery, RPM)
- **srxl2_internal.h** -- Internal context struct and constants

### Legacy (SRXL2 + SRXL2_Master)

The official Spektrum library with a custom master implementation. Still works, but has known issues. Kept for reference and interop testing.

## Building

```bash
git clone --recursive https://github.com/iksaif/libsrxl2.git
cd libsrxl2
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

All new programs support both simulated (fakeuart) and real (USB-to-serial) transports:

```
srxl2_new_master_sim  [-d <name>] [-s] [-B <rate>] [-h]
srxl2_new_battery_sim [-d <name>] [-s] [-B <rate>] [-i <0xB0-0xBF>] [-h]
srxl2_sniffer         [-d <name>] [-s] [-B <rate>] [-f details|json|oneline|state] [-x] [-n] [-h]
```

Long forms (`--device`, `--serial`, `--baud`, etc.) also work.

Simulation mode (default): different bus names create isolated virtual buses (UDP multicast groups).

Serial mode example:

```bash
./build/srxl2_sniffer --serial --device /dev/cu.usbserial-1420
./build/srxl2_new_master_sim --serial --device /dev/cu.usbserial-1420
```

## Hardware Testing with Bus Pirate 5

The sniffer and simulators can be connected to real SRXL2 hardware using a
Bus Pirate 5 as a USB-to-serial bridge.

### Wiring

```
Bus Pirate 5        Spektrum Receiver
────────────        ─────────────────
VOUT (5V)  ───────  VCC
GND        ───────  GND
RX         ───────  SRXL2 Data
```

The receiver needs 5V (check your receiver's specs). The BP5 VOUT can supply
this for bench testing without servos. Bidirectional communication (slave/master
sim) requires half-duplex TX+RX on a single wire -- this has not yet been
validated with the BP5.

### Bus Pirate 5 Configuration

Connect to the BP5 CLI (e.g., `tio /dev/cu.usbmodem5buspirate1`):

```
W 5          # Enable 5V power supply
m 3          # Select UART mode (NOT HDUART -- m 4 may produce garbled data)
             # Configure: 115200 baud, 8N1, no flow control, non-inverted
bridge       # Enter transparent bridge mode (use -s for echo suppression)
```

Exit tio (`Ctrl-T q`), then run the sniffer or simulator on the same port.

### Sniffing

```bash
# Detailed view with hex dump
./build/srxl2_sniffer --serial --device /dev/cu.usbmodem5buspirate1 -f details --hex

# Compact one-line format
./build/srxl2_sniffer --serial --device /dev/cu.usbmodem5buspirate1 -f oneline

# JSON output (pipe to file or jq)
./build/srxl2_sniffer --serial --device /dev/cu.usbmodem5buspirate1 -f json | tee capture.jsonl

# ncurses live state view
./build/srxl2_sniffer --serial --device /dev/cu.usbmodem5buspirate1 -f state
```

### Prerequisites

The BP5 firmware must include [PR #295](https://github.com/DangerousPrototypes/BusPirate5-firmware/pull/295)
(UART bridge fix) for the sniffer to receive data correctly. This fix is
included in BP5 firmware releases after that PR was merged.

### Known Issues

- **Use `m 3` (UART), not `m 4` (HDUART)** -- HDUART mode on the BP5 produces
  garbled data (byte-level corruption, no valid `0xA6` magic bytes).
- **USB latency** -- Bidirectional SRXL2 through a USB-serial bridge is not
  feasible: the USB round-trip (~3-5ms) exceeds the SRXL2 response window
  within an 11ms frame. Use an embedded target (Pico, Arduino) for slave/master
  roles. The BP5 bridge works well for passive sniffing.
- **macOS USB CDC quirk** -- The serial transport uses blocking reads with
  `VTIME` timeout. Non-blocking `select()` does not work reliably on macOS
  with BP5 USB CDC ports.
- **Baud rate 400000** -- macOS maps 400000 to 230400 (closest standard rate),
  which won't work. Stick with 115200 for testing.

## Embedded Programs

SRXL2 sniffer, bus master, flight controller, and battery sensor for Raspberry
Pi Pico and Arduino Nano 33 BLE. These compile `libsrxl2` directly for embedded
targets -- no OS, no dependencies beyond the vendor SDK. All Pico targets use
PIO half-duplex UART on a single GPIO pin (no echo guard needed).

The **FC example** (`fc_pico`, `fc_arduino`) registers as a Flight Controller
(device ID 0x30) slave, receives channel data from a Spektrum receiver, and
sends back FP_MAH + RPM telemetry. The **battery example** (`battery_pico`)
simulates a 4S LiPo sensor (device ID 0xB0) with voltage sag, current drain,
and temperature rise.

See [embedded/README.md](embedded/README.md) for build instructions and wiring.

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

## Contributing

This is a community project and contributions are welcome! Some ways to help:

- **Test on real hardware** -- If you have a Spektrum Smart ESC, receiver, or battery, try the sniffer or master and report what works (or doesn't)
- **Port to your FC** -- The libsrxl2 stack is designed to be embedded in iNav, ArduPilot, or any RTOS
- **Add telemetry types** -- The decoder covers ESC, FP_MAH, LiPo, RPM, and Smart Battery, but more X-Bus sensors exist
- **Bug reports and PRs** -- Open an issue or send a pull request

## License

MIT License. See [LICENSE](LICENSE).

The official SRXL2 library (`SRXL2/`) is MIT licensed by Horizon Hobby, LLC.
