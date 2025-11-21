# SRXL2 Parser Library

A standalone, dependency-free SRXL2 packet parser library written in modern C.

## Features

- **Zero Dependencies**: Only requires standard C library (C99/C11)
- **Pure Parsing**: Read-only parsing with no side effects or state modification
- **Complete Protocol Coverage**: Supports all SRXL2 packet types
- **CRC Validation**: Built-in CRC-16 (XMODEM/ZMODEM) validation
- **Modern C Design**: Clean API with proper error handling
- **Lightweight**: ~5KB compiled library

## Supported Packet Types

- **Handshake** (0x21): Device discovery and configuration
- **Control Data** (0xCD): RC channel data with telemetry requests
- **Telemetry** (0x80): Sensor telemetry data
- **Bind** (0x41): Binding operations
- **RSSI** (0x55): Signal strength data
- **Parameter** (0x50): Parameter configuration
- **Internal** (0x99): Spektrum internal packets (optional)

## Usage Example

```c
#include "srxl2_parser.h"

// Parse a packet
uint8_t packet_data[SRXL2_MAX_PACKET_SIZE];
srxl2_packet_t packet;

srxl2_parse_result_t result = srxl2_parse_packet(
    packet_data,
    packet_length,
    &packet
);

if (result == SRXL2_PARSE_OK) {
    // Successfully parsed
    switch (packet.header.packet_type) {
        case SRXL2_PKT_HANDSHAKE:
            printf("Source: 0x%02X\n", packet.payload.handshake.src_device_id);
            printf("UID: 0x%08X\n", packet.payload.handshake.uid);
            break;

        case SRXL2_PKT_CONTROL:
            printf("RSSI: %d\n", packet.payload.control.channel_data.rssi);
            printf("Channels: %u\n", packet.payload.control.channel_data.num_channels);
            break;

        // ... handle other packet types
    }
} else {
    fprintf(stderr, "Parse error: %s\n", srxl2_parse_result_str(result));
}
```

## API Functions

### Parsing

```c
// Parse a complete SRXL2 packet
srxl2_parse_result_t srxl2_parse_packet(
    const uint8_t *data,
    size_t length,
    srxl2_packet_t *packet
);
```

### CRC Validation

```c
// Compute CRC-16 checksum
uint16_t srxl2_compute_crc(const uint8_t *data, size_t length);

// Validate packet CRC
bool srxl2_validate_crc(const uint8_t *data, size_t length);
```

### Helper Functions

```c
// Get device type from device ID
srxl2_device_type_t srxl2_get_device_type(uint8_t device_id);

// Get unit ID from device ID (0 = master)
uint8_t srxl2_get_unit_id(uint8_t device_id);

// Check if device is bus master
bool srxl2_is_master(uint8_t device_id);

// Get human-readable strings
const char* srxl2_device_type_name(srxl2_device_type_t device_type);
const char* srxl2_packet_type_name(srxl2_packet_type_t packet_type);
const char* srxl2_parse_result_str(srxl2_parse_result_t result);
```

## Building

The library is built as part of the SRXL2 Experiments project using CMake:

```bash
cd build
cmake ..
make
```

This produces `libsrxl2parser.a` that can be linked into your project.

## Design Principles

1. **No Global State**: All functions are stateless and thread-safe
2. **No Side Effects**: Parser only reads data, never modifies packets
3. **No Dependencies**: Self-contained with only standard C library
4. **Clear Error Handling**: Explicit error codes for all failure modes
5. **Const Correctness**: Input data marked const to prevent modification

## Differences from Official SRXL2 Library

Unlike the official Spektrum SRXL2 library (`spm_srxl.c`), this parser library:

- Does **not** maintain protocol state machines
- Does **not** handle bus timing or transmission
- Does **not** generate packets (read-only)
- Has **no** hardware dependencies (UART, timers, etc.)
- Is **portable** to any platform with a C compiler

This makes it ideal for:
- Traffic analysis tools (like the included sniffer)
- Protocol debugging
- Data logging and replay
- Testing and validation
- Protocol documentation

## License

MIT License - See project root LICENSE file.
