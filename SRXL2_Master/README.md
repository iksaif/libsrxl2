# SRXL2_Master

Open-source implementation of SRXL2 bus master functionality for the Spektrum SRXL2 protocol.

## Overview

This library provides an implementation of `srxlRunMaster()` and related master-specific functions that work with the official Spektrum SRXL2 library. The official library includes placeholder hooks for master functionality but does not include the implementation (see SRXL2/Source/spm_srxl.h:575-579).

The SRXL2 bus master is responsible for:
- **Device Discovery**: Performing handshake with all devices on the bus
- **Channel Data Distribution**: Broadcasting RC channel data to slave devices
- **Telemetry Collection**: Priority-based polling of devices for telemetry data
- **Baud Rate Negotiation**: Managing bus speed negotiation
- **Bus Timing**: Maintaining 11ms frame timing for Spektrum protocol

## Architecture

### Core Files

- **spm_srxl_master.h** - Public API and master state machine definition
- **spm_srxl_master.c** - Implementation of `srxlRunMaster()` and related functions
- **spm_srxl_master_config.h** - Hook definitions for user customization

### Hook System

The library provides numerous hooks that allow users to customize master behavior without modifying the core implementation. All hooks are optional and have default no-op implementations.

#### Lifecycle Hooks

- `srxlOnMasterRunStart()` - Called at start of each master cycle
- `srxlOnMasterRunEnd()` - Called at end of each master cycle

#### Handshake Hooks

- `srxlOnMasterHandshakeStart()` - Device discovery starting
- `srxlOnMasterHandshakeComplete()` - Device discovery complete

#### Frame Timing Hooks

- `srxlOnMasterFrame()` - Called once per 11ms frame, can override default behavior

#### Channel Data Hooks

- `srxlOnMasterChannelDataSent()` - Channel data packet transmitted

#### Telemetry Hooks

- `srxlOnMasterSelectTelemDevice()` - Override default telemetry device selection
- `srxlOnMasterSetTelemTx()` - Control RF telemetry transmission
- `srxlOnMasterTelemSent()` - Telemetry transmitted over RF
- `srxlOnMasterSuppressTelem()` - Suppress internal telemetry generation

#### Bind Hooks

- `srxlOnMasterBind()` - Handle bind requests

## Examples

The library includes four comprehensive examples demonstrating different use cases:

### Example 1: Simple Logger (`example1_simple_logger.c`)

Demonstrates basic lifecycle hooks for logging all master events.

**Key Features:**
- High-resolution timing measurements
- Frame timing analysis
- Complete event logging

**Use Cases:**
- Development and debugging
- Understanding master state machine flow
- Performance profiling

**Hook Usage:**
```c
void srxlOnMasterRunStart(uint8_t busIndex)
{
    g_startTime = getMicros();
    printf("[Bus %d] Master cycle start\n", busIndex);
}

void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("[Bus %d] Found %d devices\n", busIndex, deviceCount);
}
```

### Example 2: Telemetry Monitor (`example2_telemetry_monitor.c`)

Tracks telemetry requests and provides detailed statistics.

**Key Features:**
- Per-device telemetry statistics
- Request/response rate tracking
- Periodic statistics reporting
- Telemetry balance analysis

**Use Cases:**
- Monitoring telemetry distribution across devices
- Validating priority-based scheduling
- Debugging telemetry issues

**Statistics Displayed:**
- Total frames and requests
- Per-device request counts
- Request rate percentages
- Last frame requested for each device

### Example 3: Custom Scheduler (`example3_custom_scheduler.c`)

Implements three different telemetry scheduling strategies.

**Scheduling Strategies:**

1. **Round-Robin**: Simple sequential polling
2. **Fixed Priority**: Device-type based priority
3. **Adaptive**: Increases priority for non-responsive devices

**Key Features:**
- Strategy selection at runtime
- Dynamic priority adjustment
- Device-type based prioritization

**Use Cases:**
- Custom telemetry requirements
- Prioritizing critical sensors
- Implementing application-specific scheduling

**Hook Usage:**
```c
uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    // Override default selection with custom algorithm
    switch (g_strategy)
    {
        case SCHED_ROUND_ROBIN:
            return scheduleRoundRobin(busIndex);
        case SCHED_FIXED_PRIORITY:
            return scheduleFixedPriority(busIndex);
        case SCHED_ADAPTIVE:
            return scheduleAdaptive(busIndex);
    }
}
```

### Example 4: Device Manager (`example4_device_manager.c`)

Comprehensive device discovery and management with rich status display.

**Key Features:**
- Device inventory tracking
- Device type identification
- Capability detection
- Configuration validation
- Periodic status reports

**Use Cases:**
- System validation and diagnostics
- Device inventory management
- Health monitoring
- User interface for device status

**Validates:**
- Required receiver presence
- Telemetry-capable device availability
- Expected device configuration

## Integration Guide

### 1. Include the Library

Add to your project:
```c
#include "SRXL2/Source/spm_srxl.h"
#define SRXL_INCLUDE_MASTER_CODE
#include "SRXL2_Master/spm_srxl_master.h"
```

### 2. Implement Hooks

Create your own `spm_srxl_master_config.h` or use the provided one and implement the hooks you need:

```c
#include "SRXL2_Master/spm_srxl_master_config.h"

// Implement only the hooks you need
void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount)
{
    printf("Found %d devices on bus %d\n", deviceCount, busIndex);
    // Your custom logic here
}

uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID)
{
    // Custom telemetry scheduling logic
    return myCustomScheduler(busIndex, defaultDeviceID);
}
```

### 3. Initialize SRXL2

```c
// Initialize as bus master (device ID 0x10 = remote receiver)
srxlInitDevice(0x10, 20, SRXL_DEVINFO_TELEM_TX_ENABLED, uniqueID);
srxlInitBus(0, uartHandle, SRXL_BAUD_115200);
```

### 4. Main Loop

```c
while (running)
{
    // Receive UART bytes and parse
    if (bytesReceived)
    {
        if (srxlParsePacket(0, rxBuffer, packetLength))
        {
            // Packet received and parsed
            // srxlRunMaster() will be called automatically
        }
    }
    else
    {
        // Timeout - advance state machine
        srxlRun(0, timeoutMs);
    }
}
```

## Master State Machine

The master operates in several states:

### 1. Handshake State

- Polls default device IDs for each device type (0x10, 0x21, 0x30, 0x40, etc.)
- Collects responses and builds device list
- Negotiates baud rate (115200 or 400000)
- Sends broadcast handshake (0xFF) to finalize

### 2. Running State

- Sends channel data every 11ms
- Includes telemetry request to selected device
- Uses priority-based weighted round-robin scheduling
- Ages telemetry counters to ensure fair polling

### 3. Special States

- **SendEnterBind**: Initiate bind mode on receiver
- **SendSetBindInfo**: Set bind information
- **RequestBindInfo**: Query bind status
- **SendBoundDataReport**: Report bind completion

## Telemetry Scheduling Algorithm

The default telemetry scheduler uses a priority-based weighted round-robin algorithm:

1. Each device has a **priority** (1-100) and an **age** counter
2. Age increments when device is not polled
3. Age resets to 0 when device is polled
4. **Score** = Priority × (Age + 1)
5. Device with highest score is selected

This ensures:
- Higher priority devices are polled more frequently
- All devices eventually get polled
- No device starvation

You can override this by implementing `srxlOnMasterSelectTelemDevice()`.

## Device Types

The SRXL2 protocol defines several device types:

| Type | ID Range | Description |
|------|----------|-------------|
| 0x0 | 0x00 | None |
| 0x1 | 0x10-0x1F | Remote Receiver |
| 0x2 | 0x20-0x2F | Receiver |
| 0x3 | 0x30-0x3F | Flight Controller |
| 0x4 | 0x40-0x4F | ESC |
| 0x6 | 0x60-0x6F | SRXL Servo (Type 1) |
| 0x7 | 0x70-0x7F | SRXL Servo (Type 2) |
| 0x8 | 0x80-0x8F | VTX |
| 0x9 | 0x90-0x9F | External RF |
| 0xA | 0xA0-0xAF | Remote ID |
| 0xB | 0xB0-0xBF | Sensor |
| 0xF | 0xFF | Broadcast |

## Timing Considerations

The SRXL2 protocol operates on an 11ms frame interval:

- **Channel Data**: Sent every 11ms by master
- **Telemetry Response**: Slave has <1ms to respond
- **Idle Time**: 1-character (87µs @ 115200) between packets
- **Timeout**: 50ms timeout triggers re-handshake

## Building

### With CMake

```bash
cd SRXL2_Master
mkdir build && cd build
cmake ..
make
```

### Manual Compilation

```bash
gcc -o example1 \
    examples/example1_simple_logger.c \
    spm_srxl_master.c \
    ../SRXL2/Source/spm_srxl.c \
    -I. -I../SRXL2/Source \
    -DSRXL_INCLUDE_MASTER_CODE
```

## Testing

The examples are self-contained and can be tested with:

1. **UART Simulation**: Use the included `libuartsim` for testing
2. **Hardware Testing**: Connect to real SRXL2 devices
3. **Unit Tests**: Test individual hook implementations

## Troubleshooting

### Master Not Detected

- Ensure device ID is 0x10 (remote receiver)
- Check that `master` flag is set in bus structure
- Verify UART is properly configured

### No Devices Discovered

- Check UART connections and signal levels
- Verify baud rate (start with 115200)
- Ensure devices are powered
- Check for bus conflicts (only one master allowed)

### Telemetry Not Received

- Verify devices support telemetry
- Check device info bits (SRXL_DEVINFO_TELEM_TX_ENABLED)
- Monitor telemetry requests in logs
- Verify timing (devices have <1ms to respond)

## License

MIT License - See LICENSE file for details.

## References

- [Official SRXL2 Library](https://github.com/SpektrumFPV/SpektrumDocumentation)
- [Bi-Directional SRXL Specification](../SpektrumDocumentation/Telemetry/Bi-Directional%20SRXL.pdf)
- [Spektrum Telemetry Documentation](https://github.com/SpektrumFPV/SpektrumDocumentation)

## Contributing

Contributions are welcome! Please:

1. Test thoroughly with real hardware
2. Follow the existing code style
3. Document all hooks and functions
4. Include example usage
5. Update README.md

## Acknowledgments

Based on the official Spektrum SRXL2 library by Horizon Hobby, LLC.
Protocol specification by Horizon Hobby, LLC.
