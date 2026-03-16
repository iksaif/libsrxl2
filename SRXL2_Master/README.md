# SRXL2 Master

Open-source bus master implementation for the Spektrum SRXL2 protocol.

## What is this?

The official [Spektrum SRXL2 library](https://github.com/SpektrumFPV/SpektrumDocumentation) (`spm_srxl.c`) provides the full SRXL2 state machine for **slave** devices but only has a placeholder for the **master** side:

```c
// spm_srxl.h:575-579
#ifdef SRXL_INCLUDE_MASTER_CODE
// NOTE: Most user applications should not be an SRXL2 bus master, so master-specific
// code is not open. If your application requires this functionality, please inquire
// about this from Spektrum RC.
#include "spm_srxl_master.h"
#endif
```

This library fills that gap. It implements `srxlRunMaster()` and the related functions that the official library expects, so you can build an SRXL2 bus master (e.g. for a flight controller, ground station, or test tool).

## What the bus master does

On an SRXL2 bus, the master is the device that orchestrates all communication:

1. **Device discovery** -- handshakes with every device on the bus
2. **Channel data** -- broadcasts RC channel values every 11ms (5.5ms at 400k baud)
3. **Telemetry polling** -- asks one slave per frame to send telemetry back
4. **Baud rate negotiation** -- switches from 115200 to 400000 if all devices support it
5. **Bind management** -- can initiate and complete receiver binding

## Files

| File | Description |
|------|-------------|
| `spm_srxl_master.c` | Implementation of `srxlRunMaster()` and related functions |
| `spm_srxl_master.h` | Public API |
| `examples/spm_srxl_master_config.h` | Example hook declarations (copy and adapt) |

## Integration

### 1. Add sources to your build

You need three source files:

- `SRXL2/Source/spm_srxl.c` (official library)
- `SRXL2_Master/spm_srxl_master.c` (this library)
- Your application code

Define `SRXL_INCLUDE_MASTER_CODE` so the official library pulls in the master header.

### 2. Provide `spm_srxl_config.h`

The official library requires a config header on the include path. It must define:

```c
#define SRXL_NUM_OF_BUSES           1
#define SRXL_DEVICE_ID              0x10    // Remote Receiver (typical master ID)
#define SRXL_DEVICE_PRIORITY        20
#define SRXL_DEVICE_INFO            SRXL_DEVINFO_TELEM_TX_ENABLED
#define SRXL_SUPPORTED_BAUD_RATES   SRXL_BAUD_400000  // or 0 for 115200 only
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_SPEED

// Required callbacks -- wire these to your UART driver:
static inline void srxlSendOnUart(uint8_t uart, uint8_t *pBuffer, uint8_t length) { ... }
static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate) { ... }

// Telemetry / channel data callbacks:
static inline void srxlFillTelemetry(SrxlTelemetryData *pTelemetryData) { ... }
static inline void srxlReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafe) { ... }

// Threading (no-op if single-threaded):
static inline void srxlEnterCriticalSection(void) { }
static inline void srxlExitCriticalSection(void) { }
```

### 3. Provide `spm_srxl_master_config.h`

This header declares the 13 master hooks. Copy `examples/spm_srxl_master_config.h` and implement the hooks you care about. All hooks must be defined (they are `extern`), but most can be stubs:

```c
// Minimal set -- just print when handshake completes:

void srxlOnMasterRunStart(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterRunEnd(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterHandshakeStart(uint8_t busIndex) { (void)busIndex; }
void srxlOnMasterHandshakeComplete(uint8_t busIndex, uint8_t deviceCount) {
    printf("Bus %d: found %d devices\n", busIndex, deviceCount);
}
bool srxlOnMasterFrame(uint8_t busIndex, uint16_t frameCount) {
    (void)busIndex; (void)frameCount; return false;
}
void srxlOnMasterChannelDataSent(uint8_t busIndex, uint8_t telemDeviceID) {
    (void)busIndex; (void)telemDeviceID;
}
uint8_t srxlOnMasterSelectTelemDevice(uint8_t busIndex, uint8_t defaultDeviceID) {
    (void)busIndex; return defaultDeviceID;
}
void srxlOnMasterSetTelemTx(bool enabled) { (void)enabled; }
void srxlOnMasterTelemSent(void) { }
void srxlOnMasterSuppressTelem(void *pTelemetryData) { (void)pTelemetryData; }
bool srxlOnMasterBind(void *pBindInfo) { (void)pBindInfo; return false; }
bool srxlOnMasterParseInternal(void *pInternal) { (void)pInternal; return false; }
uint8_t srxlOnMasterFillInternal(void *pInternal) { (void)pInternal; return 0; }
```

### 4. Initialize and run

```c
#include "spm_srxl.h"  // pulls in spm_srxl_master.h via SRXL_INCLUDE_MASTER_CODE

// Init device as remote receiver (typical master)
srxlInitDevice(0x10, 20, SRXL_DEVINFO_TELEM_TX_ENABLED, my_unique_id);
srxlInitBus(0, uart_index, SRXL_BAUD_400000);

// Main loop
while (running) {
    int n = uart_read(buf, sizeof(buf), timeout_ms);
    if (n > 0) {
        // Feed bytes, parse complete packets
        for (int i = 0; i < n; i++) {
            rx_buf[rx_pos++] = buf[i];
            if (rx_pos >= 3 && rx_pos >= rx_buf[2]) {
                srxlParsePacket(0, rx_buf, rx_pos);
                rx_pos = 0;
            }
        }
    } else {
        // No data -- advance state machine on timeout
        srxlRun(0, timeout_ms);
    }

    // Set channel data for the master to broadcast
    srxlChData.mask = 0x0000001F;  // channels 0-4
    srxlChData.values[0] = 32768;  // center
    // ...
}
```

## Hooks reference

| Hook | When called | Typical use |
|------|-------------|-------------|
| `srxlOnMasterRunStart` | Start of each `srxlRunMaster()` call | Timing, debug |
| `srxlOnMasterRunEnd` | End of each `srxlRunMaster()` call | Timing, debug |
| `srxlOnMasterHandshakeStart` | Handshake sequence begins | Logging |
| `srxlOnMasterHandshakeComplete` | All devices discovered | Enable channel output |
| `srxlOnMasterFrame` | Each frame in Running state | Return `true` to skip default channel send |
| `srxlOnMasterChannelDataSent` | After channel packet TX | Logging, stats |
| `srxlOnMasterSelectTelemDevice` | Choosing which slave to poll | Custom scheduling |
| `srxlOnMasterSetTelemTx` | RF telemetry enable/disable | Control RF output |
| `srxlOnMasterTelemSent` | Telemetry sent over RF | Aging, stats |
| `srxlOnMasterSuppressTelem` | External telemetry received | Dedup |
| `srxlOnMasterBind` | Bind requested | Bind logic |
| `srxlOnMasterParseInternal` | Internal test packet received | Testing |
| `srxlOnMasterFillInternal` | Internal test packet requested | Testing |

## Telemetry scheduling

The default scheduler uses priority-weighted aging:

- Each discovered device has a **priority** (from its handshake) and an **age** counter
- Every frame, all ages increment by 1
- The polled device's age resets to 0
- **Score = priority x (age + 1)** -- highest score wins

This ensures higher-priority devices are polled more often while preventing starvation. Override it by returning a different device ID from `srxlOnMasterSelectTelemDevice()`.

## Device types

| Upper nibble | ID range | Type |
|:---:|:---:|---|
| 0x1 | 0x10-0x1F | Remote Receiver |
| 0x2 | 0x20-0x2F | Receiver |
| 0x3 | 0x30-0x3F | Flight Controller |
| 0x4 | 0x40-0x4F | ESC |
| 0x6 | 0x60-0x6F | SRXL Servo (Type 1) |
| 0x7 | 0x70-0x7F | SRXL Servo (Type 2) |
| 0x8 | 0x80-0x8F | VTX |
| 0xB | 0xB0-0xBF | Sensor |
| 0xF | 0xFF | Broadcast |

## Building (as part of this repo)

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
# Run the master simulator with fakeuart:
./build/srxl2_master_sim
```

## Known limitations

- Handshake scans all IDs 0x10-0xFE instead of only the default IDs per device type
- No baud rate switch on the master side after negotiation
- No failsafe timeout escalation (e.g. re-handshake after prolonged loss)

## License

MIT -- see [LICENSE](../LICENSE).

The official Spektrum SRXL2 library (`SRXL2/Source/`) is separately licensed by Horizon Hobby, LLC under MIT.
