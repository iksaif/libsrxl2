# MSRC SRXL2 Implementation Analysis

Source: https://github.com/dgatf/msrc (`board/project/protocol/srxl2.c`)

## What is MSRC?

MSRC (Multi Sensor for RC) is an RP2040-based project that acts as an **SRXL2 slave/sensor device**. It is a **protocol translator** for **non-Spektrum ESCs**: it reads telemetry data from third-party ESCs (Hobbywing, Castle, Kontronik, APD, OpenYGE, etc.) over their **proprietary serial protocols** on a separate UART, then re-packages that data into Spektrum telemetry format and sends it to a Spektrum receiver (the bus master) over SRXL2.

### What MSRC is NOT

- **NOT a bus master** - it does not send channel/throttle data to ESCs
- **NOT a sniffer** - it does not passively read an existing SRXL2 bus
- **NOT a proxy** - it does not forward SRXL2 packets between devices
- **NOT for Spektrum Smart ESCs** - Smart ESCs speak SRXL2 natively and sit directly on the bus; they don't need a bridge

MSRC exists because third-party ESCs (Hobbywing, Castle, etc.) have their own proprietary telemetry wire protocols that Spektrum receivers can't understand. MSRC bridges that gap.

## Architecture

```
                        UART1 (proprietary ESC protocol - NOT SRXL2)
  Third-party ESC  ──────────────────────────────────────►  MSRC (RP2040)
  (Hobbywing, Castle,                                          │
   Kontronik, APD, etc.)                                       │ Converts data
   Speaks its OWN protocol                                     │ to Spektrum X-Bus
                                                               │ telemetry format
                        UART0 (SRXL2 bus, slave mode)          │
  Spektrum Receiver  ◄────────────────────────────────────  MSRC
  (bus master, 0x10/0x21)   Replies with 0x80 telemetry
                            when polled by the master
                                  │
                              Also reads (via ADC, I2C, PIO):
                              - GPS, Baro, Current, Voltage, NTC, Fuel, Gyro, Airspeed
```

The two UARTs are completely independent. UART1 speaks whatever protocol the ESC uses (Hobbywing V4 binary, Castle telemetry, etc.). UART0 speaks SRXL2.

MSRC uses FreeRTOS. Each sensor type runs as its own task, writing to shared float pointers. The SRXL2 protocol task reads those pointers and formats them into X-Bus telemetry packets.

### Contrast with Spektrum Smart ESC

A Spektrum Smart ESC needs no bridge - it connects directly to the SRXL2 bus:

```
  Spektrum Receiver ──[SRXL2 bus]──► Smart ESC (0x40)
  (bus master)                        │
    Sends channel data (throttle)     │ Responds with STRU_TELE_ESC
    via 0xCD packets                  │ telemetry (0x80 packets)
                                      │
                                  Also connects to:
                                  Smart Battery (reports via ESC telemetry)
```

This is what we want to replicate: being the bus master that sends throttle and reads telemetry back.

## SRXL2 Slave Protocol Implementation

### Configuration

```c
#define SRXL2_DEVICE_ID       0x31    // Receiver type, unit ID 1
#define SRXL2_DEVICE_PRIORITY 10      // Standard telemetry priority
#define SRXL2_DEVICE_BAUDRATE 1       // Supports 400000 baud
#define SRXL2_DEVICE_INFO     0       // No RF capability
#define SRXL2_DEVICE_UID      0x12345678
```

Device ID `0x31` puts it in the "Flight Controller" range (0x30-0x3F). Priority 10 is the standard value for one telemetry message type.

### Startup Sequence

1. Initialize UART0 at 115200 baud, half-duplex
2. Start a repeating 50ms alarm timer
3. On each 50ms timeout (if no bus activity): send an unprompted handshake with `dest_id=0` to announce presence to the bus master
4. Wait for bus master to discover and poll this device

### Packet Processing (`process()`)

The main loop waits for UART data and handles three packet types:

#### 1. Handshake received (dest matches our ID)

```c
if (data[0] == SRXL2_HEADER && data[1] == 0x21 && data[4] == SRXL2_DEVICE_ID) {
    dest_id = data[3];  // Remember the master's device ID
    // Reply with our own handshake
    srxl2_send_handshake(uart0, SRXL2_DEVICE_ID, dest_id, ...);
}
```

When the bus master polls this device during handshake, MSRC:
- Records the master's device ID (from `srcDevID` field, byte offset 3)
- Replies with its own handshake containing supported baud rate and priority

#### 2. Broadcast handshake (dest = 0xFF)

```c
if (data[0] == SRXL2_HEADER && data[1] == 0x21 && data[4] == 0xFF) {
    baudrate = data[6];  // BaudRate field
    if (baudrate) uart_set_baudrate(uart0, 400000);
    else          uart_set_baudrate(uart0, 115200);
}
```

This is the final handshake from the master, setting the negotiated baud rate. All devices must switch.

#### 3. Control Data with ReplyID matching our device

```c
if (data[0] == SRXL2_HEADER && data[1] == 0xCD && data[4] == SRXL2_DEVICE_ID) {
    send_packet();  // Send telemetry reply
}
```

When the bus master sends channel data with `replyID = SRXL2_DEVICE_ID`, MSRC responds with one telemetry packet.

### 50ms Timeout Handler

```c
static int64_t alarm_50ms(alarm_id_t id, void *user_data) {
    send_handshake = true;
    if (baudrate) uart_set_baudrate(uart0, 115200);
    baudrate = 0;
    return 50000;  // Repeat every 50ms
}
```

If 50ms passes with no bus activity, MSRC assumes the master has reset. It:
- Drops back to 115200 baud
- Starts sending unprompted handshakes again (per spec section 7.2.1)

### Auto-baud Detection

If too many bad frames arrive (CRC failures), MSRC toggles between 115200 and 400000 baud:

```c
if (bad_frames > MAX_BAD_FRAMES) {
    if (baudrate) { uart_set_baudrate(uart0, 115200); baudrate = 0; }
    else          { uart_set_baudrate(uart0, 400000); baudrate = 1; }
    bad_frames = 0;
}
```

## Telemetry Packet Formatting

### Packet Structure

```c
typedef struct srxl2_telemetry_t {
    uint8_t header;        // 0xA6
    uint8_t type;          // 0x80
    uint8_t len;           // 22
    uint8_t dest_id;       // Master's device ID (for RF telemetry routing)
    uint8_t xbus_packet[16]; // Standard X-Bus 16-byte telemetry payload
    uint16_t crc;          // CRC-16 big-endian
} __attribute__((packed));
```

### Sensor Round-Robin

`send_packet()` iterates through enabled sensors in order, sending one per request:

```
XBUS_AIRSPEED -> XBUS_BATTERY -> XBUS_ESC -> XBUS_GPS_LOC -> XBUS_GPS_STAT
-> XBUS_RPMVOLTTEMP -> XBUS_FUEL_FLOW -> XBUS_STRU_TELE_DIGITAL_AIR -> (repeat)
```

Each call sends the next enabled sensor in the list. This is how priority works in practice: at priority 10, the master polls roughly once per 10 frames.

### ESC Telemetry (Sensor ID 0x20)

The X-Bus ESC packet (`STRU_TELE_ESC` format) is formatted as:

| Field | Raw sensor unit | Wire format | Wire unit |
|-------|----------------|-------------|-----------|
| RPM | float (RPM) | `swap_16(rpm / 10)` | 10 RPM/count, big-endian |
| Input Voltage | float (V) | `swap_16(v * 100)` | 0.01V/count |
| FET Temperature | float (C) | `swap_16(t * 10)` | 0.1C/count |
| Motor Current | float (A) | `swap_16(a * 100)` | 10mA/count |
| BEC Temperature | float (C) | `swap_16(t * 10)` | 0.1C/count |
| BEC Current | float (A) | `(uint8_t)(a * 10)` | 100mA/count |
| BEC Voltage | float (V) | `(uint8_t)(v * 20)` | 0.05V/count |
| Throttle | float (%) | `(uint8_t)(t * 2)` | 0.5%/count |
| Power Out | float (%) | `(uint8_t)(p * 2)` | 0.5%/count |

**Important**: All 16-bit fields are **big-endian** on the wire (note `swap_16()`), which matches the Spektrum telemetry spec comment "Uses big-endian byte order" for ESC data.

### Battery Telemetry (Sensor ID 0x34)

Uses `STRU_TELE_FP_MAH` format:

| Field | Wire format | Wire unit |
|-------|-------------|-----------|
| Current A | `swap_16(a * 10)` | 0.1A/count |
| Charge Used A | `swap_16(mah)` | 1mAh/count |
| Temperature A | `swap_16(t * 10)` | 0.1C/count |
| Current B | same as A | |
| Charge Used B | same as A | |
| Temperature B | same as A | |

### CRC Computation

MSRC uses the standard SRXL CRC-16 (XMODEM, polynomial 0x1021, seed 0):

```c
uint16_t srxl_crc16(uint16_t crc, uint8_t data) {
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else              crc = crc << 1;
    }
    return crc;
}
```

CRC is written big-endian into the packet: `packet.crc = swap_16(crc)`.

## Supported Third-Party ESC Protocols (read via UART1, slave mode only)

In slave/bridge mode, MSRC reads telemetry from these ESC protocols and converts to Spektrum format:

| ESC Protocol | Data Available |
|-------------|---------------|
| PWM | RPM only |
| Hobbywing V3 (HW3) | RPM |
| Hobbywing V4 (HW4) | RPM, Voltage, Current, FET Temp, BEC Temp |
| Hobbywing V5 (HW5) | RPM, Voltage, Current, FET Temp, BEC Temp |
| Castle Creations | RPM, Voltage, Current, FET Temp |
| Kontronik | RPM, Voltage, Current, FET Temp, BEC Temp |
| APD F-series | RPM, Voltage, Current, FET Temp |
| APD HV | RPM, Voltage, Current, FET Temp |
| OMP M4 | RPM, Voltage, Current, FET Temp |
| ZTW | RPM, Voltage, Current, FET Temp |
| OpenYGE | RPM, Voltage, Current, FET/BEC Temp, BEC V/I, Throttle |

---

## Smart ESC Mode: MSRC as Bus Master (`smart_esc.c`)

**`board/project/sensor/smart_esc.c`** is a completely separate mode from the slave telemetry bridge above. In this mode, MSRC **is a bus master** — it directly controls a Spektrum Smart ESC over SRXL2. This is exactly the architecture we want to replicate.

### How It Works

```
  PWM throttle input ──► MSRC (RP2040) ──[UART1, SRXL2 master]──► Smart ESC (0x40)
  PWM reverse input  ──►     │                                         │
                              │ reads telemetry                        │
                              ◄────────────── STRU_TELE_ESC ──────────┘
                              ◄────────────── STRU_TELE_SMARTBATT ────┘
                                               (battery data forwarded by ESC)
```

MSRC identifies as a receiver (`0x21`) on UART1:
```c
#define SRXL2_RECEIVER_ID       0x21
#define SRXL2_ESC_ID            0x40
#define SRXL2_RECEIVER_PRIORITY 0xA
#define SRXL2_RECEIVER_BAUDRATE 1       // 400000 baud supported
#define SRXL2_RECEIVER_INFO     0x7     // Telem TX + Full Range + FwdProg
#define SRXL2_INTERVAL_MS       10      // Send channel data every 10ms
```

### Handshake (master side)

1. On startup (or when `esc_id == 0`), MSRC sends a handshake to `0x40` (ESC) with source=`0x21`
2. When the ESC sends an unprompted handshake (dest=0), MSRC replies with its own handshake
3. Once the ESC responds, MSRC records `esc_id = 0x40` and `esc_priority`
4. If the ESC sends telemetry with `dest_id = 0xFF`, MSRC re-sends the handshake (re-handshake request)

**Note**: MSRC does NOT do the full handshake scan or broadcast (dest=0xFF). It only talks to `0x40`. This is a simplified but working implementation.

### Channel Data (master sends every 10ms)

```c
srxl2_control_packet_t packet;
packet.header = SRXL2_HEADER;           // 0xA6
packet.type = SRXL2_PACKET_TYPE_CONTROL; // 0xCD
packet.len = SRXL2_CONTROL_LEN_CHANNEL;
packet.command = SRXL2_CONTROL_CMD_CHANNEL; // 0x00
packet.reply_id = (cont % 10 == 0) ? esc_id : 0; // Poll ESC every 10th frame
channel_data.rssi = 0x64;               // 100% signal
channel_data.frame_losses = 0;
channel_data.channel_mask = 0b1000001;  // CH1 (throttle) + CH7 (reverse)
channel_data.channel_data_ch1 = throttle; // 0-65532 (16-bit full range)
channel_data.channel_data_ch7 = reverse;
```

Key observations:
- **reply_id** cycles: every 10th frame requests telemetry from the ESC, other frames send `reply_id = 0` (no reply wanted)
- **channel_mask** only includes CH1 and CH7 — the minimum needed for throttle + reverse
- **10ms interval** (alarm timer) — close to the 11ms spec, fast enough to keep the ESC happy
- **throttle range**: 0 to 65532, mapped from PWM pulse width (1000-2000us)

### Telemetry Parsing (master receives)

MSRC parses two types of telemetry from the ESC:

**ESC telemetry (sensor ID 0x20):**
```c
*parameter->rpm = swap_16(esc.rpm) * 10 * parameter->rpm_multiplier;
*parameter->voltage = swap_16(esc.volts_input) / 100.0;
*parameter->current = swap_16(esc.current_motor) / 100.0;
*parameter->voltage_bec = esc.voltage_bec == 0xFF ? 0 : esc.voltage_bec / 2.0;
*parameter->current_bec = esc.current_bec == 0xFF ? 0 : esc.current_bec / 100.0;
*parameter->temperature_fet = esc.temp_fet == 0xFFFF ? 0 : swap_16(esc.temp_fet) / 10.0;
*parameter->temperature_bec = esc.temp_bec == 0xFFFF ? 0 : swap_16(esc.temp_bec) / 10.0;
```

Note the `swap_16()` on all 16-bit fields — confirms big-endian byte order. Also `0xFFFF`/`0xFF` sentinel checks for "no data".

**Smart Battery telemetry (sensor ID 0x42)** — multiple sub-types via the `type` field (byte 2):

| Sub-type | ID | Data |
|----------|----|------|
| Realtime | 0x00 | Temperature (C), Current (mA), Consumption (0.1mAh), Min/Max cell voltage |
| Cells 1-6 | 0x10 | Temperature, Cell voltages 1-6 (mV) |
| Cells 7-12 | 0x20 | Temperature, Cell voltages 7-12 (mV) |
| Cells 13-18 | 0x30 | Temperature, Cell voltages 13-18 (mV) |
| Battery ID | 0x80 | Chemistry, Cell count, Mfg code, Cycles, UID |
| Limits | 0x90 | Capacity, Discharge rate, Over-discharge, Zero capacity, Full charge, Temp limits |

This is the **undocumented `STRU_TELE_SMARTBATT` (0x42)** format — not in the public `spektrumTelemetrySensors.h` but MSRC reverse-engineered it. The Smart Battery data comes **through the ESC** as telemetry responses — the battery is connected to the ESC, which forwards its data when polled.

### Smart Battery Struct Definitions (from MSRC)

```c
typedef struct srxl2_smart_bat_realtime_t {
    uint8_t identifier;   // 0x42
    uint8_t s_id;         // Secondary ID
    uint8_t type;         // 0x00
    int8_t temp;          // Temperature in C
    uint32_t current;     // Current in mA
    uint16_t consumption; // Consumption in 0.1 mAh
    uint16_t min_cel;     // Min cell voltage in mV
    uint16_t max_cel;     // Max cell voltage in mV
} __attribute__((packed));

typedef struct srxl2_smart_bat_cells_1_t {
    uint8_t identifier;   // 0x42
    uint8_t s_id;
    uint8_t type;         // 0x10
    int8_t temp;          // Temperature in C
    uint16_t cell_1;      // Cell voltage in mV
    uint16_t cell_2;
    uint16_t cell_3;
    uint16_t cell_4;
    uint16_t cell_5;
    uint16_t cell_6;
} __attribute__((packed));
// cells_2 (0x20): cells 7-12, cells_3 (0x30): cells 13-18 — same format

typedef struct srxl2_smart_bat_id_t {
    uint8_t identifier;   // 0x42
    uint8_t s_id;
    uint8_t type;         // 0x80
    uint8_t chemistery;
    uint8_t cells;        // Cell count
    uint8_t mfg_code;
    uint16_t cycles;      // Charge cycles
    uint8_t uid;
} __attribute__((packed));

typedef struct srxl2_smart_bat_limits_t {
    uint8_t identifier;   // 0x42
    uint8_t s_id;
    uint8_t type;         // 0x90
    uint8_t rfu;
    uint16_t capacity;        // mAh
    uint16_t discharge_rate;
    uint16_t overdischarge;
    uint16_t zero_capacity;
    uint16_t fully_charged;
    uint8_t min_temp;
    uint8_t max_temp;
} __attribute__((packed));
```

### PWM Input (throttle source)

MSRC reads throttle from a PWM input pin (standard servo pulse 1000-2000us):
```c
int delta = pulse - 1000;            // 0-1000us range
if (delta < 0) delta = 0;
if (delta > 1000) delta = 1000;
throttle = delta / 1000.0F * 65532;  // Scale to SRXL2 16-bit range
```

A 100ms timeout sets throttle to 0 if PWM signal is lost — simple failsafe.

---

## Key Takeaways for Our Master Implementation

1. **`smart_esc.c` is our primary reference** — it's a working SRXL2 bus master that controls Smart ESCs and reads Smart Battery telemetry. This is exactly our use case.

2. **The slave code (`srxl2.c`) is also useful** as a reference for how slave devices expect the master to behave (timing, handshake, baud negotiation).

3. **A minimal master works**: MSRC proves you don't need the full Spektrum library or a complex state machine. A simple handshake + periodic 10ms channel data packets is enough to control a Smart ESC.

4. **No broadcast handshake needed**: MSRC skips the dest=0xFF broadcast entirely and the ESC still works.

5. **Smart Battery data comes through the ESC**: The ESC forwards battery telemetry (0x42) as its own telemetry responses. No need to poll a separate battery device.

6. **Smart Battery format (0x42) is now documented**: MSRC reverse-engineered the sub-types (realtime, cells 1-18, battery ID, limits) that aren't in the public Spektrum headers.

7. **Our implementation may be over-engineered** — MSRC's smart_esc.c does the full master role in ~300 lines with no state machine. We should consider whether the full Spektrum library machinery is needed, or if a simpler direct implementation would be better for the ESC bus.

8. **Telemetry byte order is big-endian** for ESC and Smart Battery. Must `swap_16()` all 16-bit fields on LE hosts.

9. **Failsafe is simple**: throttle = 0 on signal loss timeout (100ms in MSRC).

10. **Poll rate**: MSRC requests telemetry every 10th frame (~100ms / ~10Hz). This is sufficient for ESC monitoring.
