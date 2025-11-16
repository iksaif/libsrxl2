# SRXL2 Simulation Programs

This directory contains simulation programs that use the fake UART library to test SRXL2 protocol implementations.

## Programs

### 1. srxl2_master_sim
Simulates an SRXL2 bus master (remote receiver).

**Features:**
- Performs device discovery via handshake
- Sends RC channel data every 11ms
- Polls devices for telemetry using priority-based scheduling
- Simulates realistic channel movements

**Usage:**
```bash
./srxl2_master_sim [bus_name]

# Example
./srxl2_master_sim
./srxl2_master_sim mybus
```

**Device ID:** 0x10 (Remote Receiver)

---

### 2. srxl2_battery_sim
Simulates an SRXL2 battery telemetry device.

**Features:**
- Responds to handshake requests
- Sends battery telemetry when polled
- Simulates realistic battery discharge
- Reports voltage, current, capacity, temperature

**Usage:**
```bash
./srxl2_battery_sim [bus_name]

# Example
./srxl2_battery_sim
./srxl2_battery_sim mybus
```

**Device ID:** 0xB0 (Sensor)
**Battery Specs:** 4S LiPo, 5000mAh, simulated discharge at ~5A

---

### 3. srxl2_sniffer
Captures and decodes all SRXL2 traffic on the bus.

**Features:**
- Promiscuous mode (sees all traffic)
- Real-time packet decoding
- Statistics tracking
- Optional hex dump
- Timestamp for each packet

**Usage:**
```bash
./srxl2_sniffer [bus_name] [--hex]

# Examples
./srxl2_sniffer               # Basic mode
./srxl2_sniffer --hex         # With hex dumps
./srxl2_sniffer mybus --hex   # Custom bus with hex
```

**Decodes:**
- Handshake packets (device discovery)
- Channel data packets (RC control)
- Telemetry packets (battery, GPS, etc.)
- Bind packets
- RSSI packets
- Parameter packets

---

## Quick Start

### Terminal 1: Start the sniffer
```bash
./srxl2_sniffer
```

### Terminal 2: Start the master
```bash
./srxl2_master_sim
```

### Terminal 3: Start the battery device
```bash
./srxl2_battery_sim
```

You should see:
1. **Sniffer** displays all packets with decoded information
2. **Master** performs handshake and starts sending channel data
3. **Battery** responds to handshake and sends telemetry when polled

---

## Expected Output

### Master Output
```
╔═══════════════════════════════════════════════╗
║       SRXL2 Master Simulator                  ║
╚═══════════════════════════════════════════════╝

[FakeUART:Master] Initializing on bus 'srxl2bus' (multicast 239.255.42.1:54678)
[FakeUART:Master] Ready
[Master] Initialized successfully
[Master] Starting device discovery on bus 0...
[Master] Discovery complete: 1 device(s) found on bus 0
[Master] Frame 100: Channel data sent (telem device=0xB0)
```

### Battery Output
```
╔═══════════════════════════════════════════════╗
║       SRXL2 Battery Simulator                 ║
╚═══════════════════════════════════════════════╝

[FakeUART:Battery] Initializing on bus 'srxl2bus' (multicast 239.255.42.1:54678)
[FakeUART:Battery] Ready
[Battery] Initialized successfully
[Battery] Telemetry sent #1: 16.80V, 5.00A, 0mAh used, 25.0°C
[Battery] Telemetry sent #50: 16.75V, 5.15A, 42mAh used, 25.2°C
```

### Sniffer Output
```
╔═══════════════════════════════════════════════╗
║       SRXL2 Bus Sniffer                       ║
╚═══════════════════════════════════════════════╝

[1731700000.123456] Packet #1 (13 bytes)
  Packet Type: 0x21, Length: 13, CRC: 0x1234
    Type: HANDSHAKE
    Source: 0x10 (Remote Receiver)
    Dest: 0x10 (Discovery)
    Priority: 20
    Baud: 115200
    Info: 0x01 [TelemTX]
    UID: 0x12345678
```

---

## Protocol Flow

1. **Handshake Phase (startup)**
   - Master sends handshake to default device IDs
   - Devices respond with their capabilities
   - Master sends broadcast handshake to finalize
   - Takes ~200ms typically

2. **Normal Operation**
   - Master sends channel data every 11ms
   - Includes telemetry request to one device
   - Requested device responds with telemetry
   - Priority-based round-robin scheduling

3. **Telemetry Flow**
   - Master includes device ID in channel data packet
   - Device recognizes its ID and prepares telemetry
   - Device sends telemetry packet in reply
   - Master ages telemetry counters for fair scheduling

---

## Fake UART Details

The fake UART library uses UDP multicast for communication:

**Configuration:**
- Multicast Group: `239.255.42.1`
- Port: `54200 + hash(bus_name) % 1000`
- Default Bus: `srxl2bus` → Port 54678

**Features:**
- All devices see all traffic (real bus behavior)
- Multiple devices can join the same bus
- Works across processes on the same machine
- Promiscuous mode for sniffing

**Bus Naming:**
Different bus names create separate virtual buses. This allows running multiple independent simulations simultaneously.

---

## Troubleshooting

### No packets seen
- Make sure all programs use the same bus name
- Check firewall settings for UDP multicast
- Try explicitly: `srxl2bus` (default)

### Master not discovering devices
- Start device before master, or
- Wait a few seconds for handshake to complete
- Check that device has valid ID (0x10-0xEF range)

### Sniffer shows invalid packets
- This is normal during startup
- Partial packets may be captured
- Wait for complete handshake sequence

### High CPU usage
- This is expected for simulation
- Master sends frames every 11ms (90 fps)
- Use Release build for better performance

---

## Extending

### Adding New Devices

1. Copy `srxl2_battery_sim.c` as template
2. Change device ID and type
3. Implement `userProvidedFillSrxlTelemetry()`
4. Add to CMakeLists.txt

Example device IDs:
- 0x30-0x3F: Flight Controller
- 0x40-0x4F: ESC
- 0xB0-0xBF: Sensor

### Custom Telemetry

See `SpektrumDocumentation/Telemetry/spektrumTelemetrySensors.h` for available telemetry types:
- GPS (0x16, 0x17)
- Altitude (0x18)
- Current (0x28)
- Voltage (0x01)
- RPM (0x20)
- And many more...

---

## Performance

**Master:**
- ~90 packets/second (11ms frame rate)
- Low CPU usage (~1-2%)
- Memory: <1MB

**Battery:**
- Responds on-demand
- Updates simulation every 100ms
- Minimal CPU usage

**Sniffer:**
- Captures all traffic
- Decoding adds minimal overhead
- Memory scales with packet rate

---

## Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Master    │     │   Battery    │     │   Sniffer   │
│   (0x10)    │     │    (0xB0)    │     │             │
└──────┬──────┘     └──────┬───────┘     └──────┬──────┘
       │                   │                     │
       │    UDP Multicast  │                     │
       └───────────────────┴─────────────────────┘
                 239.255.42.1:54678
```

All programs communicate via UDP multicast, simulating a shared UART bus.
