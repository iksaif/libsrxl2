# SRXL2 Experiments

Complete SRXL2 protocol implementation with simulation environment for the Spektrum RC protocol.

## 🎯 Project Overview

This repository provides:

1. **SRXL2_Master**: Open-source bus master implementation with hooks
2. **Fake UART Library**: UDP-based virtual UART for multi-process simulation
3. **Simulation Programs**: Master, battery device, and bus sniffer
4. **Official SRXL2 Library**: Integration with Spektrum's official library

## 📁 Project Structure

- `SRXL2/`: Official Spektrum SRXL2 library (submodule)
- `SRXL2_Master/`: Open-source bus master implementation
- `fakeuart/`: UDP multicast-based virtual UART library
- `programs/`: Simulation programs (master, battery, sniffer)
- `SpektrumDocumentation/`: Protocol specifications and telemetry definitions

## 🚀 Quick Start

### Prerequisites

- CMake (version 3.10 or higher)
- C compiler (GCC, Clang, etc.)
- macOS or Linux (UDP multicast support)

### Build

```bash
# Clone and build
git clone <repository-url>
cd srxl2-experiments

# Initialize submodules
git submodule update --init --recursive

# Build
mkdir build && cd build
cmake ..
make -j4

# Binaries will be in build/
ls srxl2_*
```

### Run the Demo

Open 3 terminals:

**Terminal 1** - Start the sniffer:
```bash
./srxl2_sniffer
```

**Terminal 2** - Start the master:
```bash
./srxl2_master_sim
```

**Terminal 3** - Start the battery device:
```bash
./srxl2_battery_sim
```

You should see:
- Master discovering the battery device
- Channel data sent every 11ms
- Battery telemetry responses
- Sniffer decoding all traffic

## 📚 Documentation

### Main Components

1. **[SRXL2_Master](SRXL2_Master/README.md)** - Bus master implementation with 13 customizable hooks
   - Device discovery and handshake
   - Priority-based telemetry scheduling
   - Channel data distribution
   - Comprehensive examples

2. **[Simulation Programs](programs/README.md)** - Ready-to-run simulators
   - `srxl2_master_sim` - Bus master (remote receiver)
   - `srxl2_battery_sim` - Battery telemetry device
   - `srxl2_sniffer` - Protocol analyzer

3. **[Fake UART](fakeuart/)** - Virtual UART using UDP multicast
   - Multi-process communication
   - True bus behavior (all see all)
   - Promiscuous mode for sniffing

### Protocol Flow

```
Startup:
  Master → Handshake (discovery) → All Devices
  Devices → Handshake Reply → Master
  Master → Broadcast Handshake → All Devices

Normal Operation (every 11ms):
  Master → Channel Data + Telem Request → All Devices
  Selected Device → Telemetry → Master
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
