#!/bin/bash
# SRXL2 Simulation Demo Script
#
# This script launches the SRXL2 simulation in separate terminal windows.
# Requires: iTerm2 (macOS) or gnome-terminal (Linux)

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║       SRXL2 Simulation Demo                   ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════╝${NC}"
echo

# Check if build directory exists
if [ ! -d "build" ]; then
    echo -e "${RED}Error: Build directory not found${NC}"
    echo "Please run: mkdir build && cd build && cmake .. && make"
    exit 1
fi

# Check if binaries exist
cd build
for bin in srxl2_sniffer srxl2_master_sim srxl2_battery_sim; do
    if [ ! -f "$bin" ]; then
        echo -e "${RED}Error: $bin not found${NC}"
        echo "Please build the project first: cd build && make"
        exit 1
    fi
done

echo -e "${GREEN}Starting SRXL2 simulation...${NC}"
echo

BUS_NAME="srxl2demo"

# Detect terminal emulator
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS - use iTerm2 if available, otherwise Terminal.app
    if command -v osascript &> /dev/null; then
        echo -e "${YELLOW}Using macOS Terminal${NC}"

        # Sniffer
        osascript <<EOF
tell application "Terminal"
    do script "cd '$PWD' && echo 'SRXL2 Sniffer' && ./srxl2_sniffer $BUS_NAME --hex"
    activate
end tell
EOF

        sleep 1

        # Master
        osascript <<EOF
tell application "Terminal"
    do script "cd '$PWD' && sleep 2 && echo 'SRXL2 Master' && ./srxl2_master_sim $BUS_NAME"
end tell
EOF

        sleep 1

        # Battery
        osascript <<EOF
tell application "Terminal"
    do script "cd '$PWD' && sleep 3 && echo 'SRXL2 Battery' && ./srxl2_battery_sim $BUS_NAME"
end tell
EOF

        echo -e "${GREEN}Launched in Terminal windows${NC}"
    fi
elif command -v gnome-terminal &> /dev/null; then
    # Linux - use gnome-terminal
    echo -e "${YELLOW}Using gnome-terminal${NC}"

    gnome-terminal -- bash -c "cd '$PWD' && echo 'SRXL2 Sniffer' && ./srxl2_sniffer $BUS_NAME --hex; exec bash"
    sleep 1
    gnome-terminal -- bash -c "cd '$PWD' && sleep 2 && echo 'SRXL2 Master' && ./srxl2_master_sim $BUS_NAME; exec bash"
    sleep 1
    gnome-terminal -- bash -c "cd '$PWD' && sleep 3 && echo 'SRXL2 Battery' && ./srxl2_battery_sim $BUS_NAME; exec bash"

    echo -e "${GREEN}Launched in gnome-terminal windows${NC}"
elif command -v xterm &> /dev/null; then
    # Fallback to xterm
    echo -e "${YELLOW}Using xterm${NC}"

    xterm -e "cd '$PWD' && echo 'SRXL2 Sniffer' && ./srxl2_sniffer $BUS_NAME --hex" &
    sleep 1
    xterm -e "cd '$PWD' && sleep 2 && echo 'SRXL2 Master' && ./srxl2_master_sim $BUS_NAME" &
    sleep 1
    xterm -e "cd '$PWD' && sleep 3 && echo 'SRXL2 Battery' && ./srxl2_battery_sim $BUS_NAME" &

    echo -e "${GREEN}Launched in xterm windows${NC}"
else
    echo -e "${RED}No supported terminal found${NC}"
    echo "Please run manually:"
    echo "  Terminal 1: ./srxl2_sniffer $BUS_NAME --hex"
    echo "  Terminal 2: ./srxl2_master_sim $BUS_NAME"
    echo "  Terminal 3: ./srxl2_battery_sim $BUS_NAME"
    exit 1
fi

echo
echo -e "${GREEN}Demo started!${NC}"
echo
echo "You should see:"
echo "  1. Sniffer capturing and decoding all packets"
echo "  2. Master performing handshake and sending channel data"
echo "  3. Battery responding with telemetry"
echo
echo "Press Ctrl+C in each window to stop."
