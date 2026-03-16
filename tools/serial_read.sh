#!/bin/bash
# Read serial port output for a few seconds and print it
# Usage: ./tools/serial_read.sh [port] [seconds]
PORT="${1:-/dev/cu.usbmodem11201}"
SECONDS_TO_READ="${2:-5}"

echo "Reading from $PORT for ${SECONDS_TO_READ}s..."
stty -f "$PORT" 115200 raw -echo -echoe -echok -echoctl -echoke
timeout "$SECONDS_TO_READ" cat "$PORT" 2>&1
echo ""
echo "--- done ---"
