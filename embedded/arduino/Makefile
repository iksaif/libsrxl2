# SRXL2 Programs for Arduino Nano 33 BLE (Rev2)
#
# Build using arduino-cli. The nRF52840 has native USB (Serial) plus
# a hardware UART (Serial1), so SRXL2 bus and debug output are separate.
#
# Usage:
#   make                          Build sniffer, master, and fc
#   make sniffer                  Build sniffer only
#   make master                   Build master only
#   make fc                       Build flight controller only
#   make flash-sniffer PORT=/dev/ttyACM0  Upload sniffer
#   make flash-master  PORT=/dev/ttyACM0  Upload master
#   make flash-fc      PORT=/dev/ttyACM0  Upload FC
#   make clean                    Remove build artifacts and lib copies

# Board selection: nano33ble (nRF52840) or nanorp2040connect (RP2040)
# Override on command line: make fc BOARD=nanorp2040connect
BOARD    ?= nano33ble
FQBN      = arduino:mbed_nano:$(BOARD)
PORT     ?= /dev/ttyACM0

LIB_DIR   = ../../libsrxl2

# Sniffer: packet decoder only
SNIFFER_DIR   = sniffer_arduino
SNIFFER_LIBS  = srxl2.h srxl2_packet.h srxl2_packet.c \
                srxl2_telemetry.h srxl2_telemetry.c

# Master + FC: full state machine
FULL_LIBS     = srxl2.h srxl2_internal.h srxl2_packet.h srxl2_packet.c \
                srxl2_telemetry.h srxl2_telemetry.c srxl2.c

MASTER_DIR    = master_arduino
FC_DIR        = fc_arduino

COMPILE_FLAGS = --build-property "compiler.c.extra_flags=-DSRXL2_CRC_SMALL" \
                --build-property "compiler.cpp.extra_flags=-DSRXL2_CRC_SMALL"

.PHONY: all sniffer master fc flash-sniffer flash-master flash-fc clean

all: sniffer master fc

sniffer:
	@for f in $(SNIFFER_LIBS); do cp -f $(LIB_DIR)/$$f $(SNIFFER_DIR)/$$f; done
	arduino-cli compile --fqbn $(FQBN) $(COMPILE_FLAGS) $(SNIFFER_DIR)

master:
	@for f in $(FULL_LIBS); do cp -f $(LIB_DIR)/$$f $(MASTER_DIR)/$$f; done
	arduino-cli compile --fqbn $(FQBN) $(COMPILE_FLAGS) $(MASTER_DIR)

fc:
	@for f in $(FULL_LIBS); do cp -f $(LIB_DIR)/$$f $(FC_DIR)/$$f; done
	arduino-cli compile --fqbn $(FQBN) $(COMPILE_FLAGS) $(FC_DIR)

flash-sniffer: sniffer
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) $(SNIFFER_DIR)

flash-master: master
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) $(MASTER_DIR)

flash-fc: fc
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) $(FC_DIR)

clean:
	@cd $(SNIFFER_DIR) && rm -f $(SNIFFER_LIBS) 2>/dev/null; true
	@cd $(MASTER_DIR) && rm -f $(FULL_LIBS) 2>/dev/null; true
	@cd $(FC_DIR) && rm -f $(FULL_LIBS) 2>/dev/null; true
	arduino-cli cache clean 2>/dev/null || true
