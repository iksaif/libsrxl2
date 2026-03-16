/*
 * Board-level pin and baud configuration for Pico SRXL2 targets.
 *
 * All targets share the same SRXL2 data pin and initial baud rate.
 * Override at compile time with -DSRXL2_PIN=<n> or -DSRXL2_BAUD_INIT=<n>.
 *
 * Default: GPIO 1 (RX/D0 on Arduino Nano RP2040 Connect).
 * On a plain Raspberry Pi Pico, GPIO 0 or any other pin works equally well.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifndef SRXL2_PIN
#define SRXL2_PIN       1
#endif

#ifndef SRXL2_BAUD_INIT
#define SRXL2_BAUD_INIT 115200
#endif

#endif /* BOARD_CONFIG_H */
