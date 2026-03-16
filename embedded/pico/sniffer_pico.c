/*
 * SRXL2 Sniffer for Raspberry Pi Pico
 *
 * Passively sniffs SRXL2 bus traffic and prints decoded packet summaries
 * over USB CDC serial.
 *
 * Two UART backends (compile-time):
 *   -DUSE_HW_UART   -> hardware UART RX-only (for debugging wiring)
 *   default          -> PIO single-pin half-duplex (TX SM loaded but unused)
 *
 * Wiring:
 *   SRXL2_PIN (default GPIO 1) -> SRXL2 bus data line (RX only)
 *   GND                        -> SRXL2 bus ground
 *   USB                        -> Host computer (for decoded output)
 *
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#ifdef USE_HW_UART
#include "hardware/uart.h"
#else
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pio_uart.pio.h"
#include "pio_uart_hal.h"
#endif

#include "srxl2.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"

#include "board_config.h"

/* LED for status indication */
#define LED_PIN         PICO_DEFAULT_LED_PIN

#ifdef USE_HW_UART
/* Pick UART instance based on pin (UART0: GPIO 1/13/17/29, UART1: GPIO 5/9/21/25) */
#if (SRXL2_PIN == 1 || SRXL2_PIN == 13 || SRXL2_PIN == 17 || SRXL2_PIN == 29)
#define SRXL2_UART uart0
#elif (SRXL2_PIN == 5 || SRXL2_PIN == 9 || SRXL2_PIN == 21 || SRXL2_PIN == 25)
#define SRXL2_UART uart1
#else
#error "SRXL2_PIN is not a valid UART RX pin"
#endif
#else
static pio_uart_t pio_uart;
#endif

/* Frame assembly state */
static uint8_t frame_buf[SRXL2_MAX_PACKET_SIZE];
static uint8_t frame_pos;
static uint8_t frame_expected_len;

static uint64_t pkt_count;
static uint64_t byte_count;

static void print_telemetry(const srxl2_pkt_telemetry_t *telem)
{
    srxl2_telem_decoded_t decoded;
    if (!srxl2_decode_telemetry(telem->payload, &decoded))
        return;

    switch (decoded.type) {
    case SRXL2_TELEM_TYPE_ESC:
        printf("  ESC: %.0fRPM %.2fV %.2fA FET=%.1fC",
               decoded.esc.rpm, decoded.esc.voltage,
               decoded.esc.current, decoded.esc.temp_fet);
        break;
    case SRXL2_TELEM_TYPE_FP_MAH:
        printf("  FP: %.1fA %.0fmAh",
               decoded.fp_mah.current_a, decoded.fp_mah.charge_used_a);
        break;
    case SRXL2_TELEM_TYPE_SMART_BAT_REALTIME:
        printf("  BAT: %.2fA %.1fmAh %.3fV-%.3fV",
               decoded.smart_bat_realtime.current,
               decoded.smart_bat_realtime.consumption,
               decoded.smart_bat_realtime.min_cell,
               decoded.smart_bat_realtime.max_cell);
        break;
    case SRXL2_TELEM_TYPE_RPM:
        printf("  RPM: %.0f %.2fV",
               decoded.rpm.rpm, decoded.rpm.voltage);
        break;
    default:
        printf("  %s", srxl2_telem_type_name(decoded.type));
        break;
    }
}

static void process_frame(void)
{
    srxl2_decoded_pkt_t pkt;
    srxl2_parse_result_t res = srxl2_pkt_parse(frame_buf, frame_pos, &pkt);

    pkt_count++;

    if (res != SRXL2_PARSE_OK) {
        printf("#%llu ERR %s\n", pkt_count, srxl2_parse_result_name(res));
        return;
    }

    /* LED off briefly while processing */
    gpio_put(LED_PIN, 0);

    printf("#%llu %s", pkt_count, srxl2_packet_type_name(pkt.packet_type));

    switch (pkt.packet_type) {
    case SRXL2_PKT_HANDSHAKE:
        printf(" src=0x%02X dst=0x%02X uid=0x%08X",
               pkt.handshake.src_id, pkt.handshake.dest_id,
               (unsigned)pkt.handshake.uid);
        break;

    case SRXL2_PKT_CONTROL:
        printf(" reply=0x%02X ch=%u rssi=%d",
               pkt.control.reply_id,
               pkt.control.channel.num_channels,
               pkt.control.channel.rssi);
        break;

    case SRXL2_PKT_TELEMETRY:
        printf(" dst=0x%02X sensor=%s(0x%02X)",
               pkt.telemetry.dest_id,
               srxl2_telem_sensor_name(pkt.telemetry.payload[0]),
               pkt.telemetry.payload[0]);
        print_telemetry(&pkt.telemetry);
        break;

    case SRXL2_PKT_BIND:
        printf(" req=0x%02X dev=0x%02X",
               pkt.bind.request, pkt.bind.device_id);
        break;

    default:
        break;
    }

    printf("\n");

    /* LED back on after processing */
    gpio_put(LED_PIN, 1);
}

static void feed_byte(uint8_t byte)
{
    if (frame_pos == 0) {
        /* Waiting for sync byte */
        if (byte != SRXL2_MAGIC)
            return;
        frame_buf[0] = byte;
        frame_pos = 1;
        frame_expected_len = 0;
        return;
    }

    frame_buf[frame_pos++] = byte;

    /* After byte 2 (index 2) we know the length */
    if (frame_pos == 3) {
        frame_expected_len = frame_buf[2];
        if (frame_expected_len < 5 || frame_expected_len > SRXL2_MAX_PACKET_SIZE) {
            /* Invalid length, reset */
            frame_pos = 0;
            return;
        }
    }

    /* Check if frame is complete */
    if (frame_expected_len > 0 && frame_pos >= frame_expected_len) {
        process_frame();
        frame_pos = 0;
    }
}

static inline bool uart_rx_ready(void)
{
#ifdef USE_HW_UART
    return uart_is_readable(SRXL2_UART);
#else
    return pio_uart_readable(&pio_uart);
#endif
}

static inline uint8_t uart_rx_byte(void)
{
#ifdef USE_HW_UART
    return (uint8_t)uart_getc(SRXL2_UART);
#else
    return pio_uart_getc(&pio_uart);
#endif
}

int main(void)
{
    stdio_init_all();

    /* LED init */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#ifdef USE_HW_UART
    uart_init(SRXL2_UART, SRXL2_BAUD_INIT);
    gpio_set_function(SRXL2_PIN, GPIO_FUNC_UART);
    printf("\n=== SRXL2 Pico Sniffer (HW UART) ===\n");
    printf("UART RX on GPIO %d @ %u baud\n", SRXL2_PIN, SRXL2_BAUD_INIT);
#else
    pio_uart_init(&pio_uart, pio0, SRXL2_PIN, SRXL2_BAUD_INIT);
    printf("\n=== SRXL2 Pico Sniffer (PIO UART) ===\n");
    printf("PIO UART RX on GPIO %d @ %u baud\n", SRXL2_PIN, SRXL2_BAUD_INIT);
#endif
    printf("Listening...\n\n");

    frame_pos = 0;
    pkt_count = 0;
    byte_count = 0;

    /* LED on = listening (alive) */
    gpio_put(LED_PIN, 1);

    uint64_t last_heartbeat_us = time_us_64();

    /* Raw hex dump buffer for debugging (first N bytes when no packets) */
    uint8_t raw_dump[32];
    uint8_t raw_dump_pos = 0;

    while (true) {
        while (uart_rx_ready()) {
            uint8_t byte = uart_rx_byte();
            byte_count++;
            feed_byte(byte);
            /* Capture raw bytes for debug dump */
            if (raw_dump_pos < sizeof(raw_dump))
                raw_dump[raw_dump_pos++] = byte;
        }

        /* Heartbeat every 2 seconds */
        uint64_t now = time_us_64();
        if (now - last_heartbeat_us >= 2000000) {
            printf("[sniffer] alive  bytes=%llu pkts=%llu",
                   byte_count, pkt_count);
            /* If we have bytes but no packets, dump raw hex */
            if (pkt_count == 0 && raw_dump_pos > 0) {
                printf("  raw:");
                for (uint8_t i = 0; i < raw_dump_pos; i++)
                    printf(" %02X", raw_dump[i]);
            }
            printf("\n");
            last_heartbeat_us = now;
        }

        /* Yield to USB CDC processing */
        tight_loop_contents();
    }
}
