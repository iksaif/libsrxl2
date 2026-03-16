/*
 * SRXL2 Sniffer for Arduino Nano 33 BLE (Rev2)
 *
 * nRF52840-based sniffer with dual serial:
 *   Serial  = Native USB CDC (decoded output to host)
 *   Serial1 = Hardware UART on pins 0 (TX) / 1 (RX) (SRXL2 bus input)
 *
 * Full packet decoding + telemetry (1MB flash, 256KB RAM -- no constraints).
 *
 * Wiring:
 *   Pin 1 (Serial1 RX) -> SRXL2 bus data line
 *   GND                 -> SRXL2 bus ground
 *   USB                 -> Host computer (decoded output)
 *
 * MIT License
 */

#include "srxl2.h"
#include "srxl2_packet.h"
#include "srxl2_telemetry.h"

#define SRXL2_BAUD_RATE 115200

static uint8_t frame_buf[SRXL2_MAX_PACKET_SIZE];
static uint8_t frame_pos;
static uint8_t frame_expected_len;
static uint32_t pkt_count;

static void print_telemetry(const srxl2_pkt_telemetry_t *telem)
{
    srxl2_telem_decoded_t decoded;
    if (!srxl2_decode_telemetry(telem->payload, &decoded))
        return;

    switch (decoded.type) {
    case SRXL2_TELEM_TYPE_ESC:
        Serial.print("  ESC: ");
        Serial.print(decoded.esc.rpm, 0);
        Serial.print("RPM ");
        Serial.print(decoded.esc.voltage, 2);
        Serial.print("V ");
        Serial.print(decoded.esc.current, 2);
        Serial.print("A FET=");
        Serial.print(decoded.esc.temp_fet, 1);
        Serial.print("C");
        break;
    case SRXL2_TELEM_TYPE_FP_MAH:
        Serial.print("  FP: ");
        Serial.print(decoded.fp_mah.current_a, 1);
        Serial.print("A ");
        Serial.print(decoded.fp_mah.charge_used_a, 0);
        Serial.print("mAh");
        break;
    case SRXL2_TELEM_TYPE_SMART_BAT_REALTIME:
        Serial.print("  BAT: ");
        Serial.print(decoded.smart_bat_realtime.current, 2);
        Serial.print("A ");
        Serial.print(decoded.smart_bat_realtime.consumption, 1);
        Serial.print("mAh ");
        Serial.print(decoded.smart_bat_realtime.min_cell, 3);
        Serial.print("-");
        Serial.print(decoded.smart_bat_realtime.max_cell, 3);
        Serial.print("V");
        break;
    case SRXL2_TELEM_TYPE_RPM:
        Serial.print("  RPM: ");
        Serial.print(decoded.rpm.rpm, 0);
        Serial.print(" ");
        Serial.print(decoded.rpm.voltage, 2);
        Serial.print("V");
        break;
    default:
        Serial.print("  ");
        Serial.print(srxl2_telem_type_name(decoded.type));
        break;
    }
}

static void process_frame(void)
{
    srxl2_decoded_pkt_t pkt;
    srxl2_parse_result_t res = srxl2_pkt_parse(frame_buf, frame_pos, &pkt);

    pkt_count++;

    if (res != SRXL2_PARSE_OK) {
        Serial.print("#");
        Serial.print(pkt_count);
        Serial.print(" ERR ");
        Serial.println(srxl2_parse_result_name(res));
        return;
    }

    /* Blink LED on valid packet */
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("#");
    Serial.print(pkt_count);
    Serial.print(" ");
    Serial.print(srxl2_packet_type_name(pkt.packet_type));

    switch (pkt.packet_type) {
    case SRXL2_PKT_HANDSHAKE:
        Serial.print(" src=0x");
        Serial.print(pkt.handshake.src_id, HEX);
        Serial.print(" dst=0x");
        Serial.print(pkt.handshake.dest_id, HEX);
        Serial.print(" uid=0x");
        Serial.print(pkt.handshake.uid, HEX);
        break;

    case SRXL2_PKT_CONTROL:
        Serial.print(" reply=0x");
        Serial.print(pkt.control.reply_id, HEX);
        Serial.print(" ch=");
        Serial.print(pkt.control.channel.num_channels);
        Serial.print(" rssi=");
        Serial.print(pkt.control.channel.rssi);
        break;

    case SRXL2_PKT_TELEMETRY:
        Serial.print(" dst=0x");
        Serial.print(pkt.telemetry.dest_id, HEX);
        Serial.print(" sensor=");
        Serial.print(srxl2_telem_sensor_name(pkt.telemetry.payload[0]));
        Serial.print("(0x");
        Serial.print(pkt.telemetry.payload[0], HEX);
        Serial.print(")");
        print_telemetry(&pkt.telemetry);
        break;

    case SRXL2_PKT_BIND:
        Serial.print(" req=0x");
        Serial.print(pkt.bind.request, HEX);
        Serial.print(" dev=0x");
        Serial.print(pkt.bind.device_id, HEX);
        break;

    default:
        break;
    }

    Serial.println();
    digitalWrite(LED_BUILTIN, LOW);
}

static void feed_byte(uint8_t byte)
{
    if (frame_pos == 0) {
        if (byte != SRXL2_MAGIC)
            return;
        frame_buf[0] = byte;
        frame_pos = 1;
        frame_expected_len = 0;
        return;
    }

    frame_buf[frame_pos++] = byte;

    if (frame_pos == 3) {
        frame_expected_len = frame_buf[2];
        if (frame_expected_len < 5 || frame_expected_len > SRXL2_MAX_PACKET_SIZE) {
            frame_pos = 0;
            return;
        }
    }

    if (frame_expected_len > 0 && frame_pos >= frame_expected_len) {
        process_frame();
        frame_pos = 0;
    }
}

void setup()
{
    Serial.begin(115200);       /* USB CDC -- decoded output */
    Serial1.begin(SRXL2_BAUD_RATE);  /* Hardware UART -- SRXL2 input */
    pinMode(LED_BUILTIN, OUTPUT);

    /* Wait up to 3s for USB serial (skip if no host connected) */
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000)
        ;

    Serial.println();
    Serial.println("=== SRXL2 Nano 33 BLE Sniffer ===");
    Serial.print("Serial1 RX (pin 1) @ ");
    Serial.print(SRXL2_BAUD_RATE);
    Serial.println(" baud");
    Serial.println("Listening...");
    Serial.println();

    frame_pos = 0;
    pkt_count = 0;
}

void loop()
{
    while (Serial1.available()) {
        feed_byte((uint8_t)Serial1.read());
    }
}
