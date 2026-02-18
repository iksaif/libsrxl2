/*
 * UART Capture Adapter for Tests
 *
 * Replaces programs/uart_adapter.h. Routes uartTransmit() to a capture buffer
 * so tests can inspect exactly what the SRXL2 library sent.
 *
 * Variables are declared extern here and defined in test_helpers.c so that
 * all compilation units (spm_srxl.c, spm_srxl_master.c, test_*.c) share
 * the same capture state.
 *
 * MIT License
 */

#ifndef __UART_ADAPTER_H__
#define __UART_ADAPTER_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Capture buffer for transmitted packets
#define CAPTURE_TX_BUF_SIZE   8192
#define CAPTURE_MAX_PACKETS   512

typedef struct {
    size_t offset;
    size_t len;
} capture_packet_entry_t;

extern uint8_t g_tx_buf[CAPTURE_TX_BUF_SIZE];
extern size_t g_tx_len;
extern capture_packet_entry_t g_tx_packets[CAPTURE_MAX_PACKETS];
extern size_t g_tx_count;
extern uint32_t g_baud_rate;

static inline void capture_reset(void)
{
    g_tx_len = 0;
    g_tx_count = 0;
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
}

static inline void uartSetBaud(uint8_t uart, uint32_t baudRate)
{
    (void)uart;
    g_baud_rate = baudRate;
}

static inline void uartTransmit(uint8_t uart, uint8_t *pBuffer, uint8_t length)
{
    (void)uart;
    if (g_tx_len + length > CAPTURE_TX_BUF_SIZE || g_tx_count >= CAPTURE_MAX_PACKETS)
        return;

    g_tx_packets[g_tx_count].offset = g_tx_len;
    g_tx_packets[g_tx_count].len = length;
    g_tx_count++;

    memcpy(&g_tx_buf[g_tx_len], pBuffer, length);
    g_tx_len += length;
}

// Get a captured packet by index
static inline int capture_get_packet(size_t idx, const uint8_t **ptr, size_t *len)
{
    if (idx >= g_tx_count)
        return -1;
    *ptr = &g_tx_buf[g_tx_packets[idx].offset];
    *len = g_tx_packets[idx].len;
    return 0;
}

#endif // __UART_ADAPTER_H__
