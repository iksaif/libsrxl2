/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Fake UART Library using UDP Multicast

This library simulates a UART bus using UDP multicast, allowing multiple
processes to communicate as if they were connected on a real UART bus.
All traffic is visible to all participants (like a real bus).
*/

#ifndef __FAKEUART_H__
#define __FAKEUART_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the fake UART library
 *
 * @param busName Name of the virtual bus (e.g., "srxl2bus")
 * @param deviceName Name of this device (for logging)
 * @return true on success, false on error
 */
bool fakeuart_init(const char* busName, const char* deviceName);

/**
 * @brief Close the fake UART connection
 */
void fakeuart_close(void);

/**
 * @brief Send data on the UART bus
 *
 * All devices on the bus will receive this data.
 *
 * @param data Pointer to data to send
 * @param length Number of bytes to send
 * @return Number of bytes sent, or -1 on error
 */
int fakeuart_send(const uint8_t* data, uint8_t length);

/**
 * @brief Receive data from the UART bus
 *
 * Non-blocking receive with timeout.
 *
 * @param buffer Buffer to store received data
 * @param maxLength Maximum bytes to receive
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
 * @param sender_name Optional pointer to store sender name (can be NULL)
 * @param sender_name_len Length of sender_name buffer (if provided)
 * @return Number of bytes received, 0 on timeout, -1 on error
 */
int fakeuart_receive(uint8_t* buffer, uint8_t maxLength, uint32_t timeout_ms,
                     char* sender_name, uint8_t sender_name_len);

/**
 * @brief Check if data is available to read
 *
 * @return true if data is available, false otherwise
 */
bool fakeuart_available(void);

/**
 * @brief Get the device name
 *
 * @return Device name string
 */
const char* fakeuart_get_device_name(void);

/**
 * @brief Set promiscuous mode (sniffer mode)
 *
 * In promiscuous mode, the device receives all traffic including
 * packets sent by itself. Useful for sniffers.
 *
 * @param enabled true to enable promiscuous mode
 */
void fakeuart_set_promiscuous(bool enabled);

/**
 * @brief Get statistics
 */
typedef struct
{
    uint64_t tx_packets;
    uint64_t tx_bytes;
    uint64_t rx_packets;
    uint64_t rx_bytes;
    uint64_t rx_errors;
} fakeuart_stats_t;

void fakeuart_get_stats(fakeuart_stats_t* stats);

#ifdef __cplusplus
}
#endif

#endif // __FAKEUART_H__
