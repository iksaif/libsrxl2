/*
 * Transport Abstraction Layer
 *
 * Provides a unified interface for both simulated (fakeuart) and real
 * hardware (USB-to-serial) communication.
 *
 * MIT License
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Transport types
typedef enum {
    TRANSPORT_TYPE_FAKEUART,    // Simulated UART over shared memory
    TRANSPORT_TYPE_SERIAL,      // Real serial port (USB-to-serial)
} transport_type_t;

// Baud rates
typedef enum {
    TRANSPORT_BAUD_115200 = 115200,
    TRANSPORT_BAUD_400000 = 400000,
} transport_baud_t;

// Transport handle (opaque)
typedef struct transport_handle transport_handle_t;

/**
 * @brief Initialize transport layer
 *
 * @param type Transport type (fakeuart or serial)
 * @param device Device name (bus name for fakeuart, device path for serial)
 * @param name Sender name (for fakeuart identification)
 * @param baud Baud rate (for serial port)
 * @return Transport handle on success, NULL on failure
 */
transport_handle_t* transport_init(
    transport_type_t type,
    const char* device,
    const char* name,
    transport_baud_t baud
);

/**
 * @brief Close transport
 *
 * @param handle Transport handle
 */
void transport_close(transport_handle_t* handle);

/**
 * @brief Send data
 *
 * @param handle Transport handle
 * @param data Data to send
 * @param length Data length
 * @return Number of bytes sent, or -1 on error
 */
int transport_send(transport_handle_t* handle, const uint8_t* data, size_t length);

/**
 * @brief Receive data
 *
 * @param handle Transport handle
 * @param buffer Buffer to receive data
 * @param buffer_size Buffer size
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
 * @param sender_name Buffer to receive sender name (optional, can be NULL)
 * @param sender_name_size Sender name buffer size
 * @return Number of bytes received, 0 on timeout, -1 on error
 */
int transport_receive(
    transport_handle_t* handle,
    uint8_t* buffer,
    size_t buffer_size,
    int timeout_ms,
    char* sender_name,
    size_t sender_name_size
);

/**
 * @brief Set promiscuous mode (fakeuart only)
 *
 * In promiscuous mode, receive all packets including those sent by self.
 *
 * @param handle Transport handle
 * @param enabled Enable promiscuous mode
 * @return true on success, false if not supported
 */
bool transport_set_promiscuous(transport_handle_t* handle, bool enabled);

/**
 * @brief Get transport type
 *
 * @param handle Transport handle
 * @return Transport type
 */
transport_type_t transport_get_type(transport_handle_t* handle);

/**
 * @brief List available serial ports
 *
 * Prints available serial ports to stdout.
 * On macOS: /dev/cu.* and /dev/tty.*
 * On Linux: /dev/ttyUSB*, /dev/ttyACM*
 */
void transport_list_serial_ports(void);

#ifdef __cplusplus
}
#endif

#endif // TRANSPORT_H
