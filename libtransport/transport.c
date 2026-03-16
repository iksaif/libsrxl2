/*
 * Transport Abstraction Layer - Implementation
 *
 * MIT License
 */

#include "transport.h"
#include "fakeuart.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>

#ifdef __APPLE__
#include <sys/ioctl.h>
#include <termios.h>
#elif defined(__linux__)
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#endif

// Transport handle structure
struct transport_handle {
    transport_type_t type;
    union {
        struct {
            // fakeuart state (managed by fakeuart library)
            char bus_name[64];
            char sender_name[64];
        } fakeuart;
        struct {
            int fd;
            char device_path[256];
        } serial;
    } data;
};

//=============================================================================
// Serial Port Functions
//=============================================================================

#if defined(__APPLE__) || defined(__linux__)

static speed_t baud_to_speed(transport_baud_t baud)
{
    switch (baud) {
        case TRANSPORT_BAUD_115200: return B115200;
        case TRANSPORT_BAUD_400000:
#ifdef __linux__
            return B460800;  // Use 460800 on Linux (closest to 400000)
#else
            return B230400;  // Use 230400 on macOS (closest available)
#endif
        default: return B115200;
    }
}

static bool serial_open(transport_handle_t* handle, const char* device, transport_baud_t baud)
{
    // Open serial port (blocking mode -- we use VTIME for timeouts)
    handle->data.serial.fd = open(device, O_RDWR | O_NOCTTY);
    if (handle->data.serial.fd < 0) {
        fprintf(stderr, "Failed to open serial port %s: %s\n", device, strerror(errno));
        return false;
    }

    // Configure serial port
    struct termios tty;
    if (tcgetattr(handle->data.serial.fd, &tty) != 0) {
        fprintf(stderr, "Failed to get serial port attributes: %s\n", strerror(errno));
        close(handle->data.serial.fd);
        return false;
    }

    // Set baud rate
    speed_t speed = baud_to_speed(baud);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 mode
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // Blocking read with 100ms timeout (VTIME is in 1/10s units)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout

    // Apply settings
    if (tcsetattr(handle->data.serial.fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Failed to set serial port attributes: %s\n", strerror(errno));
        close(handle->data.serial.fd);
        return false;
    }

    // Flush buffers
    tcflush(handle->data.serial.fd, TCIOFLUSH);

    strncpy(handle->data.serial.device_path, device, sizeof(handle->data.serial.device_path) - 1);
    return true;
}

static void serial_close(transport_handle_t* handle)
{
    if (handle->data.serial.fd >= 0) {
        close(handle->data.serial.fd);
        handle->data.serial.fd = -1;
    }
}

static int serial_send(transport_handle_t* handle, const uint8_t* data, size_t length)
{
    return write(handle->data.serial.fd, data, length);
}

static int serial_receive(transport_handle_t* handle, uint8_t* buffer, size_t buffer_size, int timeout_ms)
{
    (void)timeout_ms;  // VTIME handles the timeout
    int n = (int)read(handle->data.serial.fd, buffer, buffer_size);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;
        return -1;
    }
    return n;
}

#else

// Stub implementations for unsupported platforms
static bool serial_open(transport_handle_t* handle, const char* device, transport_baud_t baud)
{
    (void)handle; (void)device; (void)baud;
    fprintf(stderr, "Serial port support not available on this platform\n");
    return false;
}

static void serial_close(transport_handle_t* handle) { (void)handle; }
static int serial_send(transport_handle_t* handle, const uint8_t* data, size_t length) {
    (void)handle; (void)data; (void)length;
    return -1;
}
static int serial_receive(transport_handle_t* handle, uint8_t* buffer, size_t buffer_size, int timeout_ms) {
    (void)handle; (void)buffer; (void)buffer_size; (void)timeout_ms;
    return -1;
}

#endif

//=============================================================================
// Public API
//=============================================================================

transport_handle_t* transport_init(
    transport_type_t type,
    const char* device,
    const char* name,
    transport_baud_t baud)
{
    if (!device) {
        return NULL;
    }

    transport_handle_t* handle = calloc(1, sizeof(transport_handle_t));
    if (!handle) {
        return NULL;
    }

    handle->type = type;

    switch (type) {
        case TRANSPORT_TYPE_FAKEUART:
            if (!fakeuart_init(device, name)) {
                free(handle);
                return NULL;
            }
            strncpy(handle->data.fakeuart.bus_name, device, sizeof(handle->data.fakeuart.bus_name) - 1);
            strncpy(handle->data.fakeuart.sender_name, name, sizeof(handle->data.fakeuart.sender_name) - 1);
            break;

        case TRANSPORT_TYPE_SERIAL:
            if (!serial_open(handle, device, baud)) {
                free(handle);
                return NULL;
            }
            break;

        default:
            free(handle);
            return NULL;
    }

    return handle;
}

void transport_close(transport_handle_t* handle)
{
    if (!handle) {
        return;
    }

    switch (handle->type) {
        case TRANSPORT_TYPE_FAKEUART:
            fakeuart_close();
            break;

        case TRANSPORT_TYPE_SERIAL:
            serial_close(handle);
            break;
    }

    free(handle);
}

int transport_send(transport_handle_t* handle, const uint8_t* data, size_t length)
{
    if (!handle || !data) {
        return -1;
    }

    switch (handle->type) {
        case TRANSPORT_TYPE_FAKEUART:
            return fakeuart_send(data, length);

        case TRANSPORT_TYPE_SERIAL:
            return serial_send(handle, data, length);

        default:
            return -1;
    }
}

int transport_receive(
    transport_handle_t* handle,
    uint8_t* buffer,
    size_t buffer_size,
    int timeout_ms,
    char* sender_name,
    size_t sender_name_size)
{
    if (!handle || !buffer) {
        return -1;
    }

    switch (handle->type) {
        case TRANSPORT_TYPE_FAKEUART:
            return fakeuart_receive(buffer, buffer_size, timeout_ms, sender_name, sender_name_size);

        case TRANSPORT_TYPE_SERIAL:
            // Serial ports don't have sender identification
            if (sender_name && sender_name_size > 0) {
                strncpy(sender_name, "serial", sender_name_size - 1);
                sender_name[sender_name_size - 1] = '\0';
            }
            return serial_receive(handle, buffer, buffer_size, timeout_ms);

        default:
            return -1;
    }
}

bool transport_set_promiscuous(transport_handle_t* handle, bool enabled)
{
    if (!handle) {
        return false;
    }

    switch (handle->type) {
        case TRANSPORT_TYPE_FAKEUART:
            fakeuart_set_promiscuous(enabled);
            return true;

        case TRANSPORT_TYPE_SERIAL:
            // Serial ports are always promiscuous (receive all traffic)
            return true;

        default:
            return false;
    }
}

transport_type_t transport_get_type(transport_handle_t* handle)
{
    if (!handle) {
        return TRANSPORT_TYPE_FAKEUART;
    }
    return handle->type;
}

void transport_list_serial_ports(void)
{
#if defined(__APPLE__) || defined(__linux__)
    printf("Available serial ports:\n");

    DIR* dir = opendir("/dev");
    if (!dir) {
        fprintf(stderr, "Failed to open /dev directory\n");
        return;
    }

    struct dirent* entry;
    int count = 0;

    while ((entry = readdir(dir)) != NULL) {
        bool match = false;

#ifdef __APPLE__
        // macOS: cu.* and tty.* devices
        if (strncmp(entry->d_name, "cu.", 3) == 0 || strncmp(entry->d_name, "tty.", 4) == 0) {
            match = true;
        }
#elif defined(__linux__)
        // Linux: ttyUSB*, ttyACM*, ttyS*
        if (strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
            strncmp(entry->d_name, "ttyACM", 6) == 0 ||
            strncmp(entry->d_name, "ttyS", 4) == 0) {
            match = true;
        }
#endif

        if (match) {
            printf("  /dev/%s\n", entry->d_name);
            count++;
        }
    }

    closedir(dir);

    if (count == 0) {
        printf("  (none found)\n");
    }
#else
    printf("Serial port listing not supported on this platform\n");
#endif
}
