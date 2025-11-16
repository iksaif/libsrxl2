/*
MIT License

Copyright (c) 2025 SRXL2 Experiments

Fake UART implementation using UDP multicast
*/

#include "fakeuart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <errno.h>

// Multicast configuration
#define MULTICAST_GROUP "239.255.42.1"
#define BASE_PORT 54200

// Packet format: sender_name_len (1 byte) + sender_name + data
#define MAX_SENDER_NAME_LEN 32

// Internal state
static struct
{
    int socket_fd;
    struct sockaddr_in mcast_addr;
    char device_name[64];
    char bus_name[64];
    bool promiscuous;
    fakeuart_stats_t stats;
    bool initialized;
} g_uart = {
    .socket_fd = -1,
    .initialized = false,
    .promiscuous = false
};

/**
 * @brief Hash a string to a port number
 */
static uint16_t hash_bus_name(const char* name)
{
    uint32_t hash = 5381;
    int c;

    while ((c = *name++))
        hash = ((hash << 5) + hash) + c;

    return BASE_PORT + (hash % 1000);
}

bool fakeuart_init(const char* busName, const char* deviceName)
{
    if (g_uart.initialized)
    {
        fprintf(stderr, "[FakeUART] Already initialized\n");
        return false;
    }

    // Store names
    strncpy(g_uart.bus_name, busName, sizeof(g_uart.bus_name) - 1);
    strncpy(g_uart.device_name, deviceName, sizeof(g_uart.device_name) - 1);

    // Determine port from bus name
    uint16_t port = hash_bus_name(busName);

    printf("[FakeUART:%s] Initializing on bus '%s' (multicast %s:%d)\n",
           deviceName, busName, MULTICAST_GROUP, port);

    // Create UDP socket
    g_uart.socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_uart.socket_fd < 0)
    {
        perror("[FakeUART] socket");
        return false;
    }

    // Allow multiple sockets to use the same port (for multiple devices)
    int reuse = 1;
    if (setsockopt(g_uart.socket_fd, SOL_SOCKET, SO_REUSEADDR,
                   &reuse, sizeof(reuse)) < 0)
    {
        perror("[FakeUART] setsockopt(SO_REUSEADDR)");
        close(g_uart.socket_fd);
        return false;
    }

#ifdef SO_REUSEPORT
    if (setsockopt(g_uart.socket_fd, SOL_SOCKET, SO_REUSEPORT,
                   &reuse, sizeof(reuse)) < 0)
    {
        perror("[FakeUART] setsockopt(SO_REUSEPORT)");
        close(g_uart.socket_fd);
        return false;
    }
#endif

    // Bind to the multicast port
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(port);

    if (bind(g_uart.socket_fd, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0)
    {
        perror("[FakeUART] bind");
        close(g_uart.socket_fd);
        return false;
    }

    // Join multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(g_uart.socket_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   &mreq, sizeof(mreq)) < 0)
    {
        perror("[FakeUART] setsockopt(IP_ADD_MEMBERSHIP)");
        close(g_uart.socket_fd);
        return false;
    }

    // Set up multicast address for sending
    memset(&g_uart.mcast_addr, 0, sizeof(g_uart.mcast_addr));
    g_uart.mcast_addr.sin_family = AF_INET;
    g_uart.mcast_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    g_uart.mcast_addr.sin_port = htons(port);

    // Enable loopback so devices on same machine can see each other's packets
    unsigned char loop = 1;
    if (setsockopt(g_uart.socket_fd, IPPROTO_IP, IP_MULTICAST_LOOP,
                   &loop, sizeof(loop)) < 0)
    {
        perror("[FakeUART] setsockopt(IP_MULTICAST_LOOP)");
    }

    memset(&g_uart.stats, 0, sizeof(g_uart.stats));
    g_uart.initialized = true;

    printf("[FakeUART:%s] Ready\n", deviceName);

    return true;
}

void fakeuart_close(void)
{
    if (!g_uart.initialized)
        return;

    printf("[FakeUART:%s] Closing (TX: %llu packets/%llu bytes, RX: %llu packets/%llu bytes)\n",
           g_uart.device_name,
           g_uart.stats.tx_packets, g_uart.stats.tx_bytes,
           g_uart.stats.rx_packets, g_uart.stats.rx_bytes);

    if (g_uart.socket_fd >= 0)
    {
        close(g_uart.socket_fd);
        g_uart.socket_fd = -1;
    }

    g_uart.initialized = false;
}

int fakeuart_send(const uint8_t* data, uint8_t length)
{
    if (!g_uart.initialized || !data || length == 0)
        return -1;

    // Build packet: [name_len][name][data]
    uint8_t packet[256];
    uint8_t name_len = strlen(g_uart.device_name);
    if (name_len > MAX_SENDER_NAME_LEN)
        name_len = MAX_SENDER_NAME_LEN;

    packet[0] = name_len;
    memcpy(&packet[1], g_uart.device_name, name_len);
    memcpy(&packet[1 + name_len], data, length);

    uint8_t packet_len = 1 + name_len + length;

    // Send to multicast group
    ssize_t sent = sendto(g_uart.socket_fd, packet, packet_len, 0,
                         (struct sockaddr*)&g_uart.mcast_addr,
                         sizeof(g_uart.mcast_addr));

    if (sent < 0)
    {
        perror("[FakeUART] sendto");
        return -1;
    }

    g_uart.stats.tx_packets++;
    g_uart.stats.tx_bytes += length;  // Count only payload bytes

    return length;  // Return payload length, not packet length
}

int fakeuart_receive(uint8_t* buffer, uint8_t maxLength, uint32_t timeout_ms,
                     char* sender_name, uint8_t sender_name_len)
{
    if (!g_uart.initialized || !buffer || maxLength == 0)
        return -1;

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(g_uart.socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                   &tv, sizeof(tv)) < 0)
    {
        perror("[FakeUART] setsockopt(SO_RCVTIMEO)");
        return -1;
    }

    // Temporary receive buffer for full packet
    uint8_t packet[256];

    struct timeval start_time, current_time;
    gettimeofday(&start_time, NULL);

    while (true)
    {
        // Receive packet: [name_len][name][data]
        ssize_t received = recvfrom(g_uart.socket_fd, packet, sizeof(packet), 0,
                                    NULL, NULL);

        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return 0;  // Timeout
            }
            perror("[FakeUART] recvfrom");
            g_uart.stats.rx_errors++;
            return -1;
        }

        if (received > 0)
        {
            // Parse packet header
            if (received < 2)  // Need at least name_len + 1 byte
            {
                g_uart.stats.rx_errors++;
                continue;
            }

            uint8_t name_len = packet[0];
            if (name_len > MAX_SENDER_NAME_LEN || 1 + name_len > received)
            {
                g_uart.stats.rx_errors++;
                continue;
            }

            // Extract sender name
            char pkt_sender[MAX_SENDER_NAME_LEN + 1];
            memcpy(pkt_sender, &packet[1], name_len);
            pkt_sender[name_len] = '\0';

            // Extract data
            uint8_t data_len = received - 1 - name_len;
            if (data_len > maxLength)
            {
                g_uart.stats.rx_errors++;
                continue;
            }

            // Filter our own packets (unless in promiscuous mode)
            if (!g_uart.promiscuous && strcmp(pkt_sender, g_uart.device_name) == 0)
            {
                // Check if we've exceeded the timeout
                gettimeofday(&current_time, NULL);
                long elapsed_ms = (current_time.tv_sec - start_time.tv_sec) * 1000 +
                                 (current_time.tv_usec - start_time.tv_usec) / 1000;

                if (elapsed_ms >= (long)timeout_ms)
                {
                    return 0;  // Timeout
                }
                continue;  // Skip our own packet, try again
            }

            // Copy data to output buffer
            memcpy(buffer, &packet[1 + name_len], data_len);

            // Copy sender name if requested
            if (sender_name && sender_name_len > 0)
            {
                strncpy(sender_name, pkt_sender, sender_name_len - 1);
                sender_name[sender_name_len - 1] = '\0';
            }

            g_uart.stats.rx_packets++;
            g_uart.stats.rx_bytes += data_len;

            return data_len;
        }
    }

    return 0;
}

bool fakeuart_available(void)
{
    if (!g_uart.initialized)
        return false;

    // Check if data is available without blocking
    struct timeval tv = {0, 0};
    fd_set readfds;

    FD_ZERO(&readfds);
    FD_SET(g_uart.socket_fd, &readfds);

    int ret = select(g_uart.socket_fd + 1, &readfds, NULL, NULL, &tv);
    return ret > 0;
}

const char* fakeuart_get_device_name(void)
{
    return g_uart.device_name;
}

void fakeuart_set_promiscuous(bool enabled)
{
    g_uart.promiscuous = enabled;
    printf("[FakeUART:%s] Promiscuous mode %s\n",
           g_uart.device_name,
           enabled ? "ENABLED" : "DISABLED");
}

void fakeuart_get_stats(fakeuart_stats_t* stats)
{
    if (stats)
        *stats = g_uart.stats;
}
