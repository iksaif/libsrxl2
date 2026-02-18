/*
 * Shared Test Scenario Harness
 *
 * Abstracts operations common to both the legacy and new SRXL2 stacks.
 * Scenarios are written against this interface and compiled into
 * both test_scenarios_new.c and test_scenarios_legacy.c.
 *
 * MIT License
 */

#ifndef SCENARIO_HARNESS_H
#define SCENARIO_HARNESS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct scenario_harness {
    /* Lifecycle */
    void (*init_master)(struct scenario_harness *h, uint8_t device_id);
    void (*init_slave)(struct scenario_harness *h, uint8_t device_id);

    /* Bus operations */
    void (*inject)(struct scenario_harness *h, const uint8_t *pkt, uint8_t len);
    void (*tick)(struct scenario_harness *h, uint32_t ms);
    void (*reset_capture)(struct scenario_harness *h);

    /* TX capture */
    size_t (*tx_count)(struct scenario_harness *h);
    const uint8_t *(*tx_packet)(struct scenario_harness *h, size_t idx,
                                 uint8_t *len_out);

    /* State queries */
    const char *(*state)(struct scenario_harness *h);
    uint32_t (*baud)(struct scenario_harness *h);
    uint8_t (*peer_count)(struct scenario_harness *h);
    bool (*connected)(struct scenario_harness *h);

    /* Master ops */
    void (*set_channels)(struct scenario_harness *h, const uint16_t *values,
                          uint32_t mask);
    void (*set_failsafe)(struct scenario_harness *h, bool failsafe);

    /* Slave ops */
    void (*set_telemetry)(struct scenario_harness *h, const uint8_t payload[16]);
    bool (*get_channels)(struct scenario_harness *h, uint16_t *values_out,
                          uint32_t *mask_out, bool *is_failsafe_out);

    /* Implementation-specific data */
    void *data;
} scenario_harness_t;

/* Global harness pointer -- set by each runner before calling scenarios */
extern scenario_harness_t *g_harness;
#define H g_harness

/*
 * find_last_tx_of_type -- scan TX packets from end, return pointer to last
 * packet with the given packet_type (byte offset 1).  Returns NULL if none.
 */
static inline const uint8_t *find_last_tx_of_type(scenario_harness_t *h,
                                                    uint8_t pkt_type,
                                                    uint8_t *len_out)
{
    size_t n = h->tx_count(h);
    for (size_t i = n; i > 0; i--) {
        uint8_t plen;
        const uint8_t *p = h->tx_packet(h, i - 1, &plen);
        if (p && p[1] == pkt_type) {
            *len_out = plen;
            return p;
        }
    }
    return NULL;
}

/*
 * complete_handshake -- loop tick(0) until state becomes "RUNNING".
 * Returns true if RUNNING was reached within max_iterations.
 */
static inline bool complete_handshake(scenario_harness_t *h,
                                       int max_iterations)
{
    for (int i = 0; i < max_iterations; i++) {
        const char *s = h->state(h);
        if (s[0] == 'R') /* "RUNNING" */
            return true;
        h->tick(h, 0);
    }
    return (h->state(h)[0] == 'R');
}

#endif /* SCENARIO_HARNESS_H */
