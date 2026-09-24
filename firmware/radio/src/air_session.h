#ifndef BLACKSHARK_AIR_SESSION_H
#define BLACKSHARK_AIR_SESSION_H

#include <stddef.h>
#include <stdint.h>

/* Pinned stock-derived 95/95-byte, 5-ms Air profile. This is a session plan,
 * not evidence of RF interoperability. Key material must never be logged.
 */
#define ULL_AIR_E2_BYTES 68u
#define ULL_AIR_INTERVAL_US 5000u
#define ULL_AIR_SUBINTERVAL_US 1420u
#define ULL_AIR_SUBEVENTS 3u

struct ull_air_anchor {
    uint32_t hs;             /* Scheduled ACL anchor, 28-bit 312.5-us clock. */
    uint32_t interval_hs;
    uint16_t hus;            /* Half-us within coarse tick, 0..624. */
    uint16_t event_counter;  /* Counter corresponding to THIS anchor. */
};

struct ull_air_session_seed {
    uint32_t access_address;
    uint32_t event_counter;  /* Independent Air counter, not the ACL counter. */
    uint8_t channel_map[5];
    uint8_t receive_enabled; /* Stock FE01 per-stream flag, 0 or 1. */
    uint8_t iv[8];           /* Final on-wire IV, including stock AA XOR. */
    uint8_t key[16];
};

struct ull_air_session_plan {
    uint32_t start_hs;
    uint16_t start_hus;
    uint16_t acl_instant;
    uint32_t event_counter;
    uint32_t access_address;
    uint32_t offset_us;
    uint8_t e2[ULL_AIR_E2_BYTES];
};

/* Input E0 is the actual sent proposal and E1 is the matching received reply.
 * This profile permits the original instant only, 2..32 ACL events ahead of
 * the supplied scheduled anchor, with a 2000..4000-us negotiated offset.
 * No stale-instant adjustment or guessed controller-to-host timestamp conversion.
 * Output unchanged on error. Caller must still recheck lead before enqueueing E2
 * and reserve/program RF for exactly the resulting start time.
 */
int ull_air_session_plan_build(struct ull_air_session_plan *plan,
                               const uint8_t proposal[39], const uint8_t reply[9],
                               const struct ull_air_anchor *anchor,
                               const struct ull_air_session_seed *seed,
                               uint32_t offset_us);

/* Compute any of the three stock-derived subevent starts without accumulated
 * rounding error. This does not arm the radio or advance peer ACK state.
 */
int ull_air_session_time(const struct ull_air_session_plan *plan,
                         uint32_t event_index, uint8_t subevent,
                         uint32_t *hs, uint16_t *hus);

#endif
