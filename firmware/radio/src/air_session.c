#include "air_session.h"
#include "e0_request.h"
#include <string.h>

#define CLOCK_MASK UINT32_C(0x0fffffff)
#define GROUP_SPAN_US (ULL_AIR_SUBINTERVAL_US * ULL_AIR_SUBEVENTS)

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static void put24(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v); p[2] = (uint8_t)(v >> 16);
}
static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16));
}

int ull_air_session_plan_build(struct ull_air_session_plan *out,
                               const uint8_t proposal[39], const uint8_t reply[9],
                               const struct ull_air_anchor *anchor,
                               const struct ull_air_session_seed *seed,
                               uint32_t offset_us)
{
    uint8_t expected[37];
    memcpy(expected, e0_parameters, sizeof(expected));
    if (proposal && proposal[2] == 2) {
        /* Stock second-stream serializer: no microphone on this stream. */
        expected[2] = 2; expected[9] = 0; expected[19] = 0; expected[26] = 1;
    }
    if (!out || !proposal || !reply || !anchor || !seed ||
        memcmp(proposal, expected, 37) || reply[0] != 0xe1 ||
        memcmp(reply + 1, proposal + 31, 8) ||
        anchor->hs > CLOCK_MASK || anchor->hus > 624 ||
        anchor->interval_hs != 96 || seed->receive_enabled > 1 ||
        !seed->access_address || (seed->channel_map[4] & 0xe0) ||
        offset_us < 2000 || offset_us > 4000 ||
        (proposal[2] == 2 && seed->receive_enabled))
        return -1;
    unsigned used = 0;
    for (unsigned i = 0; i < 37; ++i)
        used += (seed->channel_map[i / 8] >> (i % 8)) & 1u;
    if (used < 2) return -1;
    uint16_t instant = get16(proposal + 37);
    uint16_t ahead = (uint16_t)(instant - anchor->event_counter);
    if (ahead < 2 || ahead > 32) return -2;
    struct ull_air_session_plan plan = {0};
    uint32_t fine = (uint32_t)anchor->hus + offset_us * 2;
    plan.start_hs = (anchor->hs + (uint32_t)ahead * anchor->interval_hs + fine / 625) & CLOCK_MASK;
    plan.start_hus = (uint16_t)(fine % 625);
    plan.acl_instant = instant;
    plan.event_counter = seed->event_counter;
    plan.access_address = seed->access_address;
    plan.offset_us = offset_us;
    uint8_t *p = plan.e2;
    p[0] = 0xe2;
    put32(p + 1, seed->access_address);
    put24(p + 5, offset_us);
    put24(p + 8, GROUP_SPAN_US);
    put24(p + 11, GROUP_SPAN_US);
    p[14] = 0; /* Stock template index. */
    put16(p + 15, instant);
    p[17] = 1;
    p[18] = seed->receive_enabled;
    memcpy(p + 19, seed->channel_map, 5);
    put32(p + 24, seed->event_counter);
    /* Stock 081a3288: first reply offset = aggregate airtime plus guard.
     * group+64=1026; next +=302 for an enabled receiver, otherwise142.
     * Remaining unused stream offsets retain the stock ffffffff sentinel.
     */
    put32(p + 28, 1026);
    /* The table is group-wide: stream 1 retains its enabled reply slot when
     * stream 2 (receive disabled) joins the same group. */
    put32(p + 32, 1026 + ((proposal[2] == 2 || seed->receive_enabled) ? 302u : 142u));
    put32(p + 36, UINT32_MAX);
    put32(p + 40, UINT32_MAX);
    memcpy(p + 44, seed->iv, 8);
    memcpy(p + 52, seed->key, 16);
    *out = plan;
    return 0;
}

int ull_air_session_time(const struct ull_air_session_plan *plan,
                         uint32_t event_index, uint8_t subevent,
                         uint32_t *hs, uint16_t *hus)
{
    if (!plan || !hs || !hus || subevent >= ULL_AIR_SUBEVENTS ||
        plan->start_hs > CLOCK_MASK || plan->start_hus > 624)
        return -1;
    uint64_t fine = plan->start_hus +
        (uint64_t)event_index * (ULL_AIR_INTERVAL_US * 2u) +
        (uint64_t)subevent * (ULL_AIR_SUBINTERVAL_US * 2u);
    *hs = (uint32_t)((plan->start_hs + fine / 625) & CLOCK_MASK);
    *hus = (uint16_t)(fine % 625);
    return 0;
}
