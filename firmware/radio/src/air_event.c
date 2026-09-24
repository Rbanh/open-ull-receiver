// Pinned S3 scheduler experiment. The callback performs no radio operations.
#include "air_event.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "esp_attr.h"

typedef struct { uint32_t hs, hus; } radio_time_t;
typedef struct arb_event arb_event_t;
struct arb_event {
    arb_event_t *next;
    uint32_t hs, hus, limit_hs, duration_hus;
    uint16_t control;
    uint8_t priority, padding;
    void (*start)(arb_event_t *);
    void (*stop)(arb_event_t *);
    void (*cancel)(arb_event_t *);
};
_Static_assert(sizeof(arb_event_t) == 36, "Pinned arbitration element ABI");
_Static_assert(offsetof(arb_event_t, start) == 24, "Start callback offset");
_Static_assert(offsetof(arb_event_t, cancel) == 32, "Cancel callback offset");
extern radio_time_t r_rwip_time_get(void);
extern uint8_t r_sch_arb_insert(arb_event_t *event);
extern uint8_t r_sch_arb_remove(arb_event_t *event, uint8_t active_only);
extern void **r_ip_funcs_p, **r_osi_funcs_p;
extern uint8_t sch_arb_env[], rwip_prog_delay;
extern uint8_t *lld_con_env[], *llc_env[];
extern void ull_controller_diag_record(const uint8_t *, unsigned);

static DRAM_ATTR struct {
    uint32_t before;
    arb_event_t event;
    uint32_t after;
} owned;
static DRAM_ATTR volatile struct {
    uint8_t phase, insert_status, remove_status, cleanup_ok, calls, had_link;
    uint16_t generation, counter;
    uint8_t program_delay;
    radio_time_t arm, requested, callback;
} observed;

static void lock(void) { ((void (*)(void))r_osi_funcs_p[5])(); }
static void unlock(void) { ((void (*)(void))r_osi_funcs_p[6])(); }
static inline int IRAM_ATTR canaries_ok(void)
{
    return owned.before == UINT32_C(0xa1565c01) && owned.after == UINT32_C(0x5310a17e);
}
static inline int IRAM_ATTR active_is_ours(void)
{
    return *(arb_event_t **)(sch_arb_env + 8) == &owned.event;
}
static void IRAM_ATTR started(arb_event_t *event)
{
    if (event != &owned.event || observed.phase != 1 || !canaries_ok()) {
        observed.phase = 5;
        return;
    }
    observed.callback = r_rwip_time_get();
    observed.calls++;
    if (!active_is_ours()) {
        observed.phase = 5;
        return;
    }
    // ROM event-start ISR already popped the element and stored it at env+8.
    // No RF frame is submitted. Release only this active ownership slot.
    observed.remove_status = r_sch_arb_remove(event, 1);
    observed.cleanup_ok = !active_is_ours() && canaries_ok();
    observed.phase = observed.cleanup_ok ? 2 : 5;
}
static void IRAM_ATTR cancelled(arb_event_t *event)
{
    // The software ISR has already removed this element from the cancel list.
    if (event != &owned.event || observed.phase != 1 || !canaries_ok()) {
        observed.phase = 5;
        return;
    }
    observed.callback = r_rwip_time_get();
    observed.calls++;
    observed.cleanup_ok = !active_is_ours();
    observed.phase = 3;
}
static void IRAM_ATTR stopped(arb_event_t *event)
{
    if (event != &owned.event) { observed.phase = 5; return; }
    observed.callback = r_rwip_time_get();
    observed.calls++;
    if (active_is_ours()) observed.remove_status = r_sch_arb_remove(event, 1);
    observed.cleanup_ok = !active_is_ours() && canaries_ok();
    observed.phase = 6;
}

void ull_air_event_reset(void)
{
    lock();
    if (observed.phase == 1) {
        observed.remove_status = r_sch_arb_remove(&owned.event, 0);
        observed.phase = 7;
    }
    unlock();
}

uint8_t ull_air_event_arm(void)
{
    lock();
    if (observed.phase == 1 || active_is_ours()) { unlock(); return 0x0c; }
    if (!r_ip_funcs_p || r_ip_funcs_p[428] != (void *)r_sch_arb_insert ||
        (uintptr_t)sch_arb_env != UINT32_C(0x3fcefb98)) {
        observed.phase = 5;
        unlock();
        return 0x0c;
    }
    uint16_t generation = observed.generation + 1;
    memset((void *)&observed, 0, sizeof(observed));
    memset(&owned, 0, sizeof(owned));
    owned.before = UINT32_C(0xa1565c01);
    owned.after = UINT32_C(0x5310a17e);
    observed.generation = generation;
    observed.arm = r_rwip_time_get();
    owned.event.hs = (observed.arm.hs + 32u) & UINT32_C(0x0fffffff);
    owned.event.hus = observed.arm.hus;
    // Minimum accepted scheduler reservation: 625 half-microseconds.
    // Fixed time, lowest priority, no ASAP rescheduling or automatic retries.
    owned.event.duration_hus = 625;
    owned.event.start = started;
    owned.event.stop = stopped;
    owned.event.cancel = cancelled;
    observed.requested.hs = owned.event.hs;
    observed.requested.hus = owned.event.hus;
    observed.program_delay = rwip_prog_delay;
    observed.had_link = lld_con_env[0] != NULL && llc_env[0] != NULL;
    observed.counter = UINT16_MAX;
    if (observed.had_link) {
        uint16_t counter;
        memcpy(&counter, lld_con_env[0] + 124, sizeof(counter));
        observed.counter = counter;
    }
    observed.phase = 1;
    observed.insert_status = r_sch_arb_insert(&owned.event);
    if (observed.insert_status) observed.phase = 4;
    uint8_t result = observed.insert_status ? 0x0c : 0;
    unlock();
    return result;
}

void ull_air_event_report(void)
{
    uint8_t result[36] = {'U', 'L', 'L', 'A', 1};
    lock();
    result[5] = observed.phase;
    result[6] = observed.insert_status;
    result[7] = observed.remove_status;
    uint16_t v16 = observed.generation;
    memcpy(result + 8, &v16, 2);
    result[10] = observed.program_delay;
    result[11] = observed.cleanup_ok;
    uint32_t v32 = observed.requested.hs;
    memcpy(result + 12, &v32, 4);
    v16 = observed.requested.hus;
    memcpy(result + 16, &v16, 2);
    v16 = observed.callback.hus;
    memcpy(result + 18, &v16, 2);
    v32 = observed.callback.hs;
    memcpy(result + 20, &v32, 4);
    v32 = observed.arm.hs;
    memcpy(result + 24, &v32, 4);
    v16 = observed.arm.hus;
    memcpy(result + 28, &v16, 2);
    v16 = observed.counter;
    memcpy(result + 30, &v16, 2);
    result[32] = observed.calls;
    result[33] = observed.had_link;
    result[34] = canaries_ok();
    result[35] = active_is_ours();
    unlock();
    ull_controller_diag_record(result, sizeof(result));
}
