// Read-only bounded capture at the pinned S3 radio programming boundary.
#include "radio_program.h"
#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
typedef struct { uint32_t hs, hus; } radio_time_t;
extern radio_time_t r_rwip_time_get(void);
extern void **r_ip_funcs_p, **r_osi_funcs_p;
extern uint8_t sch_prog_env[];
extern void *r_emi_get_mem_addr_by_offset(uint16_t);
extern void __real_r_sch_prog_ble_push_hack(const void *, uint8_t);
extern void ull_controller_diag_record(const uint8_t *, unsigned);
#define CAPACITY 8
#define RECORD_SIZE 51
static DRAM_ATTR volatile uint8_t enabled, captured, consumed;
static DRAM_ATTR volatile uint8_t records[CAPACITY][RECORD_SIZE];
static void lock(void) { ((void (*)(void))r_osi_funcs_p[5])(); }
static void unlock(void) { ((void (*)(void))r_osi_funcs_p[6])(); }

void IRAM_ATTR __wrap_r_sch_prog_ble_push_hack(const void *parameters, uint8_t capacity_flag)
{
    // The original routine programs/submits the existing event first, unmodified.
    __real_r_sch_prog_ble_push_hack(parameters, capacity_flag);
    if (!enabled || captured >= CAPACITY) return;
    radio_time_t now = r_rwip_time_get();
    const uint8_t *p = parameters;
    uint8_t index = sch_prog_env[0x101];
    if (index >= 16) { enabled = 0; return; }
    const volatile uint16_t *et = r_emi_get_mem_addr_by_offset((uint16_t)index * 16);
    volatile uint8_t *r = records[captured];
    // Only scheduler metadata and its 16-byte exchange-memory event descriptor.
    // Never read the control structure containing encryption state or packet data.
    for (unsigned i = 0; i < 4; i++) r[i] = now.hs >> (8*i);
    r[4] = now.hus; r[5] = now.hus >> 8;
    r[6] = index; r[7] = capacity_flag;
    for (unsigned i = 0; i < 8; i++) r[8+i] = p[i]; // callback and coarse start
    r[16] = p[8]; r[17] = p[9]; // fine start <=624
    for (unsigned i = 0; i < 8; i++) r[18+i] = p[12+i]; // duration and context
    for (unsigned i = 0; i < 9; i++) r[26+i] = p[20+i];
    for (unsigned i = 0; i < 8; i++) {
        uint16_t value = et[i]; r[35+2*i] = value; r[36+2*i] = value >> 8;
    }
    captured++;
    if (captured == CAPACITY) enabled = 0;
}

uint8_t ull_radio_program_arm(void)
{
    lock();
    if (enabled || !r_ip_funcs_p ||
        r_ip_funcs_p[490] != (void *)__wrap_r_sch_prog_ble_push_hack) {
        unlock(); return 0x0c;
    }
    captured = consumed = 0;
    enabled = 1;
    unlock(); return 0;
}
void ull_radio_program_reset(void)
{
    lock(); enabled = captured = consumed = 0; unlock();
}
void ull_radio_program_report(void)
{
    uint8_t result[59] = {'U','L','L','P',1};
    lock();
    result[5] = consumed < captured ? 0 : enabled ? 1 : 2;
    result[6] = captured; result[7] = consumed;
    if (!result[5]) {
        for (unsigned i=0; i<RECORD_SIZE; i++) result[8+i] = records[consumed][i];
        consumed++;
    }
    unlock();
    ull_controller_diag_record(result, sizeof(result));
}
