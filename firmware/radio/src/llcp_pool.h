#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    void *descriptors[7];
    uint16_t offsets[7];
    uint8_t *memory;
    unsigned count;
} ull_pool_reservation_t;
// All operations must run in the controller task. Never frees an unowned slot.
int ull_pool_free_count(void);
bool ull_pool_reserve(ull_pool_reservation_t *r,unsigned count);
void ull_pool_release(ull_pool_reservation_t *r,unsigned first);
void ull_pool_dry_run(uint8_t result[12]);
