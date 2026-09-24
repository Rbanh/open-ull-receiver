#ifndef ULL_RECONNECT_BARRIER_H
#define ULL_RECONNECT_BARRIER_H
#include <stdatomic.h>
#include <stdbool.h>
/* Single coordinator and single worker. Request stays asserted throughout
 * rearm; acknowledgment is published only after the worker's final write. */
struct ull_reconnect_barrier { atomic_bool requested, acknowledged; };
static inline void ull_reconnect_request(struct ull_reconnect_barrier *b){atomic_store(&b->requested,true);}
static inline bool ull_reconnect_requested(const struct ull_reconnect_barrier *b){return atomic_load(&b->requested);}
static inline void ull_reconnect_acknowledge(struct ull_reconnect_barrier *b){atomic_store(&b->acknowledged,true);}
static inline bool ull_reconnect_paused(const struct ull_reconnect_barrier *b){return atomic_load(&b->requested) && atomic_load(&b->acknowledged);}
static inline void ull_reconnect_resume(struct ull_reconnect_barrier *b){atomic_store(&b->requested,false);}
static inline void ull_reconnect_depart(struct ull_reconnect_barrier *b){atomic_store(&b->acknowledged,false);}
#endif
