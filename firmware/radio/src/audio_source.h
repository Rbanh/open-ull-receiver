#ifndef BLACKSHARK_AUDIO_SOURCE_H
#define BLACKSHARK_AUDIO_SOURCE_H
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include "tone_packet.h"
#include "reconnect_barrier.h"

#define ULL_AUDIO_QUEUE_FRAMES 4u
/* One encoder producer, one serialized controller consumer. Init/reset only
 * while both are stopped. No heap, USB calls, codec work or waiting here.
 * The two latches retain an owned event and its prepared successor so changing
 * ACKs/retrying encryption cannot consume different PCM for the same event. */
struct ull_audio_source {
    atomic_uint written, read;
    atomic_bool requested;
    uint8_t queue[ULL_AUDIO_QUEUE_FRAMES][2][ULL_AIR_AUDIO_CHANNEL_BYTES];
    struct {
        uint32_t event;
        bool valid, present;
        uint8_t data[2][ULL_AIR_AUDIO_CHANNEL_BYTES];
    } latch[2];
    uint32_t last_event;
    atomic_uint selected, underflows, discarded;
    bool selected_any;
};

struct ull_audio_source_stats {
    uint32_t selected, underflows, discarded, queued;
    bool queue_valid;
};
/* Any task. Monotonic counters are individually atomic, not transactional.
 * Queue occupancy is accepted only across a stable consumer read index. */
void ull_audio_source_stats(const struct ull_audio_source *source,
                            struct ull_audio_source_stats *out);

/* Controller only, after radio collection and acknowledged producer pause. */
bool ull_audio_source_rearm(struct ull_audio_source *source,
                           const struct ull_reconnect_barrier *barrier);
void ull_audio_source_init(struct ull_audio_source *source);
/* False means full/invalid; caller retains the encoded frame for retry. */
bool ull_audio_source_push(struct ull_audio_source *source,
                         const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES]);
/* Controller only. 1=copied encoded frame; 0=empty record; -1=invalid/backwards
 * request. An empty selection remains empty on re-encryption. Skipped radio
 * events discard the corresponding queued audio rather than adding latency.
 * Do not wrap event UINT32_MAX to zero within a session: stop/reset instead.
 * Output is untouched on 0/-1. Source counter statistics are consumer-owned. */
int ull_audio_source_select(struct ull_audio_source *source, uint32_t event,
                          uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES]);
#endif
