#include "tone_source.h"
#include "tone_frames.inc"

int ull_tone_build(size_t frame_index, const uint8_t tx_sequence[2],
                   const uint8_t ack_sequence[2], uint8_t *aggregate,
                   size_t capacity, size_t *written, uint8_t *stream_mask)
{
    return ull_tone_build_mask(frame_index,tx_sequence,ack_sequence,6,
                               aggregate,capacity,written,stream_mask);
}

int ull_tone_build_mask(size_t frame_index, const uint8_t tx_sequence[2],
                        const uint8_t ack_sequence[2], uint8_t active_mask,
                        uint8_t *aggregate, size_t capacity, size_t *written,
                        uint8_t *stream_mask)
{
    if (frame_index >= ULL_TONE_FRAME_COUNT || !tx_sequence || !ack_sequence ||
        (active_mask != 2 && active_mask != 4 && active_mask != 6))
        return AIR_FRAME_ARGUMENT;
    const uint8_t sizes[] = { ULL_TONE_CHANNEL_BYTES };
    const struct air_tx_record records[] = {
        { tone_frames[frame_index][0], ULL_TONE_CHANNEL_BYTES,
          1, tx_sequence[0], ack_sequence[0], 0 },
        { tone_frames[frame_index][1], ULL_TONE_CHANNEL_BYTES,
          2, tx_sequence[1], ack_sequence[1], 0 },
    };
    const struct air_tx_record *first=records+(active_mask==4 ? 1 : 0);
    return air_downlink_build(aggregate, capacity, first, active_mask==6 ? 2u : 1u, sizes, 1, 0,
                             written, stream_mask);
}
