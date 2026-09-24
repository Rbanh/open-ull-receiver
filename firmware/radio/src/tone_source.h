#ifndef BLACKSHARK_TONE_SOURCE_H
#define BLACKSHARK_TONE_SOURCE_H

#include "air_frame.h"

#define ULL_TONE_FRAME_COUNT 201u
#define ULL_TONE_CHANNEL_BYTES 95u
#define ULL_TONE_FRAME_US 5000u

/* One-second faded, low-level stereo LC3plus candidate, plus encoder flush.
 * Pure record builder: no RF, autonomous playback, looping, or ACK advancement.
 * The radio session must supply current TX/ACK sequence per stream (IDs 1/2).
 * Retries use the same index/sequence. The caller advances only on its policy.
 */
int ull_tone_build(size_t frame_index, const uint8_t tx_sequence[2],
                   const uint8_t ack_sequence[2], uint8_t *aggregate,
                   size_t capacity, size_t *written, uint8_t *stream_mask);
int ull_tone_build_mask(size_t frame_index, const uint8_t tx_sequence[2],
                        const uint8_t ack_sequence[2], uint8_t active_mask,
                        uint8_t *aggregate, size_t capacity, size_t *written,
                        uint8_t *stream_mask);

#endif
