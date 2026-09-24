#ifndef ULL_PCM_CODEC_H
#define ULL_PCM_CODEC_H

#include <stddef.h>
#include <stdint.h>

/* One owner/task; separate instances must not call the reference codec concurrently.
 * No USB, radio, FreeRTOS, file or device operations are performed here.
 * Float FFT initialization allocates heap; encoding itself must not. */
#define PCM_CODEC_INPUT_HZ 48000
#define PCM_CODEC_OUTPUT_HZ 96000
#define PCM_CODEC_CHANNELS 2
#define PCM_CODEC_INPUT_FRAMES 240
#define PCM_CODEC_OUTPUT_SAMPLES 480
#define PCM_CODEC_CHANNEL_BYTES 95
#define PCM_CODEC_PACKET_BYTES 190
#define PCM_CODEC_ALIGNMENT 16

typedef struct pcm_codec pcm_codec_t;
typedef struct {
    size_t context_bytes;
    size_t encoder_bytes;
    size_t scratch_bytes;
    size_t arena_bytes;
} pcm_codec_layout_t;

/* Arena holds state and PCM; FFT plans allocate extra initialization-only heap. */
pcm_codec_layout_t pcm_codec_layout(void);
void pcm_codec_destroy(pcm_codec_t *codec);
int pcm_codec_init(pcm_codec_t **out, void *arena, size_t arena_bytes);
/* Exactly 240 interleaved stereo S16 frames -> 95 left bytes then 95 right bytes.
 * NULL input means 5 ms of zero PCM (normal encoder/resampler state is retained).
 * On error no packet is valid. Caller must not transmit output from a failed call. */
int pcm_codec_encode(pcm_codec_t *codec, const int16_t *pcm48,
                     uint8_t packet[PCM_CODEC_PACKET_BYTES]);
uint64_t pcm_codec_frames_encoded(const pcm_codec_t *codec);
uint64_t pcm_codec_resampler_clips(const pcm_codec_t *codec);

#endif
