#include "pcm_codec.h"
#include "resample_2x.h"
#include "functions.h"
#include "lc3plus.h"
#include "setup_enc_lc3plus.h"
#include <string.h>
#ifdef ESP_PLATFORM
int ull_codec_profile_active;
extern int64_t esp_timer_get_time(void);
extern void ull_pcm_stage_time(unsigned stage,uint32_t elapsed);
#endif

struct pcm_codec {
    LC3PLUS_Enc *encoder;
    resample_2x_t resampler;
    int16_t planar[2][PCM_CODEC_OUTPUT_SAMPLES];
    uint64_t encoded;
    int ready;
    int initialized;
};

static size_t align16(size_t n) { return (n + 15) & ~(size_t)15; }

pcm_codec_layout_t pcm_codec_layout(void)
{
    pcm_codec_layout_t l = {
        .context_bytes = sizeof(pcm_codec_t),
        .encoder_bytes = (size_t)lc3plus_enc_get_size(96000, 2),
        .scratch_bytes = 0,
    };
    l.arena_bytes = align16(l.context_bytes) + align16(l.encoder_bytes) + align16(l.scratch_bytes);
    return l;
}

int pcm_codec_init(pcm_codec_t **out, void *arena, size_t size)
{
    if (!out) return -1;
    *out = NULL;
    pcm_codec_layout_t l = pcm_codec_layout();
    if (!arena || (uintptr_t)arena % PCM_CODEC_ALIGNMENT || size < l.arena_bytes) return -1;
    memset(arena, 0, l.arena_bytes);
    pcm_codec_t *s = arena;
    s->encoder = (LC3PLUS_Enc *)((uint8_t *)arena + align16(l.context_bytes));

    int rc = lc3plus_enc_init(s->encoder, 96000, 2, 1, NULL);
    if (rc) return rc;
    s->initialized = 1;
    rc = lc3plus_enc_set_frame_dms(s->encoder, LC3PLUS_FRAME_DURATION_5MS);
    if (!rc) rc = lc3plus_enc_set_ep_mode(s->encoder, LC3PLUS_EP_OFF);
    if (!rc) rc = lc3plus_enc_set_bitrate(s->encoder, 304000);
    if (rc) { pcm_codec_destroy(s); return rc; }
    if (lc3plus_enc_get_input_samples(s->encoder) != PCM_CODEC_OUTPUT_SAMPLES ||
        lc3plus_enc_get_num_bytes(s->encoder) != PCM_CODEC_PACKET_BYTES) { pcm_codec_destroy(s); return -2; }
    s->ready = 1;
    *out = s;
    return 0;
}

int pcm_codec_encode(pcm_codec_t *s, const int16_t *pcm48, uint8_t packet[190])
{
    if (!s || !s->ready || !packet) return -1;
#ifdef ESP_PLATFORM
    ull_codec_profile_active=((uint32_t)s->encoded%100u)==0;
    int64_t started=ull_codec_profile_active?esp_timer_get_time():0;
#endif
    resample_2x_process(&s->resampler, pcm48, 240, s->planar[0], s->planar[1]);
#ifdef ESP_PLATFORM
    if(ull_codec_profile_active){ull_pcm_stage_time(0,(uint32_t)(esp_timer_get_time()-started));
    started=esp_timer_get_time();}
#endif
    int16_t *channels[2] = { s->planar[0], s->planar[1] };
    int bytes = 0;
    int rc = lc3plus_enc16(s->encoder, channels, packet, &bytes, NULL);
#ifdef ESP_PLATFORM
    if(ull_codec_profile_active)ull_pcm_stage_time(1,(uint32_t)(esp_timer_get_time()-started));
#endif
    if (rc || bytes != 190) {
        s->ready = 0;
        memset(packet, 0, 190);
        return rc ? rc : -2;
    }
    ++s->encoded;
    return 0;
}

uint64_t pcm_codec_frames_encoded(const pcm_codec_t *s) { return s ? s->encoded : 0; }
uint64_t pcm_codec_resampler_clips(const pcm_codec_t *s) { return s ? s->resampler.clips : 0; }

void pcm_codec_destroy(pcm_codec_t *s) { if (s && s->initialized) { lc3plus_enc_free_memory(s->encoder); s->ready = 0; s->initialized = 0; } }
