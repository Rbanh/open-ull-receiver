#pragma once
#include <stdint.h>
#include <stddef.h>
#define ULL_PCM_RING_FRAMES 1920
// Caller serializes access. Capacity/count are frames, channels is 1 or 2.
typedef struct { int16_t samples[ULL_PCM_RING_FRAMES*2]; uint32_t read, count; uint8_t channels; } ull_pcm_ring_t;
void ull_pcm_reset(ull_pcm_ring_t *r,uint8_t channels);
size_t ull_pcm_push(ull_pcm_ring_t *r,const int16_t *samples,size_t frames);
size_t ull_pcm_pop(ull_pcm_ring_t *r,int16_t *samples,size_t frames);
void ull_pcm_gain(int16_t *samples,size_t count,int32_t gain_q15);
