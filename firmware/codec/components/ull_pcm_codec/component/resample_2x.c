#include "resample_2x.h"

/* Exact power-of-two scaling of the original Q30 taps, rounded to binary32.
 * Symmetric partner taps use the same coefficient. No filter redesign. */
#ifdef ESP_PLATFORM
__attribute__((section(".dram1")))
#endif
static const float halfband_even_f32[16] = {
    0x0.0p+0f,
    0x1.596f000000000p-14f,
    -0x1.82a4c00000000p-12f,
    0x1.f35c200000000p-11f,
    -0x1.0345580000000p-9f,
    0x1.ddf4c00000000p-9f,
    -0x1.97a8080000000p-8f,
    0x1.48cf100000000p-7f,
    -0x1.fca2740000000p-7f,
    0x1.7d6a040000000p-6f,
    -0x1.184e7e0000000p-5f,
    0x1.991a820000000p-5f,
    -0x1.2e4ae60000000p-4f,
    0x1.d51a120000000p-4f,
    -0x1.a267020000000p-3f,
    0x1.44946c0000000p-1f
};

static inline int16_t round_clip(float value, uint64_t *clips)
{
    /* Accumulation is bounded below 77000 for every valid S16 input history,
     * so conversion to int32 is defined. Truncate after signed half-LSB bias. */
    int32_t rounded = (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
    if (rounded > 32767) { ++*clips; return 32767; }
    if (rounded < -32768) { ++*clips; return -32768; }
    return (int16_t)rounded;
}

#ifdef ESP_PLATFORM
__attribute__((section(".iram1")))
#endif
void resample_2x_process(resample_2x_t *s, const int16_t *in, size_t frames,
                         int16_t *left, int16_t *right)
{
    for (size_t n = 0; n < frames; ++n) {
        s->newest = (s->newest + 1) & 31;
        for (unsigned ch = 0; ch < 2; ++ch) {
            /* Each S16 sample and each 17-bit pair sum is exactly representable
             * in binary32. Mirror the circular history so every filter window
             * is contiguous and each input needs only one integer conversion. */
            float sample = in ? (float)in[2*n + ch] : 0.0f;
            s->history[ch][s->newest] = sample;
            s->history[ch][s->newest + 32] = sample;
            const float *newest = &s->history[ch][s->newest + 32];
            const float *early = newest - 1;
            const float *late = newest - 30;
            float sum = 0.0f;
            for (unsigned k = 1; k < 16; ++k) {
                float pair = *early-- + *late++;
                sum += halfband_even_f32[k] * pair;
            }
            int16_t *out = ch ? right : left;
            out[2*n] = round_clip(sum, &s->clips);
            out[2*n+1] = (int16_t)newest[-15];
        }
    }
}
