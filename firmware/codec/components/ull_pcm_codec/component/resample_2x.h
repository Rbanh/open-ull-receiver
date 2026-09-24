#ifndef ULL_RESAMPLE_2X_H
#define ULL_RESAMPLE_2X_H
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float history[2][64];
    unsigned newest;
    uint64_t clips;
} resample_2x_t;

/* Zero-initialize state once, then retain it across all USB/codec boundaries.
 * Produces 2*n planar samples per channel, arbitrary chunk sizes accepted.
 * Causal group delay: 31 output samples (322.9167 us). No look-ahead. */
void resample_2x_process(resample_2x_t *state, const int16_t *interleaved,
                         size_t n, int16_t *left96, int16_t *right96);
#endif
