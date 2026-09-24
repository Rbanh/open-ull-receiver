#pragma once
#include <stddef.h>
#include <stdint.h>
typedef struct mic_codec mic_codec_t;
size_t mic_codec_size(void);
size_t mic_codec_workspace_bytes(void);
int mic_codec_init(mic_codec_t **out,void *arena,size_t bytes);
/* One mono 32kHz/5ms/40-byte LC3plus frame ->240 USB48kHz S16 samples.
 * NULL input requests packet-loss concealment; return2 means concealed input. */
int mic_codec_decode(mic_codec_t *codec,const uint8_t *frame,int16_t out[240]);
/* Releases decoder-owned allocations and reinitializes the same arena. */
int mic_codec_reset(mic_codec_t *codec);
