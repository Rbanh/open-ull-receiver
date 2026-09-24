#ifndef BLACKSHARK_PCM_STREAM_H
#define BLACKSHARK_PCM_STREAM_H
#include <stdbool.h>
struct ull_audio_source;
bool ull_pcm_stream_init(void);
struct ull_audio_source *ull_pcm_stream_source(void);
void ull_pcm_stream_stats(void);
bool ull_pcm_stream_pause_for_reconnect(void);
bool ull_pcm_stream_paused_for_reconnect(void);
bool ull_pcm_stream_rearm(void); /* Controller, radio and pump already stopped. */
void ull_pcm_stream_resume_after_reconnect(void);
#endif
