#pragma once
#include <stdbool.h>
#include <stdint.h>
bool ull_mic_stream_init(void);
void ull_mic_stream_enable(bool enabled);
void ull_mic_stream_new_session(void);
void ull_air_microphone_frame(uint32_t frame,uint8_t sequence,const uint8_t payload[40]);
void ull_mic_stream_stats(void);
