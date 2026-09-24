#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef struct {
    bool mounted,playback_active,capture_active;
    uint32_t playback_packets,playback_frames,playback_dropped_frames,playback_buffered_frames;
    uint32_t capture_packets,capture_frames,capture_underflow_frames,capture_buffered_frames,capture_dropped_frames;
    uint32_t radio_playback_frames,radio_capture_frames,cdc_dropped_bytes,malformed_playback_packets;
    uint32_t usb_task_core,usb_irq_core,cdc_tx_buffered_bytes,cdc_fifo_free;
} ull_usb_audio_stats_t;
esp_err_t ull_usb_audio_init(void);
void ull_usb_audio_stats(ull_usb_audio_stats_t *out);
void ull_usb_audio_print_stats(void);
void ull_usb_audio_print_level(void);
// Task-context API only: USB 48kHz signed16 LE PCM, playback interleaved L/R, capture mono.
// Returns actual frames; playback does not manufacture silence. Volume/mute applied on read.
size_t ull_usb_audio_read_playback(int16_t *stereo,size_t frames);
size_t ull_usb_audio_write_capture(const int16_t *mono,size_t frames);
enum {ULL_USB_PLAY_PAUSE=0x00cd,ULL_USB_VOLUME_UP=0x00e9,ULL_USB_VOLUME_DOWN=0x00ea};
bool ull_usb_audio_consumer_key(uint16_t usage);
void ull_usb_audio_set_headset_mic_mute(bool muted);
bool ull_usb_audio_headset_mic_muted(void);
void ull_usb_disconnect_phy(void);
bool ull_usb_bootloader_allowed(void);
