#pragma once
#include <stdbool.h>
#include <stdint.h>
enum { ULL_FB_NOMINAL_Q16=48u<<16, ULL_FB_TARGET_FRAMES=960 };
typedef struct {
    int32_t average_q8;
    uint32_t consumed, value_q16;
    unsigned idle_packets;
    bool active;
} ull_usb_feedback_t;
void ull_usb_feedback_reset(ull_usb_feedback_t *s,uint32_t consumed);
/* Called once for each valid 1 ms USB playback packet. Input and consumption
 * are stereo PCM frames. Result is samples per USB frame in unsigned16.16.
 * No samples are altered; the host adjusts its packet sizes to this rate. */
uint32_t ull_usb_feedback_step(ull_usb_feedback_t *s,uint32_t queued,
                               uint32_t consumed);
