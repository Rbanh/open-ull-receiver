#include "usb_feedback.h"
#include <string.h>

void ull_usb_feedback_reset(ull_usb_feedback_t *s,uint32_t consumed)
{
    memset(s,0,sizeof(*s));s->consumed=consumed;
    s->average_q8=ULL_FB_TARGET_FRAMES*256;
    s->value_q16=ULL_FB_NOMINAL_Q16;
}

uint32_t ull_usb_feedback_step(ull_usb_feedback_t *s,uint32_t queued,
                               uint32_t consumed)
{
    if(consumed!=s->consumed){s->consumed=consumed;s->idle_packets=0;s->active=true;}
    else if(s->idle_packets<100)s->idle_packets++;
    if(!s->active || s->idle_packets>=100){
        s->active=false;s->average_q8=ULL_FB_TARGET_FRAMES*256;
        return s->value_q16=ULL_FB_NOMINAL_Q16;
    }
    if(queued>1920)queued=1920;
    /* 64 ms low-pass suppresses the encoder's 240-frame/5 ms block jitter.
     * Proportional correction has a 4-second settling time constant.
     * A100 ppm oscillator offset leaves only~19 frames target error.
     * Clamp to1000 ppm: this handles clock drift, not CPU throughput failure. */
    s->average_q8+=((int32_t)(queued*256)-s->average_q8)/64;
    int32_t correction=(s->average_q8-ULL_FB_TARGET_FRAMES*256)*64/1000;
    if(correction>3146)correction=3146;
    if(correction< -3146)correction= -3146;
    s->value_q16=(uint32_t)((int32_t)ULL_FB_NOMINAL_Q16-correction);
    return s->value_q16;
}
