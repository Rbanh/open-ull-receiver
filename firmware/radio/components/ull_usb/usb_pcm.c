#include "usb_pcm.h"
#include <string.h>
void ull_pcm_reset(ull_pcm_ring_t *r,uint8_t channels){r->read=0;r->count=0;r->channels=channels;}
size_t ull_pcm_push(ull_pcm_ring_t *r,const int16_t *s,size_t frames){
    if(!s||!frames||!r->channels||r->channels>2)return 0;
    size_t dropped=0;
    if(frames>ULL_PCM_RING_FRAMES){size_t skip=frames-ULL_PCM_RING_FRAMES;s+=skip*r->channels;frames-=skip;dropped+=skip;}
    if(r->count+frames>ULL_PCM_RING_FRAMES){size_t drop=r->count+frames-ULL_PCM_RING_FRAMES;r->read=(r->read+drop)%ULL_PCM_RING_FRAMES;r->count-=drop;dropped+=drop;}
    size_t write=(r->read+r->count)%ULL_PCM_RING_FRAMES;
    size_t first=ULL_PCM_RING_FRAMES-write;if(first>frames)first=frames;
    memcpy(r->samples+write*r->channels,s,first*r->channels*sizeof(*s));
    memcpy(r->samples,s+first*r->channels,(frames-first)*r->channels*sizeof(*s));r->count+=frames;return dropped;
}
size_t ull_pcm_pop(ull_pcm_ring_t *r,int16_t *s,size_t frames){
    if(!s||!r->channels||r->channels>2)return 0;
    if(frames>r->count)frames=r->count;
    size_t first=ULL_PCM_RING_FRAMES-r->read;if(first>frames)first=frames;
    memcpy(s,r->samples+r->read*r->channels,first*r->channels*sizeof(*s));
    memcpy(s+first*r->channels,r->samples,(frames-first)*r->channels*sizeof(*s));r->read=(r->read+frames)%ULL_PCM_RING_FRAMES;r->count-=frames;return frames;
}
void ull_pcm_gain(int16_t *s,size_t count,int32_t gain){
    if(gain==32768)return;
    if(!gain){memset(s,0,count*sizeof(*s));return;}
    for(size_t i=0;i<count;i++)s[i]=(int16_t)(((int32_t)s[i]*gain)/32768);
}
