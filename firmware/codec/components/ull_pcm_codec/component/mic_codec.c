#include "mic_codec.h"
#include "lc3plus.h"
#include "setup_dec_lc3plus.h"
#include <string.h>
struct mic_codec {LC3PLUS_Dec *decoder;int16_t previous;int initialized;};
static size_t aligned(size_t n){return (n+15u)&~(size_t)15u;}
size_t mic_codec_workspace_bytes(void){return sizeof(DecoderWorkspace);}
size_t mic_codec_size(void){return aligned(sizeof(struct mic_codec))+aligned((size_t)lc3plus_dec_get_size(32000,1));}
int mic_codec_init(mic_codec_t **out,void *arena,size_t bytes){
    if(!out)return -1;
    *out=NULL;
    if(!arena || (uintptr_t)arena%16u || bytes<mic_codec_size())return -1;
    memset(arena,0,mic_codec_size());mic_codec_t *s=arena;
    s->decoder=(LC3PLUS_Dec *)((uint8_t *)arena+aligned(sizeof(*s)));
    int rc=lc3plus_dec_init(s->decoder,32000,1,LC3PLUS_PLC_ADVANCED,0);
    if(!rc)rc=lc3plus_dec_set_frame_dms(s->decoder,LC3PLUS_FRAME_DURATION_5MS);
    if(!rc)rc=lc3plus_dec_set_ep_enabled(s->decoder,0);
    if(!rc && lc3plus_dec_get_output_samples(s->decoder)!=160)rc=-2;
    if(rc)return rc;
    s->initialized=1;*out=s;return 0;
}
int mic_codec_decode(mic_codec_t *s,const uint8_t *frame,int16_t out[240]){
    if(!s || !s->initialized || !out)return -1;
    uint8_t packet[40]={0};int16_t decoded[160],*channels[1]={decoded};
    if(frame)memcpy(packet,frame,sizeof(packet));
    int rc=lc3plus_dec16(s->decoder,packet,frame?40:0,channels,NULL,frame?0:1);
    if(rc && rc!=LC3PLUS_DECODE_ERROR)return rc;
    /* Causal linear 3:2 conversion, continuous across frame boundaries. */
    for(unsigned j=0;j<240;j++){
        unsigned i=j*2u/3u, fraction=j*2u%3u;
        int32_t previous=i?decoded[i-1]:s->previous,current=decoded[i];
        out[j]=(int16_t)((previous*(int32_t)(3u-fraction)+current*(int32_t)fraction)/3);
    }
    s->previous=decoded[159];return rc;
}

int mic_codec_reset(mic_codec_t *s){
    if(!s || !s->initialized)return -1;
    int rc=lc3plus_dec_free_memory(s->decoder);
    s->initialized=0;
    if(rc)return rc;
    mic_codec_t *fresh=NULL;
    return mic_codec_init(&fresh,s,mic_codec_size());
}
