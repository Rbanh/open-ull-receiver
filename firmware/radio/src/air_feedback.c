#include "air_feedback.h"
#include "esp_attr.h"
#include "air_frame.h"
#include "tone_packet.h"
#include "tone_source.h"
#include "mbedtls/platform_util.h"
#include <string.h>
#include "mic_stream.h"
#include "status_probe.h"

int IRAM_ATTR ull_air_feedback_accept_frame(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot,
                             uint32_t frame,
                             struct ull_air_control_rx *control_out)
{
    if(!state || !plan || !snapshot || plan->e2[0]!=0xe2 ||
       snapshot->hs>UINT32_C(0x0fffffff) || snapshot->hus>624 || plan->start_hus>624) return -1;
    const uint16_t *d=snapshot->descriptor;
    uint8_t header=(uint8_t)d[2];
    unsigned length=d[2]>>8;
    /* CCM's standard AAD masks bits used by the Air bitmap, so MIC alone is
     * insufficient. Reject hardware CRC failures before reading any records. */
    if((d[1]&9u) || length<7 || length!=snapshot->copied ||
       length>sizeof(snapshot->payload)) return -2;
    uint32_t expected_hs;uint16_t expected_hus;
    if(ull_air_session_time(plan,frame,0,&expected_hs,&expected_hus))return -1;
    uint32_t coarse=(snapshot->hs-expected_hs)&UINT32_C(0x0fffffff);
    if(coarse>=UINT32_C(0x08000000)) return -1;
    int64_t elapsed=(int64_t)coarse*625+snapshot->hus-expected_hus;
    if(elapsed<0 || elapsed>=ULL_AIR_INTERVAL_US*2) return -1;
    uint8_t plain[64]={0};
    if(ull_ble_ccm_decrypt(plan->e2+52,plan->e2+44,0,0,(uint8_t)(header&0xe3),
                            snapshot->payload,length,plain,sizeof(plain))) return -3;
    const uint8_t sizes[2]={40,95};
    struct air_rx_record records[4];
    struct air_control_record control;
    size_t count=0;
    int result=air_uplink_parse_control(plain,length-4,header,
                                        sizes,2,records,4,&count,&control);
    if(result || !count) {
        if((header&0x79u)==1u)
            ull_status_probe_control_only(header,plain,length-4);
        mbedtls_platform_zeroize(plain,sizeof(plain));return -4;
    }
    struct ull_air_feedback updated=*state;
    bool changed=false;
    for(size_t i=0;i<count;i++) {
        const struct air_rx_record *r=&records[i];
        if(r->stream_id<1 || r->stream_id>2) {
            mbedtls_platform_zeroize(plain,sizeof(plain));return -4;
        }
        struct ull_air_feedback_channel *ch=&updated.channels[r->stream_id-1];
        if(ch->valid && frame<ch->frame) continue;
        uint8_t ack=(r->flags&8u) ? 0 : (uint8_t)((r->tx_sequence+1u)&15u);
        if(!ch->valid || ch->frame!=frame || ch->expected_tx!=r->ack_sequence || ch->next_ack!=ack) {
            if(!ch->valid || ch->frame!=frame || ch->expected_tx!=r->ack_sequence)ch->after_tx=0;
            ch->frame=frame;ch->expected_tx=r->ack_sequence;ch->next_ack=ack;
            ch->valid=1;changed=true;
        }
    }
    struct ull_air_control_rx owned={0};
    if(control.present){
        if(control.length>sizeof(owned.payload)){
            mbedtls_platform_zeroize(plain,sizeof(plain));return -4;
        }
        owned.present=1;owned.header=control.header;owned.length=(uint8_t)control.length;
        if(control.length)memcpy(owned.payload,control.payload,control.length);
    }
    if(changed) {updated.generation++;*state=updated;}
    if(control_out)*control_out=owned;
    /* Commit capture only after CRC, CCM, every record and control validate. */
    for(size_t i=0;i<count;i++)if(records[i].stream_id==1 && records[i].length==40)
        ull_air_microphone_frame(frame,records[i].tx_sequence,records[i].payload);
    mbedtls_platform_zeroize(&owned,sizeof(owned));
    mbedtls_platform_zeroize(plain,sizeof(plain));
    return 0;
}

int ull_air_feedback_accept_control(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot,
                             struct ull_air_control_rx *control_out)
{
    if(!plan || !snapshot || snapshot->hus>624 || plan->start_hus>624)return -1;
    uint32_t coarse=(snapshot->hs-plan->start_hs)&UINT32_C(0x0fffffff);
    if(coarse>=UINT32_C(0x08000000))return -1;
    int64_t elapsed=(int64_t)coarse*625+snapshot->hus-plan->start_hus;
    if(elapsed<0 || elapsed>=(int64_t)ULL_TONE_FRAME_COUNT*10000)return -1;
    return ull_air_feedback_accept_frame(state,plan,snapshot,
                                         (uint32_t)(elapsed/10000),control_out);
}

int ull_air_feedback_accept(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot)
{ return ull_air_feedback_accept_control(state,plan,snapshot,NULL); }

void IRAM_ATTR ull_air_feedback_note_tx(struct ull_air_feedback *state,uint32_t frame,
                              uint8_t stream_mask,const uint8_t sequences[2])
{
    bool changed=false;
    for(unsigned i=0;i<2;i++){
        struct ull_air_feedback_channel *ch=&state->channels[i];
        if((stream_mask&(1u<<(i+1))) && ch->valid && ch->frame==frame &&
           ch->expected_tx==((sequences[i]+1u)&15u) && !ch->after_tx){
            ch->after_tx=1;changed=true;
        }
    }
    if(changed)state->generation++;
}

static uint32_t expiry_age(uint32_t frame,uint32_t start)
{
    return frame>start ? frame-start-1u : 0;
}
void IRAM_ATTR ull_air_feedback_sequences(const struct ull_air_feedback *state,
                                 uint32_t frame,const uint32_t starts[2],
                                 uint8_t tx_sequence[2],uint8_t ack_sequence[2])
{
    for(unsigned i=0;i<2;i++) {
        const struct ull_air_feedback_channel *ch=&state->channels[i];
        uint32_t age=expiry_age(frame,starts[i]);
        tx_sequence[i]=(uint8_t)(age&15u);ack_sequence[i]=0;
        if(ch->valid && ch->frame>=starts[i] && frame>=ch->frame) {
            uint32_t advance=age-expiry_age(ch->frame,starts[i]);
            if(ch->after_tx && advance)advance--;
            tx_sequence[i]=(uint8_t)((ch->expected_tx+advance)&15u);
            ack_sequence[i]=ch->next_ack;
        }
    }
}
