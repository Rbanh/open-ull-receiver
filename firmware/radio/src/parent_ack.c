#include "parent_ack.h"
#include <string.h>
void ull_parent_ack_reset(struct ull_parent_ack *s){memset(s,0,sizeof(*s));}
uint8_t ull_parent_ack_header(const struct ull_parent_ack *s){
    return s->active?(uint8_t)(1u|(s->tx_sn<<3)|(s->rx_nesn<<2)):0;
}
bool ull_parent_ack_receive(struct ull_parent_ack *s,uint8_t h,const uint8_t *p,
    unsigned n,uint8_t sent,unsigned sent_length){
    bool empty=(h&3u)==1 && n==0;
    bool wheel=(h&3u)==2 && n==10 && p && p[0]==6 && p[1]==0 &&
        p[2]==1 && p[3]==1 && ull_parent_ack_wheel(p+4,6);
    bool media=(h&3u)==2 && n==6 && p && p[0]==2 && p[1]==0 &&
        p[2]==1 && p[3]==1 && ull_parent_ack_media(p+4,2);
    bool mic=(h&3u)==2 && n==7 && p && p[0]==3 && p[1]==0 &&
        p[2]==1 && p[3]==1 && ull_parent_ack_mic(p+4,3);
    if((h&0xe0u) || !(h&3u) || n>60 || (n && !p)){s->rejected++;return false;}
    if(!s->active){
        /* No pending local data exists. Seed from peer's next expected bit.
         * This is an experimental detached-link bootstrap, not proof of the
         * retired controller's final sequence. Only exact wheel data seeds. */
        /* A bounded local poll may have sent LLID1/SN0/NESN0. The live
         * headset answered with wheel LLID2/SN0/NESN1 (header 0x06),
         * acknowledging that actual poll. No other sent packet bootstraps. */
        if(!(wheel || media || mic || empty) || (empty && sent!=1) ||
           sent_length || (sent!=0 && sent!=1) ||
           (sent==1 && ((h>>2)&1u)!=1u)){s->rejected++;return false;}
        s->active=true;s->tx_sn=(h>>2)&1u;s->rx_nesn=((h>>3)&1u)^1u;
        if(empty)s->empty++;else s->accepted++;return true;
    }
    if(sent_length || sent!=ull_parent_ack_header(s)){
        s->stale++;return false;
    }
    /* Core5.4 LL4.5.9: TX acknowledgement is independent of received SN
     * and payload. The caller already authenticated this owned Air PDU.
     * Exact actually-sent empty metadata remains required above. */
    if(((h>>2)&1u)!=s->tx_sn){s->tx_sn^=1u;s->acked++;}
    if(((h>>3)&1u)!=s->rx_nesn){
        /* Duplicate payload is ignored, including length/kind changes. MD
         * does not distinguish new data. Never apply or re-consume it. */
        s->duplicates++;return true;
    }
    /* This experiment flow-controls new unknown data rather than accepting
     * arbitrary LLCP. Its independent acknowledgement was still processed. */
    if(!empty && !wheel && !media && !mic){s->rejected++;return false;}
    s->rx_nesn^=1u;
    if(wheel || media || mic)s->accepted++;else s->empty++;
    return true;
}
