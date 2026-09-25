#include "status_probe.h"
#include "parent_ack.h"
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>

static atomic_uint air_seen,acl_seen,known_seen,unknown_seen;
static atomic_uint setup_hidden,unframed_seen,last_unknown;
static atomic_uint control_only_seen,control_only_proprietary;
static atomic_uint control_only_setup_hidden,control_only_last_meta;

static bool known_scalar(const uint8_t *p,unsigned n){
    return ull_parent_ack_wheel(p,n) || ull_parent_ack_media(p,n) ||
        ull_parent_ack_mic(p,n) ||
        (n==6 && p[0]==4 && (p[1]==1 || p[1]==2) && p[2]==0 &&
         p[3]<=100 && p[4]<=100 && p[5]==0) ||
        (n==3 && (p[0]==5 || p[0]==6) &&
         (p[1]==1 || p[1]==2) && p[2]==0);
}

static void observe(unsigned origin,const uint8_t *p,unsigned n){
    if(!p || !n)return;
    if(p[0]==0x0e || p[0]==0x0f || (p[0]>=0xe0 && p[0]<=0xe4)){
        atomic_fetch_add_explicit(&setup_hidden,1,memory_order_relaxed);
        return;
    }
    atomic_fetch_add_explicit(origin==1 ? &air_seen : &acl_seen,1,memory_order_relaxed);
    if(known_scalar(p,n))atomic_fetch_add_explicit(&known_seen,1,memory_order_relaxed);
    else{
        /* Source, length and command byte only. Unknown does not imply battery. */
        atomic_store_explicit(&last_unknown,
            (origin<<24u)|((n&0xffffu)<<8u)|p[0],memory_order_relaxed);
        atomic_fetch_add_explicit(&unknown_seen,1,memory_order_relaxed);
    }
}

void ull_status_probe_air(uint8_t header,const uint8_t *payload,unsigned length){
    if(!payload || length<4 || (header&3u)!=2u)return;
    unsigned n=(unsigned)payload[0]|((unsigned)payload[1]<<8u);
    if(n!=length-4u || payload[2]!=1 || payload[3]!=1){
        atomic_fetch_add_explicit(&unframed_seen,1,memory_order_relaxed);
        return;
    }
    observe(1,payload+4,n);
}
void ull_status_probe_acl(const uint8_t *payload,unsigned length){
    if(length && length<=4092u)observe(2,payload,length);
}
void ull_status_probe_control_only(uint8_t air_header,const uint8_t *plain,unsigned length){
    /* This path runs only after hardware CRC and CCM authentication. The live
     * audio parser still rejects zero-stream frames; do not change its result. */
    if(!plain || ((air_header>>3u)&15u) || !(air_header&1u) || length<3u ||
       (unsigned)plain[2]+3u!=length || !(plain[1]&3u) ||
       (!plain[2] && (plain[1]&3u)!=1u))return;
    atomic_fetch_add_explicit(&control_only_seen,1,memory_order_relaxed);
    if((plain[1]&3u)!=2u || plain[2]<5u)return;
    const uint8_t *p=plain+3;
    unsigned n=(unsigned)p[0]|((unsigned)p[1]<<8u);
    if(n!=(unsigned)plain[2]-4u || p[2]!=1u || p[3]!=1u)return;
    if(p[4]==0x0e || p[4]==0x0f || (p[4]>=0xe0 && p[4]<=0xe4)){
        atomic_fetch_add_explicit(&control_only_setup_hidden,1,memory_order_relaxed);
        return;
    }
    atomic_fetch_add_explicit(&control_only_proprietary,1,memory_order_relaxed);
    atomic_store_explicit(&control_only_last_meta,(n<<8u)|p[4],memory_order_relaxed);
}
void ull_status_probe_snapshot(uint32_t out[15]){
    if(!out)return;
    out[0]=atomic_load_explicit(&air_seen,memory_order_relaxed);
    out[1]=atomic_load_explicit(&acl_seen,memory_order_relaxed);
    out[2]=atomic_load_explicit(&known_seen,memory_order_relaxed);
    out[3]=atomic_load_explicit(&unknown_seen,memory_order_relaxed);
    out[4]=atomic_load_explicit(&setup_hidden,memory_order_relaxed);
    out[5]=atomic_load_explicit(&unframed_seen,memory_order_relaxed);
    out[6]=atomic_load_explicit(&last_unknown,memory_order_relaxed);
    out[7]=atomic_load_explicit(&control_only_seen,memory_order_relaxed);
    out[8]=atomic_load_explicit(&control_only_proprietary,memory_order_relaxed);
    out[9]=atomic_load_explicit(&control_only_setup_hidden,memory_order_relaxed);
    out[10]=atomic_load_explicit(&control_only_last_meta,memory_order_relaxed);
    out[14]=2; /* schema version; other words are zeroed by EP0 */
}
