#include "status_probe.h"
#include "parent_ack.h"
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>

static atomic_uint air_seen,acl_seen,known_seen,unknown_seen;
static atomic_uint setup_hidden,unframed_seen,last_unknown;

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
void ull_status_probe_snapshot(uint32_t out[15]){
    if(!out)return;
    out[0]=atomic_load_explicit(&air_seen,memory_order_relaxed);
    out[1]=atomic_load_explicit(&acl_seen,memory_order_relaxed);
    out[2]=atomic_load_explicit(&known_seen,memory_order_relaxed);
    out[3]=atomic_load_explicit(&unknown_seen,memory_order_relaxed);
    out[4]=atomic_load_explicit(&setup_hidden,memory_order_relaxed);
    out[5]=atomic_load_explicit(&unframed_seen,memory_order_relaxed);
    out[6]=atomic_load_explicit(&last_unknown,memory_order_relaxed);
    out[14]=1; /* schema version; other words are zeroed by EP0 */
}
