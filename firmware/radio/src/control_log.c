#include "parent_ack.h"
#include "control_log.h"
#include "status_probe.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <stdatomic.h>
#include <inttypes.h>
#include <stdio.h>

enum {CAPACITY=32, PAYLOAD_BYTES=6, ORIGIN_AIR=1, ORIGIN_ACL=2};
struct record {
    int64_t us;
    uint16_t length;
    uint8_t origin,header,opcode,opcode_valid,copied;
    uint8_t payload[PAYLOAD_BYTES];
};
static DRAM_ATTR struct record records[CAPACITY];
static DRAM_ATTR portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR atomic_bool enabled;
static DRAM_ATTR atomic_uint generation;
_Static_assert(ATOMIC_BOOL_LOCK_FREE==2,"Disabled logger check must be lock-free");
static unsigned head,tail,count,dropped,suppressed,total;

/* Key/configuration-bearing proprietary setup is deliberately never recorded,
 * including malformed or truncated instances. Nothing reads private material. */
static bool setup_opcode(uint8_t op){return op==0x0e || op==0x0f || (op>=0xe0 && op<=0xe4);}
/* Payload allowlist, not a length-based guess. All other payloads are omitted.
 * Known stream volume and mute commands contain only bounded scalar controls. */
static unsigned safe_payload(const uint8_t *p,unsigned n){
    if(ull_parent_ack_wheel(p,n))return 6;
    if(ull_parent_ack_media(p,n))return 2;
    if(ull_parent_ack_mic(p,n))return 3;
    if(n==6 && p[0]==4 && (p[1]==1 || p[1]==2) && p[2]==0 &&
       p[3]<=100 && p[4]<=100 && p[5]==0)return 6;
    if(n==3 && (p[0]==5 || p[0]==6) && (p[1]==1 || p[1]==2) && p[2]==0)return 3;
    return 0;
}
static void publish(unsigned origin,uint8_t header,const uint8_t *p,unsigned n){
    unsigned capture=atomic_load_explicit(&generation,memory_order_relaxed);
    if(!atomic_load_explicit(&enabled,memory_order_relaxed))return;
    struct record r={.us=esp_timer_get_time(),.length=(uint16_t)n,
                     .origin=(uint8_t)origin,.header=header};
    const uint8_t *control=NULL;unsigned length=0;
    bool hide=false;
    if(origin==ORIGIN_ACL){control=p;length=n;}
    else if((header&3u)==3 && n){
        /* LL_ENC_REQ/RSP carry session diversifiers/IVs. Never log those,
         * or proprietary E0/E2/setup. Unknown LLCP gets opcode metadata only. */
        hide=p[0]==3 || p[0]==4 || setup_opcode(p[0]);
        r.opcode=p[0];r.opcode_valid=1;
    }else if((header&3u)==2 && n>=4){
        /* Only a complete unfragmented CID0101 PDU has a known opcode offset.
         * Other CIDs/fragments remain header/length only (no raw prefix). */
        unsigned l=(unsigned)p[0]|(unsigned)p[1]<<8;
        if(l==n-4 && p[2]==1 && p[3]==1){control=p+4;length=l;}
    }
    if(control && length){
        r.opcode=control[0];r.opcode_valid=1;hide=setup_opcode(control[0]);
        if(!hide){r.copied=(uint8_t)safe_payload(control,length);
            for(unsigned i=0;i<r.copied;i++)r.payload[i]=control[i];}
    }
    /* Both RX producers and console use this same tiny publication lock.
     * No formatting, allocation, timer reads or input scanning under it. */
    portENTER_CRITICAL(&mux);
    if(atomic_load_explicit(&enabled,memory_order_relaxed) &&
       capture==atomic_load_explicit(&generation,memory_order_relaxed)){
        if(hide)suppressed++;
        else if(count==CAPACITY)dropped++;
        else{records[head]=r;head=(head+1u)%CAPACITY;count++;total++;}
    }
    portEXIT_CRITICAL(&mux);
}
void ull_control_log_air(uint8_t header,const uint8_t *payload,unsigned length){
    if(length && length<=60 && payload){
        ull_status_probe_air(header,payload,length);
        publish(ORIGIN_AIR,header,payload,length);
    }
}
void ull_control_log_acl(const uint8_t *payload,unsigned length){
    if(length<=4092 && (!length || payload)){
        ull_status_probe_acl(payload,length);
        publish(ORIGIN_ACL,0,payload,length);
    }
}
void ull_control_log_enable(bool value){
    portENTER_CRITICAL(&mux);
    atomic_store_explicit(&enabled,false,memory_order_relaxed);
    atomic_fetch_add_explicit(&generation,1,memory_order_relaxed);
    if(value){head=tail=count=dropped=suppressed=total=0;}
    atomic_store_explicit(&enabled,value,memory_order_relaxed);
    portEXIT_CRITICAL(&mux);
}
void ull_control_log_status(void){
    portENTER_CRITICAL(&mux);
    bool on=atomic_load_explicit(&enabled,memory_order_relaxed);
    unsigned queued=count,lost=dropped,hidden=suppressed,seen=total;
    portEXIT_CRITICAL(&mux);
    flockfile(stdout);
    printf("{\"controls\":{\"enabled\":%s,\"queued\":%u,\"recorded\":%u,\"dropped\":%u,\"suppressed_setup\":%u,\"capacity\":%u}}\n",
           on?"true":"false",queued,seen,lost,hidden,CAPACITY);
    funlockfile(stdout);
}
void ull_control_log_drain(void){
    ull_control_log_enable(false);
    /* Capture stays disabled. Late publishers recheck enabled under mux and
     * cannot publish after freeze. Console is the only enable/drain owner. */
    for(;;){
        struct record r;bool present;
        portENTER_CRITICAL(&mux);
        present=count!=0;
        if(present){r=records[tail];tail=(tail+1u)%CAPACITY;count--;}
        portEXIT_CRITICAL(&mux);
        if(!present)break;
        flockfile(stdout);
        printf("{\"control\":{\"source\":\"%s\",\"us\":%" PRId64 ",\"length\":%u,\"header\":%u,\"opcode\":",
               r.origin==ORIGIN_AIR?"air":"acl0101",r.us,r.length,r.header);
        if(r.opcode_valid)printf("%u",r.opcode);else printf("null");
        printf(",\"payload\":\"");
        for(unsigned i=0;i<r.copied;i++)printf("%02x",r.payload[i]);
        printf("\",\"payload_omitted\":%s}}\n",r.copied?"false":"true");
        funlockfile(stdout);
    }
    ull_control_log_status();
}
