#include "control_log.h"
#include "parent_ack.h"
#include "usb_audio.h"
#include <stdatomic.h>
#include <stdio.h>
// One-shot, controller-task LLCP experiment for the pinned S3 controller.
// Host commands permit ping/E4/E0. E2 has a separate exact-plan internal API.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "llcp_pool.h"
#include "raw_llcp.h"
#include "air_event.h"
#include "radio_program.h"
#include "e4_template.h"
#include "e0_request.h"
extern uint8_t *llc_env[], *lld_con_env[];
extern void **r_ip_funcs_p, **r_osi_funcs_p, **r_plf_funcs_p;
extern void __real_r_ble_util_buf_llcp_tx_free(uint16_t offset);
extern uint8_t r_lld_con_llcp_tx(uint8_t link,void *descriptor);
extern uint8_t __real_r_lld_con_llcp_tx(uint8_t link,void *descriptor);
extern void *r_emi_get_mem_addr_by_offset(uint16_t offset);
extern void *r_ke_malloc(uint32_t size,uint8_t type);
extern void r_ke_free(void *ptr);
extern void r_co_list_push_back(void *list,void *node);
extern void *r_co_list_pop_front(void *list);
extern uint16_t r_lld_con_event_counter_get(uint8_t link);
extern void r_ble_util_buf_rx_free(uint16_t offset,void *data);
// Pinned ROM body 4002bd50 returns coarse/fine time in a2/a3.
typedef struct {uint32_t hs,hus;} radio_time_t;
extern radio_time_t r_rwip_time_get(void);
_Static_assert(sizeof(radio_time_t)==8,"Unexpected radio-clock return ABI");
extern void ull_controller_diag_record(const uint8_t *data,unsigned size);

static ull_pool_reservation_t held;
static bool handed_off;
static uint8_t released_slots,tail_ok;
static bool e4_confirmed;
static int64_t e4_confirmed_us;
static bool e1_expected;
static uint16_t e0_counter;
static int64_t e0_sent_us;
static uint8_t sent_e0[39], received_e1[9];
static struct ull_air_anchor origin_anchor;
static struct ull_air_session_plan prepared_plan;
static uint8_t *session_llc, *session_lld;
static bool e1_received, session_prepared;
static bool e2_confirmed;
static struct ull_air_session_plan committed_first;
static uint8_t *committed_llc, *committed_lld;
static bool second_available;
static int64_t first_commit_us;
static struct {
    bool active,pending;
    uint8_t tx_sn,rx_nesn;
    uint16_t hardware_sequence;
    uint8_t *llc,*lld;
    uint8_t accepted_e1[9];
    struct ull_parent_pdu message;
} air_parent;
static atomic_bool parent_retired;
static bool detached_stereo;
static struct ull_parent_ack detached;
static atomic_uint detached_status[8];
static atomic_uint detached_eligibility;
/* Bounded opt-in empty parent poll before sequence bootstrap. */
static atomic_uint control_probe_frames;
static atomic_bool control_auto_poll=true;
_Static_assert(ATOMIC_BOOL_LOCK_FREE==2 && ATOMIC_INT_LOCK_FREE==2,"parent status lock free");
void ull_raw_parent_retired(void){atomic_store_explicit(&parent_retired,true,memory_order_release);}
void ull_raw_control_probe_set(bool enabled){atomic_store_explicit(&control_probe_frames,enabled?2000u:0u,memory_order_relaxed);}
unsigned ull_raw_control_probe_remaining(void){return atomic_load_explicit(&control_probe_frames,memory_order_relaxed);}
void ull_raw_control_auto_set(bool enabled){atomic_store_explicit(&control_auto_poll,enabled,memory_order_relaxed);}
bool ull_raw_control_auto_get(void){return atomic_load_explicit(&control_auto_poll,memory_order_relaxed);}
static void detached_publish(void){
    unsigned v[8]={detached.active,detached.accepted,detached.duplicates,detached.empty,
        detached.acked,detached.rejected,detached.stale,ull_parent_ack_header(&detached)};
    for(unsigned i=0;i<8;i++)atomic_store_explicit(&detached_status[i],v[i],memory_order_relaxed);
}
void ull_raw_detached_parent_status(void){
    unsigned v[8];for(unsigned i=0;i<8;i++)v[i]=atomic_load_explicit(&detached_status[i],memory_order_relaxed);
    printf("{\"parent_ack\":{\"retired\":%s,\"eligibility\":%u,\"required\":1023,\"active\":%u,\"accepted\":%u,\"duplicates\":%u,\"empty\":%u,\"acked\":%u,\"rejected\":%u,\"stale\":%u,\"header\":%u}}\n",
        atomic_load_explicit(&parent_retired,memory_order_acquire)?"true":"false",
        atomic_load_explicit(&detached_eligibility,memory_order_relaxed),
        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
}
bool ull_raw_session_confirmed(void){return e2_confirmed;}
static uint16_t get16(const uint8_t *p){uint16_t v;memcpy(&v,p,2);return v;}
static uint32_t get32(const uint8_t *p){uint32_t v;memcpy(&v,p,4);return v;}
static void put16(uint8_t *p,uint16_t v){memcpy(p,&v,2);}
static void lock_controller(void){((void (*)(void))r_osi_funcs_p[5])();}
static void unlock_controller(void){((void (*)(void))r_osi_funcs_p[6])();}
static bool parent_sequence(uint8_t *llc,uint8_t *lld,uint16_t *sequence){
    if(!llc || !lld || llc_env[0]!=llc || lld_con_env[0]!=lld || lld[142]>=6 ||
       (get16(lld+132)&0x60)!=0x60 || (get16(llc+66)&0x40))return false;
    volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset((uint16_t)(0x400+90*lld[142]));
    *sequence=cs[12]&0x3000u;return true;
}
static bool air_parent_current(void){
    uint16_t sequence;
    return air_parent.active && parent_sequence(air_parent.llc,air_parent.lld,&sequence) &&
        sequence==air_parent.hardware_sequence;
}
static void air_control_report(uint8_t stage,uint8_t rx_header,uint8_t tx_header){
    uint8_t report[10]={'U','L','L','P',1,stage,rx_header,tx_header,
                       air_parent.tx_sn,air_parent.rx_nesn};
    ull_controller_diag_record(report,sizeof(report));
}
static void clear_session(void){
    e1_received=false;session_prepared=false;session_llc=NULL;session_lld=NULL;
    memset(sent_e0,0,sizeof(sent_e0));memset(received_e1,0,sizeof(received_e1));
    memset(&origin_anchor,0,sizeof(origin_anchor));
    memset(&prepared_plan,0,sizeof(prepared_plan));
}
static bool capture_anchor(struct ull_air_anchor *a){
    const uint8_t *lld=lld_con_env[0];
    if(!lld || !llc_env[0] || get32(lld+72)!=get32(lld+4) ||
       get32(lld+8)>624 || get16(lld+126)>32)return false;
    a->hs=get32(lld+72);a->hus=(uint16_t)get32(lld+8);
    a->interval_hs=get32(lld+100);
    a->event_counter=(uint16_t)(r_lld_con_event_counter_get(0)+get16(lld+126));
    return a->hs<=0x0fffffff && a->interval_hs==96;
}
static bool session_connection_valid(void){
    int64_t elapsed=esp_timer_get_time()-e0_sent_us;
    return e1_received && session_llc && session_lld &&
        llc_env[0]==session_llc && lld_con_env[0]==session_lld &&
        elapsed>=0 && elapsed<1000000 &&
        (get16(session_lld+132)&0x60)==0x60 && !(get16(session_llc+66)&0x40);
}
static bool same_plan(const struct ull_air_session_plan *a,
                      const struct ull_air_session_plan *b){
    return a->start_hs==b->start_hs && a->start_hus==b->start_hus &&
        a->acl_instant==b->acl_instant && a->event_counter==b->event_counter &&
        a->access_address==b->access_address && a->offset_us==b->offset_us &&
        !memcmp(a->e2,b->e2,sizeof(a->e2));
}
/* Bind the current anchor to the original E0 anchor, not just a coincidentally
 * matching event counter after a reconnect or connection-parameter change. */
static bool plan_for_current_anchor(const struct ull_air_session_seed *seed,
                                   uint32_t offset,struct ull_air_session_plan *plan){
    struct ull_air_anchor current={0};
    struct ull_air_session_plan original={0};
    bool connection=session_connection_valid(), captured=connection && capture_anchor(&current);
    int initial=captured?ull_air_session_plan_build(&original,sent_e0,received_e1,&origin_anchor,seed,offset):-99;
    int advanced=(captured && !initial)?ull_air_session_plan_build(plan,sent_e0,received_e1,&current,seed,offset):-99;
    bool same=captured && !initial && !advanced && same_plan(&original,plan);
    if(sent_e0[2]==2){
        uint8_t d[40]={'U','L','L','N',1,connection,captured,(uint8_t)initial,(uint8_t)advanced,same};
        put16(d+10,e0_counter);put16(d+12,origin_anchor.event_counter);put16(d+14,current.event_counter);
        memcpy(d+16,&origin_anchor.hs,4);put16(d+20,origin_anchor.hus);
        memcpy(d+22,&current.hs,4);put16(d+26,current.hus);
        memcpy(d+28,&original.start_hs,4);put16(d+32,original.start_hus);
        if(!advanced){memcpy(d+34,&plan->start_hs,4);put16(d+38,plan->start_hus);}
        ull_controller_diag_record(d,sizeof(d));
    }
    memset(&original,0,sizeof(original));
    return same;
}
uint8_t ull_raw_prepare_session(const struct ull_air_session_seed *seed,
                               uint32_t offset,struct ull_air_session_plan *plan){
    if(!seed || !plan)return 0x12;
    lock_controller();
    struct ull_air_session_plan candidate;
    bool valid=plan_for_current_anchor(seed,offset,&candidate);
    if(valid){prepared_plan=candidate;session_prepared=true;*plan=candidate;}
    memset(&candidate,0,sizeof(candidate));
    unlock_controller();return valid?0:0x0c;
}
bool ull_raw_second_stream_ready(void){
    return e1_received && sent_e0[2]==2;
}
uint8_t ull_raw_prepare_second_stream(const struct ull_air_session_plan *first,
                                     struct ull_air_session_plan *second){
    if(!first || !second)return 0x12;
    uint8_t current_map[5];
    uint8_t diag[24]={'U','L','L','J',2,1};
    if(ull_raw_session_channel_map(current_map) || memcmp(current_map,first->e2+19,5)){
        ull_controller_diag_record(diag,sizeof(diag));return 0x0c;
    }
    lock_controller();
    struct ull_air_session_seed seed={0};
    struct ull_air_session_plan candidate={0};
    bool valid=ull_raw_second_stream_ready() && same_plan(first,&committed_first) &&
        llc_env[0]==committed_llc && lld_con_env[0]==committed_lld;
    diag[5]=2;
    if(valid){
        seed.access_address=first->access_address;
        memcpy(seed.channel_map,first->e2+19,5);
        memcpy(seed.iv,first->e2+44,8);memcpy(seed.key,first->e2+52,16);
        valid=plan_for_current_anchor(&seed,first->offset_us,&candidate);
        diag[5]=3;
    }
    if(valid){
        uint32_t coarse=(candidate.start_hs-first->start_hs)&UINT32_C(0x0fffffff);
        int64_t delta=(int64_t)coarse*625+candidate.start_hus-first->start_hus;
        diag[5]=4;memcpy(diag+6,&first->start_hs,4);put16(diag+10,first->start_hus);
        memcpy(diag+12,&candidate.start_hs,4);put16(diag+16,candidate.start_hus);
        int32_t bounded_delta=(int32_t)delta;memcpy(diag+18,&bounded_delta,4);
        /* A later 30-ms ACL instant must lie on the existing 5-ms Air grid. */
        valid=delta>0 && delta<=4000000 && delta%10000==0;
        if(valid){
            seed.event_counter=first->event_counter+(uint32_t)(delta/10000);
            valid=plan_for_current_anchor(&seed,first->offset_us,&candidate);
            diag[5]=5;
        }
    }
    if(valid){prepared_plan=candidate;session_prepared=true;*second=candidate;diag[5]=0;}
    memset(&seed,0,sizeof(seed));memset(&candidate,0,sizeof(candidate));
    unlock_controller();ull_controller_diag_record(diag,sizeof(diag));return valid?0:0x0c;
}
uint8_t ull_raw_session_channel_map(uint8_t channel_map[5]){
    if(!channel_map)return 0x12;
    lock_controller();
    bool valid=session_connection_valid();
    if(valid){
        uint8_t cs_index=session_lld[142];
        valid=cs_index<6;
        if(valid){
            volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset(
                (uint16_t)(0x400+90*cs_index));
            /* Same pinned CS channel-map fields used by radio_tx.c. */
            uint16_t lo=cs[17],mid=cs[18],hi=cs[19];
            channel_map[0]=lo;channel_map[1]=lo>>8;
            channel_map[2]=mid;channel_map[3]=mid>>8;channel_map[4]=hi&31;
            unsigned count=0;
            for(unsigned i=0;i<5;i++)count+=__builtin_popcount(channel_map[i]);
            valid=count>=2;
        }
    }
    unlock_controller();return valid?0:0x0c;
}
uint8_t ull_raw_session_crc_init(uint32_t *crc_init){
    if(!crc_init)return 0x12;
    lock_controller();
    bool valid=session_prepared && session_connection_valid();
    if(valid){
        /* Pinned LLD +142 is its CS index; CS +16/+18 hold CRCInit.
         * Stock 081c6a9c copies the parent ACL descriptor's CRC into Air
         * descriptor +4..6 for role1. Original-live-session-03 and the
         * independent C3 original-air-stream-02 capture confirm that seed. */
        uint8_t cs_index=session_lld[142];
        valid=cs_index<6;
        if(valid){
            volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset(
                (uint16_t)(0x400+90*cs_index));
            *crc_init=(uint32_t)cs[8] | ((uint32_t)(cs[9]&0xffu)<<16);
        }
    }
    unlock_controller();return valid?0:0x0c;
}
uint8_t ull_raw_parent_poll_header(uint8_t *header){
    if(!header)return 0x12;
    lock_controller();
    uint8_t *llc=llc_env[0],*lld=lld_con_env[0];
    bool valid=llc && lld &&
        ((llc==committed_llc && lld==committed_lld) ||
         (llc==session_llc && lld==session_lld)) &&
        (get16(lld+132)&0x60)==0x60 && !(get16(llc+66)&0x40) && lld[142]<6;
    if(valid){
        volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset(
            (uint16_t)(0x400+90*lld[142]));
        /* Pinned S3 ROM40019a2c locates TXRXCNTL at CS+24, matching
         * live snapshot0x300b and the saved RW field layout SN13/NESN12.
         * Treat this as a bounded polling probe, not a full parent handoff. */
        *header=(uint8_t)(1u|((cs[12]>>10)&12u));
    }
    unlock_controller();return valid?0:0x0c;
}
uint8_t ull_raw_parent_air_pdu(struct ull_parent_pdu *pdu){
    if(!pdu)return 0x12;
    lock_controller();
    bool active=air_parent.active,valid=!active || air_parent_current();
    if(!valid){
        uint16_t current=0;bool connected=parent_sequence(air_parent.llc,air_parent.lld,&current);
        uint8_t report[10]={'U','L','L','P',2,(uint8_t)connected};
        put16(report+6,air_parent.hardware_sequence);put16(report+8,current);
        ull_controller_diag_record(report,sizeof(report));
    }
    if(valid && active){
        memset(pdu,0,sizeof(*pdu));
        pdu->header=(uint8_t)((air_parent.pending?3u:1u)|
                             (air_parent.tx_sn<<3)|(air_parent.rx_nesn<<2));
        if(air_parent.pending){pdu->length=air_parent.message.length;
            memcpy(pdu->payload,air_parent.message.payload,pdu->length);}
    }
    unlock_controller();
    if(!valid)return 0x0c;
    if(!active){memset(pdu,0,sizeof(*pdu));return ull_raw_parent_poll_header(&pdu->header);}
    return 0;
}

/* Controller lock held. Never inspect retained LLC/LLD pointers here. */
static bool detached_eligible(void){
    unsigned mask=(atomic_load_explicit(&parent_retired,memory_order_acquire)?1u:0u) |
        (detached_stereo?2u:0u) | (!llc_env[0]?4u:0u) | (!lld_con_env[0]?8u:0u) |
        (e2_confirmed?16u:0u) | (!second_available?32u:0u) | (!e1_expected?64u:0u) |
        (!session_prepared?128u:0u) | (!handed_off?256u:0u) | (!air_parent.pending?512u:0u);
    atomic_store_explicit(&detached_eligibility,mask,memory_order_relaxed);
    return mask==1023u;
}
bool ull_raw_detached_parent_air_pdu(struct ull_parent_pdu *pdu){
    if(!pdu)return false;
    memset(pdu,0,sizeof(*pdu));lock_controller();
    /* Caller promises confirmed stereo mask6 in this same session. */
    detached_stereo=true;
    bool eligible=detached_eligible();
    bool valid=eligible && detached.active;
    if(valid)pdu->header=ull_parent_ack_header(&detached);
    else if(eligible){
        unsigned remaining=atomic_load_explicit(&control_probe_frames,memory_order_relaxed);
        if(remaining || atomic_load_explicit(&control_auto_poll,memory_order_relaxed)){
            if(remaining)atomic_store_explicit(&control_probe_frames,remaining-1u,memory_order_relaxed);
            pdu->header=1u; /* Empty LL data PDU, SN=0, NESN=0. */
            valid=true;
        }
    }
    unlock_controller();return valid;
}

bool ull_raw_receive_air_control(const struct ull_air_control_rx *received,
                                 const struct ull_parent_pdu *sent){
    if(!received || !sent || !received->present || received->length>sizeof(received->payload) ||
       sent->length>sizeof(sent->payload) || (received->header&0xe0u) ||
       (sent->header&~15u))return false;
    lock_controller();
    bool detached_path=detached_eligible(),detached_ok=false,fresh_control=false;
    if(detached_path){
        unsigned before=detached.accepted;
        detached_ok=ull_parent_ack_receive(&detached,received->header,received->payload,
            received->length,sent->header,sent->length);
        fresh_control=detached_ok && detached.accepted!=before;
        detached_publish();
    }
    unlock_controller();
    if(detached_path){
        ull_control_log_air(received->header,received->payload,received->length);
        if(fresh_control && (received->header&3u)==2 && received->payload[2]==1 &&
           received->payload[3]==1){
            const uint8_t *control=received->payload+4;
            if(received->length==10 && ull_parent_ack_wheel(control,6)){
                if(control[3]==1)ull_usb_audio_consumer_key(ULL_USB_VOLUME_UP);
                else if(control[3]==2)ull_usb_audio_consumer_key(ULL_USB_VOLUME_DOWN);
            }else if(received->length==6 && ull_parent_ack_media(control,2) && control[1]==2){
                ull_usb_audio_consumer_key(ULL_USB_PLAY_PAUSE);
            }else if(received->length==7 && ull_parent_ack_mic(control,3)){
                ull_usb_audio_set_headset_mic_mute(control[0]==5);
            }
        }
        return detached_ok;
    }
    bool is_e1=(received->header&3u)==3 && received->length==9 && received->payload[0]==0xe1;
    bool empty=(received->header&3u)==1 && received->length==0;
    if(!is_e1 && !empty){
        ull_control_log_air(received->header,received->payload,received->length);
        return false; /* Never swallow an unknown parent procedure. */
    }
    lock_controller();
    bool valid=false,returned_to_parent=false;uint8_t stage=0;
    if(!air_parent.active){
        uint16_t hardware;int64_t elapsed=esp_timer_get_time()-e0_sent_us;
        valid=is_e1 && e1_expected && sent_e0[2]==2 && elapsed>=0 && elapsed<1000000 &&
            !sent->length && (sent->header&3u)==1 &&
            ((received->header>>3)&1u)==((sent->header>>2)&1u) &&
            ((received->header>>2)&1u)!=((sent->header>>3)&1u) &&
            session_llc==committed_llc && session_lld==committed_lld &&
            parent_sequence(session_llc,session_lld,&hardware) &&
            !handed_off && !*(void **)(session_llc+40) && !*(void **)(session_lld+36) &&
            !(get16(session_llc+66)&2u) &&
            !memcmp(received->payload+1,sent_e0+31,6) && get16(received->payload+7)==e0_counter;
        if(valid){
            e1_expected=false;e1_received=true;session_prepared=false;
            memcpy(received_e1,received->payload,9);
            memset(&air_parent,0,sizeof(air_parent));air_parent.active=true;
            air_parent.llc=session_llc;air_parent.lld=session_lld;
            air_parent.hardware_sequence=hardware;
            air_parent.tx_sn=(received->header>>2)&1u;
            air_parent.rx_nesn=(uint8_t)(((received->header>>3)&1u)^1u);
            memcpy(air_parent.accepted_e1,received->payload,9);stage=1;
        }
    }else{
        valid=air_parent_current() && (empty || !memcmp(received->payload,air_parent.accepted_e1,9));
        if(valid){
            /* Only a reply to the exact pending PDU can acknowledge that PDU.
             * An old empty poll, stale prepared frame or duplicate reply may
             * not mark E2 complete. No ROM descriptors or queues are freed. */
            bool exact=(sent->header>>3&1u)==air_parent.tx_sn &&
                sent->length==(air_parent.pending?air_parent.message.length:0) &&
                (sent->header&3u)==(air_parent.pending?3u:1u) &&
                (!sent->length || !memcmp(sent->payload,air_parent.message.payload,sent->length));
            if(exact && ((received->header>>2)&1u)!=air_parent.tx_sn){
                air_parent.tx_sn^=1u;
                if(air_parent.pending){
                    air_parent.pending=false;e2_confirmed=true;stage=3;
                    memset(&air_parent.message,0,sizeof(air_parent.message));
                }
            }
            if(((received->header>>3)&1u)==air_parent.rx_nesn)air_parent.rx_nesn^=1u;
            /* Two acknowledged Air PDUs can bring the software sequence back
             * to the untouched parent hardware sequence. Return polling to
             * the controller only after checking actual equality; otherwise
             * retain the guard. This does not write a guessed CS value. */
            uint16_t hardware;
            uint16_t software=(uint16_t)((air_parent.tx_sn<<13)|(air_parent.rx_nesn<<12));
            if(stage==3 && parent_sequence(air_parent.llc,air_parent.lld,&hardware) &&
               software==hardware){air_parent.active=false;returned_to_parent=true;}
        }
    }
    unlock_controller();
    if(stage)air_control_report(stage,received->header,sent->header);
    if(returned_to_parent)air_control_report(4,received->header,sent->header);
    return valid;
}
static bool prepared_plan_current(const struct ull_air_session_plan *plan){
    if(!plan || !session_prepared || !same_plan(plan,&prepared_plan))return false;
    struct ull_air_session_seed seed={0};
    seed.access_address=plan->access_address;seed.event_counter=plan->event_counter;
    seed.receive_enabled=plan->e2[18];memcpy(seed.channel_map,plan->e2+19,5);
    memcpy(seed.iv,plan->e2+44,8);memcpy(seed.key,plan->e2+52,16);
    struct ull_air_session_plan current;
    bool valid=plan_for_current_anchor(&seed,plan->offset_us,&current) && same_plan(plan,&current);
    memset(&seed,0,sizeof(seed));memset(&current,0,sizeof(current));
    return valid;
}
void ull_raw_reset_sequence(void){
    ull_air_event_reset();
    ull_radio_program_reset();
    lock_controller();
    atomic_store_explicit(&parent_retired,false,memory_order_release);
    detached_stereo=false;ull_parent_ack_reset(&detached);detached_publish();
    atomic_store_explicit(&control_probe_frames,0,memory_order_relaxed);
    atomic_store_explicit(&detached_eligibility,0,memory_order_relaxed);
    e4_confirmed=false;e1_expected=false;e2_confirmed=false;clear_session();
    second_available=false;committed_llc=NULL;committed_lld=NULL;
    memset(&air_parent,0,sizeof(air_parent));
    memset(&committed_first,0,sizeof(committed_first));unlock_controller();
}
uint8_t ull_radio_timing_snapshot(void){
    uint8_t r[34]={'U','L','L','S',1,1};
    /* Pinned S3 ROM r_lld_reset_reg (body4001f8e4) writes these four
     * timing registers. Read the runtime PHY values before choosing any
     * turnaround experiment; foreign RW register headers are not authority
     * for changing this controller. All five accesses are read-only. */
    uint8_t physical[25]={'U','L','L','I',1};
    static const uintptr_t timing_registers[5]={
        UINT32_C(0x60031000),UINT32_C(0x60031090),UINT32_C(0x60031094),
        UINT32_C(0x60031098),UINT32_C(0x6003109c)};
    uint8_t identity[12]={'U','L','L','G',1};bool identity_valid=false;
    lock_controller();
    for(unsigned i=0;i<5;i++){
        uint32_t value=*(volatile const uint32_t *)timing_registers[i];
        memcpy(physical+5+4*i,&value,4);
    }
    uint8_t *lld=lld_con_env[0];
    if(lld && llc_env[0]){
        radio_time_t now=r_rwip_time_get();
        r[5]=0;
        put16(r+6,r_lld_con_event_counter_get(0));
        memcpy(r+8,&now.hs,4);put16(r+12,(uint16_t)now.hus);
        memcpy(r+14,lld+126,2); // Events pending until the programmed event.
        memcpy(r+16,lld+72,4); // LLD scheduled anchor, coarse clock.
        memcpy(r+20,lld+100,4); // Connection interval, coarse clock units.
        memcpy(r+24,lld+4,4); // Arbitration start, coarse clock.
        memcpy(r+28,lld+8,4); // Arbitration start, fine clock.
        memcpy(r+32,lld+132,2);
        uint8_t cs_index=lld[142];
        if(cs_index<6){
            volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset((uint16_t)(0x400+90*cs_index));
            uint16_t aa0=cs[6],aa1=cs[7],crc0=cs[8],crc1=cs[9];
            put16(identity+5,aa0);put16(identity+7,aa1);
            put16(identity+9,crc0);identity[11]=(uint8_t)crc1;
            identity_valid=true;
        }
    }
    unlock_controller();
    ull_controller_diag_record(r,sizeof(r));
    ull_controller_diag_record(physical,sizeof(physical));
    if(identity_valid)ull_controller_diag_record(identity,sizeof(identity));
    return r[5]?0x0c:0;
}
// Called in the controller RX dispatcher after the passive trace copied bytes.
// Consume one exact expected E1 and reproduce the stock handler's common exit:
// ip[65](ind+4,ind+8), then optional coexistence scheduling via plf[60]/ip[388].
// No standard LLCP state transition is applicable to this proprietary response.
bool ull_raw_receive_e1(const void *indication,uint16_t destination){
    if(!indication || (destination>>8)!=0)return false;
    const uint8_t *ind=indication;uint8_t *payload;
    memcpy(&payload,ind+8,sizeof(payload));
    if(ind[2]!=9 || !payload || payload[0]!=0xe1)return false;
    lock_controller();
    uint8_t *llc=llc_env[0],*lld=lld_con_env[0];
    bool valid=e1_expected && esp_timer_get_time()-e0_sent_us<5000000 && llc && lld &&
        (get16(lld+132)&0x60)==0x60 && !(get16(llc+66)&0x40) &&
        r_ip_funcs_p[65]==(void *)r_ble_util_buf_rx_free && r_plf_funcs_p && r_plf_funcs_p[60] &&
        memcmp(payload+1,e0_parameters+31,6)==0 && get16(payload+7)==e0_counter;
    uint8_t result[16]={'U','L','L','R',1,1,0};
    if(valid){
        e1_expected=false;e1_received=true;session_prepared=false;
        memcpy(received_e1,payload,9);memcpy(result+7,payload,9);
    }
    unlock_controller();
    if(!valid)return false;
    ((void (*)(uint16_t,void *))r_ip_funcs_p[65])(get16(ind+4),payload);
    uint8_t *coex=((uint8_t *(*)(void))r_plf_funcs_p[60])();
    if(coex[32])((void (*)(uint8_t))r_ip_funcs_p[388])(0);
    ull_controller_diag_record(result,sizeof(result));
    return true;
}
static void actual_free(uint16_t offset){
    __real_r_ble_util_buf_llcp_tx_free(offset);
}
uint8_t __wrap_r_lld_con_llcp_tx(uint8_t link,void *descriptor){
    const uint8_t *d=descriptor;
    const uint8_t *payload=r_emi_get_mem_addr_by_offset(get16(d+4));
    // Feature negotiation is not secret. Never log encryption setup contents.
    if((d[6]==9 && (payload[0]==8 || payload[0]==9 || payload[0]==14 || payload[0]==20 || payload[0]==21)) ||
       (d[6]==2 && (payload[0]==7 || payload[0]==13)) || (d[6]==3 && payload[0]==17)){
        uint8_t r[16]={'U','L','L','F',1,link,d[6]};
        memcpy(r+7,payload,d[6]);ull_controller_diag_record(r,sizeof(r));
    }
    return __real_r_lld_con_llcp_tx(link,descriptor);
}
// All lower-driver frees pass through this SDK function-table entry, including
// ACK and disconnect cleanup. Free the six extra slots when it frees the first.
void IRAM_ATTR __wrap_r_ble_util_buf_llcp_tx_free(uint16_t offset){
    lock_controller();
    if(handed_off && held.count && held.offsets[0]==offset){
        handed_off=false;released_slots=held.count;tail_ok=1;
        for(unsigned i=186;i<189;i++)if(held.memory[i]!=0xa5)tail_ok=0;
        actual_free(offset);
        for(unsigned i=1;i<held.count;i++)actual_free(held.offsets[i]);
        memset(&held,0,sizeof(held));
    }else actual_free(offset);
    unlock_controller();
}
bool ull_raw_free_hook_ready(void){
    return r_ip_funcs_p && r_ip_funcs_p[62]==(void *)__wrap_r_ble_util_buf_llcp_tx_free;
}
static void acknowledged(uint8_t link,uint8_t opcode){
    uint8_t r[12]={'U','L','L','T',1,2,opcode,link,0,0,0,0};
    r[8]=released_slots;r[9]=tail_ok;r[10]=ull_pool_free_count();
    if(opcode==0xe4 && link==0 && released_slots==7 && tail_ok){
        e4_confirmed=true;e4_confirmed_us=esp_timer_get_time();
    }
    if(opcode==0xe2 && link==0 && released_slots==7 && tail_ok)e2_confirmed=true;
    ull_controller_diag_record(r,sizeof(r));
}
static uint8_t raw_send(uint8_t opcode,const struct ull_air_session_plan *plan){
    const uint8_t ping=0x12;
    uint8_t e0[39]={0};
    if(opcode!=0x12 && opcode!=0xe4 && opcode!=0xe0 && opcode!=0xe2)return 0x12;
    if(opcode==0xe2 && !plan)return 0x12;
    const uint8_t *payload=opcode==0xe4?e4_template:opcode==0xe0?e0:opcode==0xe2?plan->e2:&ping;
    unsigned length=opcode==0xe4?sizeof(e4_template):opcode==0xe0?sizeof(e0):opcode==0xe2?68:1;
    struct ull_air_anchor proposal_anchor;
    if(opcode==0xe0){
        memcpy(e0,e0_parameters,sizeof(e0_parameters));
        if(plan){e0[2]=2;e0[9]=0;e0[19]=0;e0[26]=1;}
    }
    uint8_t r[12]={'U','L','L','T',1,1,payload[0],0,0,0,0,0};
    uint8_t status=0x0c;void *node=NULL;
    lock_controller();
    uint8_t *llc=llc_env[0],*lld=lld_con_env[0];
    // Single link only, both encryption directions active, empty LLC control
    // queue, and no lower-driver control descriptor. Never bypass a procedure.
    if(!ull_raw_free_hook_ready()){r[8]=1;goto done;}
    if(!llc || !lld){r[8]=2;goto done;}
    if((get16(lld+132)&0x60)!=0x60){r[8]=3;goto done;}
    if(handed_off || *(void **)(llc+40) || *(void **)(lld+36) || (get16(llc+66)&2)){r[8]=4;goto done;}
    if(r_ip_funcs_p[218]!=(void *)r_lld_con_llcp_tx){r[8]=5;goto done;}
    // E0 is allowed once, shortly after this session's E4 TX confirmation.
    // E2 requires the retained actual E0/E1 and precisely the prepared plan.
    if(opcode==0xe0){
        if(plan){
            if(!second_available || !e2_confirmed || !same_plan(plan,&committed_first) ||
               llc!=committed_llc || lld!=committed_lld ||
               esp_timer_get_time()-first_commit_us>1000000){r[8]=9;goto done;}
        }else if(!e4_confirmed || esp_timer_get_time()-e4_confirmed_us>5000000){r[8]=9;goto done;}
    }
    if(opcode==0xe0 && !capture_anchor(&proposal_anchor)){r[8]=10;goto done;}
    if(opcode==0xe2 && !prepared_plan_current(plan)){r[8]=11;goto done;}
    if(!ull_pool_reserve(&held,7)){r[8]=6;goto done;}
    node=r_ke_malloc(length+12,2);
    if(!node){r[8]=7;ull_pool_release(&held,0);goto done;}
    memset(node,0,length+12);
    if(opcode==0xe0){
        uint16_t counter=r_lld_con_event_counter_get(0);
        /* The second E1 can wait for the first stream's activation. Measured
         * diagnostic-02: origin67, proposed78, reply current81. Give the second
         * transaction 24 future events (720ms), still within the 2..32 guard.
         * Never slide an already-sent instant or accept a stale response. */
        uint16_t instant=(uint16_t)(counter+(plan?24:12));
        put16(e0+37,instant);
        uint8_t timing[16]={'U','L','L','E',1,0};
        put16(timing+6,counter);put16(timing+8,instant);
        memcpy(timing+10,e0+31,6);
        ull_controller_diag_record(timing,sizeof(timing));
    }
    void (*callback)(uint8_t,uint8_t)=acknowledged;
    _Static_assert(sizeof(callback)==4,"Unexpected controller callback ABI");
    memcpy((uint8_t *)node+4,&callback,4);
    ((uint8_t *)node)[8]=length;memcpy((uint8_t *)node+9,payload,length);
    memset(held.memory,0xa5,189);memcpy(held.memory,payload,length);
    ((uint8_t *)held.descriptors[0])[6]=length;
    released_slots=0;tail_ok=0;handed_off=true;
    // Set the existing pending flag before enqueue: normal TX-check must never
    // index its standard-opcode table using a proprietary opcode.
    put16(llc+66,get16(llc+66)|2);
    r_co_list_push_back(llc+40,node);
    status=((uint8_t (*)(uint8_t,void *))r_ip_funcs_p[218])(0,held.descriptors[0]);
    if(status){
        handed_off=false;
        r_co_list_pop_front(llc+40);put16(llc+66,get16(llc+66)&~2);
        ull_pool_release(&held,0);r_ke_free(node);r[8]=8;
    }else if(opcode==0xe0 || opcode==0xe4){
        if(opcode==0xe4){
            second_available=false;committed_llc=NULL;committed_lld=NULL;
            memset(&committed_first,0,sizeof(committed_first));
        }else if(plan){second_available=false;}
        clear_session();
        e2_confirmed=false;
        e4_confirmed=false;
        e1_expected=opcode==0xe0;
        if(e1_expected){
            e0_counter=get16(e0+37);e0_sent_us=esp_timer_get_time();
            memcpy(sent_e0,e0,sizeof(sent_e0));origin_anchor=proposal_anchor;
            session_llc=llc;session_lld=lld;
        }
    }else if(opcode==0xe2){
        if(sent_e0[2]==1){
            committed_first=*plan;committed_llc=llc;committed_lld=lld;
            first_commit_us=esp_timer_get_time();second_available=true;
        }else{
            memset(&committed_first,0,sizeof(committed_first));
            committed_llc=NULL;committed_lld=NULL;second_available=false;
        }
        clear_session();
    }
done:
    r[9]=status;r[10]=length;r[11]=ull_pool_free_count();
    unlock_controller();
    ull_controller_diag_record(r,sizeof(r));
    return status;
}
uint8_t ull_raw_send(uint8_t opcode){
    if(opcode==0xe2)return 0x12;
    return raw_send(opcode,NULL);
}
uint8_t ull_raw_commit_session(const struct ull_air_session_plan *plan){
    if(air_parent.active){
        lock_controller();
        bool valid=plan && sent_e0[2]==2 && air_parent_current() &&
            !air_parent.pending && !handed_off &&
            !*(void **)(air_parent.llc+40) && !*(void **)(air_parent.lld+36) &&
            !(get16(air_parent.llc+66)&2u) && prepared_plan_current(plan);
        if(valid){
            memcpy(air_parent.message.payload,plan->e2,68);air_parent.message.length=68;
            air_parent.pending=true;e2_confirmed=false;session_prepared=false;
            second_available=false;
        }
        unlock_controller();
        if(valid)air_control_report(2,0,(uint8_t)(3u|(air_parent.tx_sn<<3)|(air_parent.rx_nesn<<2)));
        return valid?0:0x0c;
    }
    return raw_send(0xe2,plan);
}
uint8_t ull_raw_request_second_stream(const struct ull_air_session_plan *first){
    if(!first)return 0x12;
    return raw_send(0xe0,first);
}
