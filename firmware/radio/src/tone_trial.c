#include "esp_attr.h"
#include "tone_trial.h"
#include "tone_packet.h"
#include "tone_source.h"
#include "raw_llcp.h"
#include "radio_tx.h"
#include "phy_packet.h"
#include "air_feedback.h"
#include "audio_source.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/platform_util.h"
#include <stdbool.h>
#include <string.h>
#include <stdatomic.h>

typedef struct {uint32_t hs, hus;} radio_time_t;
extern radio_time_t r_rwip_time_get(void);
extern void ull_controller_diag_record(const uint8_t *,unsigned);
#define MASK UINT32_C(0x0fffffff)
/* 0=TX, 1=receive-only, 2=alternating RX/TX, 3=two bootstrap RX intervals then
 * native TX with a longer first-slot reply window on every interval.
 * 4=two bootstrap RX intervals, then TX followed by dedicated slot1 RX. */
#ifndef ULL_RX_DIAGNOSTIC
#define ULL_RX_DIAGNOSTIC 4
#endif
/* Diagnostic negative control: one half of TX intervals have a sequence eight
 * steps away from the authenticated expectation. Same framing and valid MIC;
 * the stock receiver should reject these records before delivering audio. */
#ifndef ULL_SEQUENCE_CONTROL
#define ULL_SEQUENCE_CONTROL 0
#endif
#ifndef ULL_CONTROL_GAP
#define ULL_CONTROL_GAP 0
#endif
#ifndef ULL_EMPTY_BOOTSTRAP
#define ULL_EMPTY_BOOTSTRAP 1
#endif
#ifndef ULL_PARENT_POLL
#define ULL_PARENT_POLL 1
#endif
#ifndef ULL_POLL_TX_SHIFT_US
#define ULL_POLL_TX_SHIFT_US 20
#endif
#ifndef ULL_AUDIO_REPEAT
#define ULL_AUDIO_REPEAT 0
#endif
enum phase {IDLE, WAIT_E2, TRANSMITTING, FINISHED, CANCELLED, FAILED, FAULT};
struct prepared_frame {
    uint32_t inner_hs, outer_hs, build_us, generation;
    uint16_t inner_hus, outer_hus;
    uint32_t index;
    uint8_t channel, header, stream_mask, poll_header, tx_sequence[2], ack_sequence[2];
    struct ull_parent_pdu parent;
    bool valid, receive, stereo_audio, cache_reused;
    size_t length;
    uint8_t wire[ULL_PHY_PACKET_MAX];
};
static struct {
    struct ull_air_session_plan plan;
    uint32_t next_frame, submitted, skipped, reports, tx_count;
    uint8_t phase, reason;
    bool cancel;
    uint32_t requested_hs, build_us, crc_init;
    uint16_t requested_hus;
    uint32_t active_frame;
    uint8_t preparation_failure;
    bool first_confirmed, second_requested, second_committed, second_confirmed;
    uint32_t stream_starts[2];
    struct ull_air_feedback feedback;
    uint32_t received_valid, received_rejected;
    int64_t setup_started_us;
    struct prepared_frame prepared[2], alternate[2], gap[2];
} trial;
static struct ull_audio_source *audio_source;
static atomic_uint control_last_data_ms;
static atomic_uint control_retry_policy[3]; /* repeated, held single, data RX */
void ull_tone_trial_control_retry_status(uint32_t out[4]){
    for(unsigned i=0;i<3;i++)out[i]=atomic_load_explicit(&control_retry_policy[i],memory_order_relaxed);
    unsigned last=atomic_load_explicit(&control_last_data_ms,memory_order_relaxed);
    out[3]=last ? (unsigned)(esp_timer_get_time()/1000)-last : UINT32_MAX;
}
static bool control_retry_quiet(void){
    unsigned last=atomic_load_explicit(&control_last_data_ms,memory_order_relaxed);
    return !last || (unsigned)(esp_timer_get_time()/1000)-last>=5000u;
}
static bool control_retry_slot(const struct prepared_frame *p){
    /* Reserve every fourth 30 ms parent poll as a single-send opportunity.
     * That lets the first wheel event through without waiting for a previous
     * control reply to open the five-second activity guard. */
    return p->index>=trial.stream_starts[1] &&
           ((p->index-trial.stream_starts[1])/6u)%4u!=0u;
}
static bool retry_audio_eligible(const struct prepared_frame *p){
    /* Repeat stable audio packets. A control-bearing packet is repeated only
     * when its exact parent header and ACK vector matched a prebuilt cache;
     * a fresh CCM rebuild gets the short single-send reservation. */
    return audio_source && !p->receive && p->stereo_audio &&
        p->stream_mask==6 &&
        ((p->length==203 && p->header==0x30) ||
         (p->length==205 && p->header==0x32 && p->cache_reused &&
          control_retry_quiet() && control_retry_slot(p)));
}
static bool control_poll_due(uint32_t index){
    /* The detached parent's verified control interval is 30 ms. At a 5 ms
     * audio period, one poll every six frames preserves that cadence. */
    return index>=trial.stream_starts[1] &&
           (index-trial.stream_starts[1])%6u==0u;
}
static bool continuous;
static atomic_uint stream_state[8];
void ull_tone_trial_stream_state(uint32_t out[8]){
    for(unsigned i=0;i<8;i++)out[i]=atomic_load(&stream_state[i]);
}
bool ull_tone_trial_continuous(bool value){
    if(trial.phase!=IDLE)return false;
    continuous=value;return true;
}
/* One probe per boot, explicitly armed by console. Never persists in NVS or
 * enables a repeating stream. Only the controller consumes the pending ticket. */
/* Console reserves owner; controller consumes bounded, once-per-boot run. */
static atomic_uint retry_owner; /* 0free,1oneprobe,2burst,3automatic */
static atomic_uint retry_burst[6],retry_burst_started_ms;
static bool burst_active,burst_watch,burst_lead;
static atomic_bool retry_auto_enabled=true;
static atomic_uint retry_auto_values[7]; /* active,blocked,attempt,done,fail,skip,warmup */
static atomic_uint live_retry_errors[2]; /* scheduler conflict, deadline expired */
static uint32_t auto_last_frame=UINT32_MAX;
static bool auto_failed_skip_pending;
static atomic_uint auto_cooldown,auto_stable_frames,auto_recoveries,auto_cooldown_since;
static bool auto_last_submitted_repeat;
static atomic_uint auto_cooldown_probe; /* 0unused,1requested,2consumed/closed */
bool ull_tone_trial_retry_cooldown_probe_arm(void){
    if(!atomic_load(&retry_auto_enabled) || atomic_load(&retry_owner)!=3 || atomic_load(&retry_auto_values[1]))return false;
    unsigned expected=0;
    if(!atomic_compare_exchange_strong(&auto_cooldown_probe,&expected,1u))return false;
    if(!atomic_load(&retry_auto_enabled) || atomic_load(&retry_owner)!=3 || atomic_load(&retry_auto_values[1])){
        expected=1;atomic_compare_exchange_strong(&auto_cooldown_probe,&expected,2u);return false;
    }
    return true;
}
void ull_tone_trial_retry_auto_recovery_status(uint32_t out[4]){
    if(!out)return;
    out[0]=atomic_load(&auto_cooldown);out[1]=atomic_load(&auto_stable_frames);
    out[2]=atomic_load(&auto_recoveries);
    uint32_t elapsed=(uint32_t)((uint32_t)(esp_timer_get_time()/1000)-atomic_load(&auto_cooldown_since));
    out[3]=out[0] && elapsed<250u?250u-elapsed:0;
}
static void auto_ordinary_completed(bool stereo){
    if(!atomic_load(&auto_cooldown))return;
    if(!stereo || auto_last_submitted_repeat){atomic_store(&auto_stable_frames,0);return;}
    if(atomic_load(&auto_stable_frames)<50u)atomic_fetch_add(&auto_stable_frames,1);
}

void ull_tone_trial_retry_auto_enable(bool enabled){atomic_store(&retry_auto_enabled,enabled);}
void ull_tone_trial_retry_auto_status(uint32_t out[8]){
    if(!out)return;
    out[0]=atomic_load(&retry_auto_enabled);
    for(unsigned i=0;i<7;i++)out[i+1]=atomic_load(&retry_auto_values[i]);
}
void ull_tone_trial_retry_error_counts(uint32_t out[2]){
    for(unsigned i=0;i<2;i++)out[i]=atomic_load_explicit(&live_retry_errors[i],memory_order_relaxed);
}
static void auto_release(void){
    atomic_store(&retry_auto_values[0],0);
    if(atomic_load(&retry_owner)==3)atomic_store(&retry_owner,0);
}
static void auto_block(void){
    atomic_store(&auto_cooldown,0);atomic_store(&auto_stable_frames,0);
    atomic_store(&retry_auto_values[1],1);burst_watch=false;burst_lead=false;auto_release();
}
static void auto_cooldown_start(void){
    atomic_store(&auto_cooldown_since,(uint32_t)(esp_timer_get_time()/1000));
    atomic_store(&auto_stable_frames,0);atomic_store(&auto_cooldown,1);
    burst_watch=false;burst_lead=false;auto_release();
}
static bool auto_take(bool eligible){
    if(!eligible || trial.cancel || !atomic_load(&retry_auto_enabled) || atomic_load(&retry_auto_values[1]))return false;
    unsigned owner=atomic_load(&retry_owner);
    if(owner!=0 && owner!=3)return false; /* Explicit pending tests win. */
    if(owner==3 && !burst_active){
        unsigned requested=1;
        if(atomic_compare_exchange_strong(&auto_cooldown_probe,&requested,2u)){
            auto_cooldown_start();return false; /* Same policy, no artificial RF failure. */
        }
    }
    bool recovering=atomic_load(&auto_cooldown)!=0;
    if(recovering && (atomic_load(&auto_stable_frames)<50u ||
       (uint32_t)((uint32_t)(esp_timer_get_time()/1000)-atomic_load(&auto_cooldown_since))<250u))return false;
    if(auto_last_frame!=trial.active_frame){
        auto_last_frame=trial.active_frame;
        if(atomic_load(&retry_auto_values[6])<1000u)atomic_fetch_add(&retry_auto_values[6],1);
    }
    if(atomic_load(&retry_auto_values[6])<1000u || trial.active_frame<1000u)return false;
    if(owner==0 && !atomic_compare_exchange_strong(&retry_owner,&owner,3u))return false;
    if(recovering){atomic_store(&auto_cooldown,0);atomic_fetch_add(&auto_recoveries,1);}
    atomic_store(&retry_auto_values[0],1);atomic_fetch_add(&retry_auto_values[2],1);
    burst_active=true;ull_radio_tx_retry_trace_enable(false);return true;
}

static uint32_t retry_cache_work[16]; /* Controller-private, no payload/key. */
static atomic_uint retry_cache_failure[16];
void ull_tone_trial_retry_cache_failure(uint32_t out[16]){
    if(!out)return;
    out[0]=atomic_load_explicit(&retry_cache_failure[0],memory_order_acquire);
    for(unsigned i=1;i<16;i++)out[i]=atomic_load_explicit(&retry_cache_failure[i],memory_order_relaxed);
}
static void retry_cache_publish(unsigned reason,uint32_t frame){
    if(atomic_load_explicit(&retry_cache_failure[0],memory_order_relaxed))return;
    retry_cache_work[1]=reason;retry_cache_work[2]=frame;
    for(unsigned i=1;i<16;i++)atomic_store_explicit(&retry_cache_failure[i],retry_cache_work[i],memory_order_relaxed);
    atomic_store_explicit(&retry_cache_failure[0],1,memory_order_release);
}

/* state0unused/1armed/2running/3ended; reason1budget/2timeout/3cancel/
 * 4fault-or-session-end/5scheduling-skip/6submit-failure. */
bool ull_tone_trial_retry_burst_arm(void){
    if(atomic_load(&stream_state[0])!=TRANSMITTING)return false;
    unsigned owner=0;
    if(!atomic_compare_exchange_strong(&retry_owner,&owner,2u))return false;
    unsigned expected=0;
    atomic_store(&retry_burst_started_ms,(uint32_t)(esp_timer_get_time()/1000));
    if(!atomic_compare_exchange_strong(&retry_burst[0],&expected,1u)){
        atomic_store(&retry_owner,0);return false;
    }
    if(atomic_load(&stream_state[0])!=TRANSMITTING){
        atomic_store(&retry_burst[1],4);atomic_store(&retry_burst[0],3);
        atomic_store(&retry_owner,0);return false;
    }
    return true;
}
void ull_tone_trial_retry_burst_status(uint32_t out[6]){
    for(unsigned i=0;i<6;i++)out[i]=atomic_load(&retry_burst[i]);
}
static void burst_end(unsigned reason){
    unsigned state=atomic_load(&retry_burst[0]);
    if(state==1 || state==2){
        atomic_store(&retry_burst[1],reason);atomic_store(&retry_burst[0],3);
    }
    if(!burst_active && !burst_watch && atomic_load(&retry_owner)==2)atomic_store(&retry_owner,0);
}
static void burst_tick(void){
    if(atomic_load(&retry_owner)==3 && !atomic_load(&retry_auto_enabled) && !burst_active && !burst_watch)auto_release();
    unsigned state=atomic_load(&retry_burst[0]);
    if((state==1 || state==2) && (uint32_t)((uint32_t)(esp_timer_get_time()/1000)-atomic_load(&retry_burst_started_ms))>=125000u)burst_end(2);
}
static bool burst_take(bool eligible){
    burst_tick();unsigned state=atomic_load(&retry_burst[0]);
    if(!eligible || (state!=1 && state!=2))return false;
    if(atomic_load(&retry_burst[2])>=24000u){burst_end(1);return false;}
    atomic_store(&retry_burst[0],2);atomic_fetch_add(&retry_burst[2],1);
    burst_active=true;ull_radio_tx_retry_trace_enable(false);return true;
}
static void burst_done(const struct ull_radio_tx_result *result,unsigned error){
    if(!burst_active)return;
    burst_active=false;ull_radio_tx_retry_trace_enable(true);
    if(atomic_load(&retry_owner)==3){
        if(error || !result || result->phase!=ULL_TX_ENDED){
            if(error){
                retry_cache_publish(6,trial.active_frame);auto_failed_skip_pending=true;
                if(error==ULL_TX_SCHEDULE_CONFLICT)atomic_fetch_add_explicit(&live_retry_errors[0],1,memory_order_relaxed);
                else if(error==ULL_TX_TIME_EXPIRED)atomic_fetch_add_explicit(&live_retry_errors[1],1,memory_order_relaxed);
            }
            atomic_fetch_add(&retry_auto_values[4],1);
            if(error==ULL_TX_TIME_EXPIRED || error==ULL_TX_SCHEDULE_CONFLICT)auto_cooldown_start();
            else auto_block();
            return;
        }
        atomic_fetch_add(&retry_auto_values[3],1);burst_watch=true;burst_lead=true;return;
    }
    if(error){retry_cache_publish(6,trial.active_frame);atomic_fetch_add(&retry_burst[4],1);burst_end(6);return;}
    if(!result || result->phase!=ULL_TX_ENDED){burst_end(4);return;}
    atomic_fetch_add(&retry_burst[3],1);burst_watch=true;burst_lead=true;
    if(atomic_load(&retry_burst[2])>=24000u)burst_end(1);
}
static void burst_skip(void){
    if(atomic_load(&auto_cooldown))atomic_store(&auto_stable_frames,0);
    if(auto_failed_skip_pending){atomic_fetch_add(&retry_auto_values[5],1);auto_failed_skip_pending=false;}
    if(!burst_watch){
        if(!atomic_load(&retry_auto_values[0]) && !atomic_load(&auto_cooldown)){atomic_store(&retry_auto_values[6],0);auto_last_frame=UINT32_MAX;}
        return;
    }
    if(atomic_load(&retry_owner)==3){
        retry_cache_publish(5,trial.next_frame);atomic_fetch_add(&retry_auto_values[5],1);auto_cooldown_start();return;
    }
    retry_cache_publish(5,trial.next_frame);
    atomic_fetch_add(&retry_burst[5],1);burst_watch=false;burst_lead=false;
    burst_end(5);
}
static void burst_submitted(void){
    burst_watch=false;
    if(atomic_load(&retry_owner)==3 && !burst_active && !atomic_load(&retry_auto_enabled))auto_release();
    if(atomic_load(&retry_burst[0])==3)burst_end(atomic_load(&retry_burst[1]));
}
static atomic_uint retry_probe_state;
static atomic_uint retry_probe_values[16];
static bool retry_probe_active,retry_probe_next_pending;
static bool retry_probe_lead_ticket;
static atomic_uint retry_attempts[64];
static unsigned retry_attempt_row=UINT32_MAX;
static uint32_t retry_probe_start_hs,retry_probe_skip_start;
static uint16_t retry_probe_start_hus;
bool ull_tone_trial_retry_probe_arm(void){
    if(atomic_load(&stream_state[0])!=TRANSMITTING)return false;
    unsigned owner=0;
    if(!atomic_compare_exchange_strong(&retry_owner,&owner,1u))return false;
    unsigned expected=0;
    bool armed=atomic_compare_exchange_strong(&retry_probe_state,&expected,1u);
    if(armed && atomic_load(&stream_state[0])!=TRANSMITTING){
        atomic_store(&retry_probe_state,4);armed=false;
    }
    if(!armed)atomic_store(&retry_owner,0);
    return armed;
}
void ull_tone_trial_retry_probe_status(uint32_t out[17]){
    out[0]=atomic_load_explicit(&retry_probe_state,memory_order_acquire);
    for(unsigned i=0;i<16;i++)out[i+1]=atomic_load(&retry_probe_values[i]);
}
void ull_tone_trial_retry_attempts(uint32_t out[64]){
    for(unsigned i=0;i<64;i++)out[i]=atomic_load(&retry_attempts[i]);
}
static bool retry_probe_take(bool eligible,uint32_t frame){
    if(!eligible)return false;
    unsigned expected=1;
    if(!atomic_compare_exchange_strong(&retry_probe_state,&expected,2u))return false;
    atomic_store(&retry_probe_values[0],frame);
    retry_probe_start_hs=trial.requested_hs;retry_probe_start_hus=trial.requested_hus;
    retry_probe_skip_start=trial.skipped;retry_probe_active=true;
    return true;
}
static void retry_probe_done(const struct ull_radio_tx_result *r,unsigned submit_error){
    if(!retry_probe_active)return;
    atomic_store(&retry_probe_values[1],submit_error);
    if(r){
        atomic_store(&retry_probe_values[2],r->phase);
        atomic_store(&retry_probe_values[3],r->followup_status);
        atomic_store(&retry_probe_values[4],r->tx_callbacks);
        atomic_store(&retry_probe_values[5],r->rx_callbacks);
        atomic_store(&retry_probe_values[6],r->rx_snapshots);
        atomic_store(&retry_probe_values[7],r->fault_detail);
    }
    retry_probe_next_pending=!submit_error && r && r->phase==ULL_TX_ENDED;
    retry_probe_lead_ticket=retry_probe_next_pending;
    retry_probe_active=false;
    if(!retry_probe_next_pending)atomic_store(&retry_owner,0);
    atomic_store_explicit(&retry_probe_state,
        submit_error || (r && r->phase==ULL_TX_FAULT)?4u:3u,memory_order_release);
}
static uint32_t retry_prepare_minimum(uint32_t minimum,bool *one_attempt){
    bool use=*one_attempt;*one_attempt=false;
    return minimum+(use?0u:1u);
}
static uint32_t retry_elapsed(void){
    radio_time_t now=r_rwip_time_get();
    uint32_t coarse=(now.hs-retry_probe_start_hs)&MASK;
    int64_t elapsed=(int64_t)coarse*625+now.hus-retry_probe_start_hus;
    return elapsed>=0 && elapsed<200000?(unsigned)(elapsed/2):UINT32_MAX;
}
static void retry_attempt_begin(uint32_t frame,uint32_t ahead,uint32_t minimum){
    retry_attempt_row=UINT32_MAX;
    if(!retry_probe_next_pending)return;
    unsigned count=atomic_load(&retry_attempts[0]);
    if(count>=7)return;
    retry_attempt_row=1+count*8;
    atomic_store(&retry_attempts[retry_attempt_row],frame);
    atomic_store(&retry_attempts[retry_attempt_row+1],retry_elapsed());
    atomic_store(&retry_attempts[retry_attempt_row+2],ahead);
    atomic_store(&retry_attempts[retry_attempt_row+3],minimum);
    atomic_store(&retry_attempts[retry_attempt_row+7],0x100);
    atomic_store(&retry_attempts[0],count+1);
}
static void retry_attempt_mark(unsigned slot,uint32_t value){
    if(retry_probe_next_pending && retry_attempt_row!=UINT32_MAX)
        atomic_store(&retry_attempts[retry_attempt_row+slot],value);
}
static void retry_attempt_time(unsigned slot){
    if(retry_probe_next_pending)retry_attempt_mark(slot,retry_elapsed());
}
static void retry_next_mark(unsigned slot){
    if(!retry_probe_next_pending)return;
    radio_time_t now=r_rwip_time_get();
    uint32_t coarse=(now.hs-retry_probe_start_hs)&MASK;
    int64_t elapsed=(int64_t)coarse*625+now.hus-retry_probe_start_hus;
    atomic_store(&retry_probe_values[slot],elapsed>=0 && elapsed<200000?(unsigned)(elapsed/2):UINT32_MAX);
}
static atomic_uint live_position,live_feedback,live_collect_us,live_build_us;
static atomic_uint live_delivery[8];
static atomic_uint live_repeat_delivery[6];
static atomic_uint live_retry_fallback[3]; /* attempted, sent, rejected */
void ull_tone_trial_retry_fallback(uint32_t out[3]){
    for(unsigned i=0;i<3;i++)out[i]=atomic_load_explicit(&live_retry_fallback[i],memory_order_relaxed);
}
void ull_tone_trial_retry_delivery(uint32_t out[6]){
    for(unsigned i=0;i<6;i++)out[i]=atomic_load_explicit(
        &live_repeat_delivery[i],memory_order_relaxed);
}
static uint8_t submit_with_retry_fallback(const struct ull_radio_tx_request *request,
                                          bool automatic,bool *fallback_success,
                                          bool *auto_failure_recorded){
    uint8_t status=ull_radio_tx_submit(request);
    *fallback_success=false;*auto_failure_recorded=false;
    if(!automatic || status!=ULL_TX_SCHEDULE_CONFLICT)return status;
    /* A long second-slot reservation may collide with native radio work.
     * Preserve the frame by trying the already-proven single-slot shape
     * at the same instant. Keep the automatic retry on cooldown either way. */
    burst_done(NULL,status);
    *auto_failure_recorded=true;
    atomic_fetch_add_explicit(&live_retry_fallback[0],1,memory_order_relaxed);
    struct ull_radio_tx_request ordinary=*request;
    ordinary.repeat_payload=0;
    ordinary.prequeued_reply=0;
    ordinary.followup_delay_us=2350;
    uint8_t single_status=ull_radio_tx_submit(&ordinary);
    if(!single_status){
        *fallback_success=true;
        auto_failed_skip_pending=false;
        atomic_fetch_add_explicit(&live_retry_fallback[1],1,memory_order_relaxed);
        return 0;
    }
    atomic_fetch_add_explicit(&live_retry_fallback[2],1,memory_order_relaxed);
    return status; /* Preserve the original recoverable error class. */
}
static atomic_uint live_channels[37][3];
void ull_tone_trial_channel_stats(uint32_t out[37][3]){
    for(unsigned ch=0;ch<37;ch++)for(unsigned i=0;i<3;i++)
        out[ch][i]=atomic_load_explicit(&live_channels[ch][i],memory_order_relaxed);
}
void ull_tone_trial_delivery(uint32_t out[8]){
    for(unsigned i=0;i<8;i++)out[i]=atomic_load(&live_delivery[i]);
}
void ull_tone_trial_progress(uint32_t out[4]){
    out[0]=atomic_load(&live_position);out[1]=atomic_load(&live_feedback);
    out[2]=atomic_load(&live_collect_us);out[3]=atomic_load(&live_build_us);
}
#ifndef ULL_USB_STREAM_FRAME_COUNT
#define ULL_USB_STREAM_FRAME_COUNT 60000u
#endif
_Static_assert(ULL_USB_STREAM_FRAME_COUNT<=60000u,"Bounded stream counters");
static uint32_t frame_limit(void){return audio_source?(continuous?UINT32_MAX-1u:ULL_USB_STREAM_FRAME_COUNT):ULL_TONE_FRAME_COUNT;}

bool ull_tone_trial_set_audio_source(struct ull_audio_source *source){
    if(trial.phase!=IDLE)return false;
    audio_source=source;return true;
}

static void put16(uint8_t *p,uint32_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void radio_report(uint8_t stage,uint8_t status,const struct ull_radio_tx_result *result){
#if ULL_LIVE_USB
    (void)status;
    if(stage==2 && result && result->phase==ULL_TX_FAULT){
        uint8_t failure[12]={'U','L','L','!',1,result->fault_detail,
            result->callback_reason,result->abort_checks,result->followup_status,
            result->rx_snapshots,0,0};
        put16(failure+10,trial.active_frame);
        ull_controller_diag_record(failure,sizeof(failure));
    }
    if(stage==2 && result){
        radio_time_t now=r_rwip_time_get();
        uint32_t coarse=(now.hs-trial.requested_hs)&MASK;
        int64_t elapsed=(int64_t)coarse*625+now.hus-trial.requested_hus;
        if(elapsed>=0 && elapsed<200000)atomic_store(&live_collect_us,(unsigned)(elapsed/2));
        atomic_store(&live_build_us,trial.build_us);
    }
    return;
#else
    radio_time_t now=r_rwip_time_get();
    uint8_t r[54]={'U','L','L','W',2,stage,status,255};
    memcpy(r+8,&trial.requested_hs,4);put16(r+12,trial.requested_hus);
    memcpy(r+14,&now.hs,4);put16(r+18,(uint16_t)now.hus);
    if(result){r[7]=result->callback_reason;memcpy(r+20,&result->end_hs,4);
        put16(r+24,result->tx_count);r[26]=result->deadline_fired;
        memcpy(r+34,&result->test_configured,4);memcpy(r+38,&result->test_programmed,4);
        memcpy(r+42,&result->test_at_callback,4);memcpy(r+46,result->supplied_prefix,8);}
    put16(r+28,trial.active_frame);memcpy(r+30,&trial.build_us,4);
    ull_controller_diag_record(r,sizeof(r));
    if(result){
        if(result->sequence_active){
            uint8_t sequence[23]={'U','L','L','I',3};
            put16(sequence+5,trial.active_frame);
            memcpy(sequence+7,&result->sequence_before,4);
            memcpy(sequence+11,&result->sequence_active,4);
            memcpy(sequence+15,&result->sequence_callback,4);
            memcpy(sequence+19,&result->sequence_restored,4);
            ull_controller_diag_record(sequence,sizeof(sequence));
        }
        if(result->rx_timing_before){
            uint8_t timing[23]={'U','L','L','I',2};
            put16(timing+5,trial.active_frame);
            memcpy(timing+7,&result->rx_timing_before,4);
            memcpy(timing+11,&result->rx_timing_active,4);
            memcpy(timing+15,&result->rx_timing_callback,4);
            memcpy(timing+19,&result->rx_timing_restored,4);
            ull_controller_diag_record(timing,sizeof(timing));
        }
        uint8_t summary[15]={'U','L','L','C',2};
        put16(summary+5,trial.active_frame);put16(summary+7,result->rx_callbacks);
        put16(summary+9,result->tx_callbacks);put16(summary+11,result->rx_freed);
        summary[13]=result->rx_snapshots;summary[14]=result->rx_invalid;
        ull_controller_diag_record(summary,sizeof(summary));
        uint8_t deadline[18]={'U','L','L','B',1};
        put16(deadline+5,trial.active_frame);
        memcpy(deadline+7,&result->deadline_hs,4);put16(deadline+11,result->deadline_hus);
        deadline[13]=result->abort_checks;memcpy(deadline+14,&result->abort_control,4);
        ull_controller_diag_record(deadline,sizeof(deadline));
        uint8_t paired[24]={'U','L','L','Y',1};
        put16(paired+5,trial.active_frame);paired[7]=result->followup_status;
        memcpy(paired+8,&result->followup_hs,4);put16(paired+12,result->followup_hus);
        memcpy(paired+14,&result->first_end_hs,4);put16(paired+18,result->first_end_hus);
        ull_controller_diag_record(paired,sizeof(paired));
        for(unsigned i=0;i<result->rx_snapshots && i<3;i++){
            const struct ull_radio_rx_snapshot *s=&result->rx[i];
            uint8_t packet[99]={'U','L','L','R',2};
            put16(packet+5,trial.active_frame);packet[7]=(uint8_t)i;
            memcpy(packet+8,&s->hs,4);put16(packet+12,s->hus);
            memcpy(packet+14,s->descriptor,20);packet[34]=s->copied;
            memcpy(packet+35,s->payload,s->copied);
            ull_controller_diag_record(packet,35u+s->copied);
        }
    }
#endif
}
static void report(void){
    const uint32_t values[8]={trial.phase,trial.reason,trial.next_frame,trial.submitted,trial.skipped,trial.received_valid,trial.received_rejected,continuous};
    for(unsigned i=0;i<8;i++)atomic_store(&stream_state[i],values[i]);
    atomic_store(&live_position,(trial.next_frame&65535u) | (trial.submitted<<16));
    atomic_store(&live_feedback,(trial.skipped&65535u) | (trial.received_valid<<16));
#if ULL_LIVE_USB
    if(trial.phase<FINISHED)return;
#endif
    uint8_t r[18]={'U','L','L','A',1,trial.phase,trial.reason,0};
    put16(r+8,trial.next_frame);put16(r+10,trial.submitted);
    put16(r+12,trial.skipped);put16(r+14,trial.reports);put16(r+16,trial.tx_count);
    ull_controller_diag_record(r,sizeof(r));
}
static void finish(uint8_t phase,uint8_t reason){
    unsigned requested_probe=1;atomic_compare_exchange_strong(&auto_cooldown_probe,&requested_probe,2u);
    atomic_store(&stream_state[0],phase); /* Close console arming before cleanup. */
    unsigned pending=1;
    atomic_compare_exchange_strong(&retry_probe_state,&pending,4u);
    if(retry_probe_active)retry_probe_done(NULL,0xff);
    retry_probe_next_pending=false;retry_probe_lead_ticket=false;
    burst_watch=false;burst_lead=false;
    if(burst_active)burst_done(NULL,0);
    burst_end(phase==CANCELLED?3:4);
    atomic_store(&auto_cooldown,0);atomic_store(&auto_stable_frames,0);
    atomic_store(&retry_auto_values[0],0);atomic_store(&retry_auto_values[1],1);
    atomic_store(&retry_owner,0);
    trial.phase=phase;trial.reason=reason;
    ull_ble_ccm_reset();
    mbedtls_platform_zeroize(&trial.plan,sizeof(trial.plan));
    mbedtls_platform_zeroize(trial.prepared,sizeof(trial.prepared));
    mbedtls_platform_zeroize(trial.alternate,sizeof(trial.alternate));
    mbedtls_platform_zeroize(trial.gap,sizeof(trial.gap));
    mbedtls_platform_zeroize(&trial.feedback,sizeof(trial.feedback));
    report();
}
static bool aa_valid(uint32_t aa){
    uint32_t difference=aa^UINT32_C(0x8e89bed6);
    if(!difference || !(difference&(difference-1)))return false;
    if((uint8_t)aa==(uint8_t)(aa>>8) && (uint8_t)aa==(uint8_t)(aa>>16) &&
       (uint8_t)aa==(uint8_t)(aa>>24))return false;
    unsigned run=1,transitions=0,top_transitions=0;
    for(unsigned i=1;i<32;i++){
        if(((aa>>i)^(aa>>(i-1)))&1u){run=1;transitions++;if(i>=27)top_transitions++;}
        else if(++run>6)return false;
    }
    return transitions<=24 && top_transitions>=2;
}
static uint32_t lead(uint32_t hs){return (hs-r_rwip_time_get().hs)&MASK;}
static bool receive_frame(uint32_t index){
    return ULL_RX_DIAGNOSTIC==1 || (ULL_RX_DIAGNOSTIC==2 && !(index&1u)) ||
           ((ULL_RX_DIAGNOSTIC>=3) && index<2);
}
static void receive_time_shift(uint32_t index,uint32_t *hs,uint16_t *hus){
    if(!receive_frame(index))return;
    uint32_t fine=(uint32_t)*hus+(ULL_RX_DIAGNOSTIC==1 ? 4440u : 1600u);
    *hs=(*hs+fine/625)&MASK;*hus=(uint16_t)(fine%625);
}

static bool build_prepared_frame(uint32_t index,uint8_t mask,
                                 const uint8_t seq[2],const uint8_t ack[2],
                                 const struct ull_parent_pdu *parent,struct prepared_frame *p){
    p->valid=false;
    bool poll=ULL_PARENT_POLL && ULL_EMPTY_BOOTSTRAP && mask==2 && !receive_frame(index);
    bool parent_ack=mask==6 && !receive_frame(index) && parent->header!=0;
    if(parent_ack && (parent->length || (parent->header & ~12u)!=1))return false;
    /* Stock downlink low bits select stream2; they are not uplink flags. */
    uint8_t header=(uint8_t)((mask<<3)|(poll?1:parent_ack?2:0));
    const struct ull_tone_packet_options options={
        /* Original-air-stream-01: 204 independently authenticated packets;
         * counter0, direction1, AAD = header & 0xe3. Fresh key/IV per trial. */
        .event_index=index,.packet_counter=0,.direction=1,.header=header,
        .aad=(uint8_t)(header&0xe3),.tx_sequence={seq[0],seq[1]},
        .ack_sequence={ack[0],ack[1]},.stream_mask=mask};
    struct ull_tone_packet packet;
    int64_t started=esp_timer_get_time();
    bool empty=ULL_EMPTY_BOOTSTRAP && mask==2;
    uint8_t audio[2][ULL_AIR_AUDIO_CHANNEL_BYTES];
    if(!empty && audio_source){
        int selected=ull_audio_source_select(audio_source,index,audio);
        if(selected<0)return false;
        empty=selected==0;
    }
    int built=parent_ack ? (empty ?
                        ull_empty_parent_ack_packet_build(&trial.plan,&options,parent->header,&packet) :
                        audio_source ? ull_audio_parent_ack_packet_build(&trial.plan,&options,audio,parent->header,&packet) :
                        ull_tone_parent_ack_packet_build(&trial.plan,&options,parent->header,&packet)) : poll ? (parent->length ?
                        ull_control_packet_build(&trial.plan,&options,parent->header,
                                                 parent->payload,parent->length,&packet) :
                        ull_poll_packet_build(&trial.plan,&options,parent->header,&packet)) :
              empty ? ull_empty_packet_build(&trial.plan,&options,&packet) :
              audio_source ? ull_audio_packet_build(&trial.plan,&options,audio,&packet) :
                      ull_tone_packet_build(&trial.plan,&options,&packet);
    if(built || packet.length!=(poll ? 13+parent->length : (empty ? (mask==6?15:11) : mask==6 ? 205 : 106)+(parent_ack?2:0)) || packet.pdu[0]!=header ||
       packet.pdu[1]!=packet.length-2)
        return false;
    /* Direct framing experiment: let hardware emit preamble, AA, whitening
     * and CRC. Independent C3 capture must verify whether format2 preserves
     * the requested proprietary header. No extra outer packet or prefix. */
    p->length=packet.length-2;memcpy(p->wire,packet.pdu+2,p->length);
    p->header=packet.pdu[0];p->outer_hs=packet.start_hs;p->outer_hus=packet.start_hus;
    p->inner_hs=packet.start_hs;p->inner_hus=packet.start_hus;
    if(poll){
        /* Measured control reply offset258us is slightly later than native
         * BLE turnaround for the11-byte ciphertext. Stock headset schedules
         * this response from its stream anchor. Shift only the poll's TX by
         *20us to test native RX alignment; keep E2/group timeline unchanged. */
        uint32_t fine=p->inner_hus+2u*ULL_POLL_TX_SHIFT_US;
        p->inner_hs=(p->inner_hs+fine/625u)&MASK;
        p->inner_hus=(uint16_t)(fine%625u);
        p->outer_hs=p->inner_hs;p->outer_hus=p->inner_hus;
    }
    p->channel=packet.channel;p->index=index;
    p->receive=receive_frame(index);
    if(p->receive){
        /* Mixed mode listens226us before the first reply. Returning earlier
         * leaves enough lead time to queue TX at the next5ms interval. */
        receive_time_shift(index,&p->inner_hs,&p->inner_hus);
        p->outer_hs=p->inner_hs;p->outer_hus=p->inner_hus;
        if(ull_ble_csa2_subevent(trial.plan.access_address,
              (uint16_t)(trial.plan.event_counter+index),trial.plan.e2+19,
              ULL_RX_DIAGNOSTIC==1 ? 2 : 1,&p->channel))return false;
    }
    p->generation=trial.feedback.generation;p->stream_mask=mask;p->poll_header=parent->header;
    p->parent=(poll || parent_ack)?*parent:(struct ull_parent_pdu){0};
    p->stereo_audio=mask==6 && !empty;
    memcpy(p->tx_sequence,seq,2);memcpy(p->ack_sequence,ack,2);
    p->build_us=(uint32_t)(esp_timer_get_time()-started);
    p->cache_reused=false;p->valid=true;
    return true;
}

static bool frame_matches(const struct prepared_frame *p,uint32_t index,
                           uint8_t mask,const uint8_t seq[2],const uint8_t ack[2],
                           const struct ull_parent_pdu *parent){
    return parent->length<=sizeof(parent->payload) && p->valid && p->index==index && p->stream_mask==mask &&
        p->parent.header==parent->header && p->parent.length==parent->length &&
        !memcmp(p->parent.payload,parent->payload,parent->length) &&
        !memcmp(p->tx_sequence,seq,2) && !memcmp(p->ack_sequence,ack,2);
}
static uint32_t cache_vector(const uint8_t seq[2],const uint8_t ack[2]){
    return seq[0]|((uint32_t)seq[1]<<8)|((uint32_t)ack[0]<<16)|((uint32_t)ack[1]<<24);
}
static bool cache_parent_matches(const struct prepared_frame *p,const struct ull_parent_pdu *parent){
    return parent->length<=sizeof(parent->payload) && p->parent.header==parent->header &&
        p->parent.length==parent->length && !memcmp(p->parent.payload,parent->payload,parent->length);
}
static bool prepare_frame(uint32_t index){
    if(index>=frame_limit())return true;
    struct prepared_frame *p=&trial.prepared[index&1u];
    uint8_t mask=trial.second_confirmed && index>=trial.stream_starts[1] &&
        trial.feedback.channels[1].valid ? 6 : 2;
    uint8_t seq[2],ack[2];struct ull_parent_pdu parent={0};
    if(ULL_PARENT_POLL && ULL_EMPTY_BOOTSTRAP && mask==2 && !receive_frame(index) &&
       ull_raw_parent_air_pdu(&parent))return false;
    if(mask==6 && !receive_frame(index) && control_poll_due(index))
        (void)ull_raw_detached_parent_air_pdu(&parent);
    ull_air_feedback_sequences(&trial.feedback,index,trial.stream_starts,seq,ack);
    if(ULL_SEQUENCE_CONTROL && !receive_frame(index) && (index&2u))
        seq[0]=(uint8_t)((seq[0]+8u)&15u);
    /* Selection always uses actual authenticated feedback. The alternative
     * only avoids doing CCM after RX when that feedback matches a prediction. */
    const struct prepared_frame *a=&trial.alternate[index&1u];
    const struct prepared_frame *g=&trial.gap[index&1u];
    unsigned state=atomic_load_explicit(&retry_burst[0],memory_order_relaxed);
    bool capture=state==1 || state==2 || atomic_load(&retry_owner)==3;
    if(capture){
        retry_cache_work[3]=index;retry_cache_work[4]=cache_vector(seq,ack);
        retry_cache_work[5]=cache_vector(p->tx_sequence,p->ack_sequence);
        retry_cache_work[6]=cache_vector(a->tx_sequence,a->ack_sequence);
        retry_cache_work[7]=p->index;retry_cache_work[8]=a->index;
        retry_cache_work[9]=(p->valid?1u:0u)|(a->valid?2u:0u)|
            (p->index==index?4u:0u)|(a->index==index?8u:0u)|
            (cache_parent_matches(p,&parent)?16u:0u)|(cache_parent_matches(a,&parent)?32u:0u);
        retry_cache_work[10]=0;retry_cache_work[11]=0;retry_cache_work[12]=0;
        retry_cache_work[13]=mask;retry_cache_work[14]=p->stream_mask;retry_cache_work[15]=a->stream_mask;
    }
    if(frame_matches(p,index,mask,seq,ack,&parent)){
        if(capture){retry_cache_work[10]=1;retry_cache_work[12]=1;}
        p->generation=trial.feedback.generation;p->cache_reused=true;return true;
    }
    if(frame_matches(a,index,mask,seq,ack,&parent)){
        if(capture){retry_cache_work[10]=2;retry_cache_work[12]=1;}
        *p=*a;p->generation=trial.feedback.generation;p->cache_reused=true;return true;
    }
    if(frame_matches(g,index,mask,seq,ack,&parent)){
        if(capture){retry_cache_work[10]=4;retry_cache_work[12]=1;}
        *p=*g;p->generation=trial.feedback.generation;p->cache_reused=true;return true;
    }
    bool built=build_prepared_frame(index,mask,seq,ack,&parent,p);
    if(capture){retry_cache_work[10]=3;retry_cache_work[11]=built?p->build_us:0;retry_cache_work[12]=built;}
    return built;
}
static bool prepare_alternate(uint32_t index){
    if(index>=frame_limit())return true;
    const struct prepared_frame *p=&trial.prepared[index&1u];
    struct prepared_frame *a=&trial.alternate[index&1u];a->valid=false;
    struct prepared_frame *g=&trial.gap[index&1u];g->valid=false;
    if(!p->valid || p->index!=index || p->receive ||
       (p->stream_mask!=2 && p->stream_mask!=6))return true;
    /* First-stream captures commonly advance the remote sequence once per
     * interval. Prepare that ACK as well while the radio owns the current
     * event. A different actual reply falls back to the ordinary builder. */
    uint8_t ack[2]={p->ack_sequence[0],p->ack_sequence[1]};
    /* Stereo speculation changes only the microphone ACK. Selection still
     * requires frame_matches against actual authenticated feedback. */
    ack[0]=p->stream_mask==6 ? (uint8_t)((ack[0]+1u)&15u) :
                              (uint8_t)((index?index-1u:0u)&15u);
    if(ack[0]==p->ack_sequence[0])return true;
    if(!build_prepared_frame(index,p->stream_mask,p->tx_sequence,ack,&p->parent,a))return false;
    /* Last authenticated mic frame can lag behind the reply expected before
     * this prepared frame. Predict that elapsed ACK gap only as a cache entry;
     * prepare_frame still selects solely from actual authenticated feedback. */
    const struct ull_air_feedback_channel *mic=&trial.feedback.channels[0];
    if(p->stream_mask==6 && index && mic->valid &&
       mic->frame>=trial.stream_starts[0] && mic->frame<index-1u){
        uint32_t advance=index-1u-mic->frame;
        uint8_t gap_ack=(uint8_t)((p->ack_sequence[0]+advance)&15u);
        if(advance>1u && gap_ack!=p->ack_sequence[0] && gap_ack!=ack[0]){
            ack[0]=gap_ack;
            return build_prepared_frame(index,6,p->tx_sequence,ack,&p->parent,g);
        }
    }
    return true;
}

uint8_t ull_tone_trial_start(void){
    if(trial.phase!=IDLE)return 0x0c; /* A collected terminal trial must be explicitly rearmed. */
    ull_ble_ccm_reset();
    trial.stream_starts[1]=UINT32_MAX;
    struct ull_air_session_seed seed={0};
    bool valid=false;
    for(unsigned i=0;i<128;i++){
        seed.access_address=esp_random();
        if(aa_valid(seed.access_address)){valid=true;break;}
    }
    if(!valid)return 0x0c;
    /* Original-live-session-04 group+54/+58 =1026/1328us, corresponding
     * to the enabled receiver row. Preserve its bidirectional reply slot. */
    seed.receive_enabled=1;
    /* Working original role1 inherits its parent ACL map. */
    uint8_t status=ull_raw_session_channel_map(seed.channel_map);
    if(status)return status;
    uint8_t map_report[10]={'U','L','L','M',1};
    memcpy(map_report+5,seed.channel_map,5);
    ull_controller_diag_record(map_report,sizeof(map_report));
    /* Fresh independent group material. IV API accepts the final transmitted
     * value. Random IV XOR AA remains uniformly random, as in stock creation. */
    esp_fill_random(seed.iv,sizeof(seed.iv));esp_fill_random(seed.key,sizeof(seed.key));
    for(unsigned i=0;i<4;i++)seed.iv[i]^=(uint8_t)(seed.access_address>>(8*i));
    status=ull_raw_prepare_session(&seed,2500,&trial.plan);
    mbedtls_platform_zeroize(&seed,sizeof(seed));
    if(status)return status; /* E1 not ready/instant stale; no E2 or RF sent. */
    status=ull_raw_session_crc_init(&trial.crc_init);
    if(status){mbedtls_platform_zeroize(&trial.plan,sizeof(trial.plan));return status;}
    const uint8_t framing[8]={'U','L','L','F',1,(uint8_t)trial.crc_init,
        (uint8_t)(trial.crc_init>>8),(uint8_t)(trial.crc_init>>16)};
    ull_controller_diag_record(framing,sizeof(framing)); /* Non-secret capture CRC. */
    uint8_t native[12]={'U','L','L','V',1};
    memcpy(native+5,&trial.plan.access_address,4);
    memcpy(native+9,framing+5,3);
    ull_controller_diag_record(native,sizeof(native)); /* Public AA/CRC only. */
    uint8_t shift[7]={'U','L','L','H',1,0,0};
    put16(shift+5,ULL_PARENT_POLL?ULL_POLL_TX_SHIFT_US:0);
    ull_controller_diag_record(shift,sizeof(shift));
    /* Prove the selected candidate can be built before committing its session. */
    if(!prepare_frame(0)){
        ull_ble_ccm_reset();
        mbedtls_platform_zeroize(&trial.plan,sizeof(trial.plan));return 0x0c;
    }
    status=ull_raw_commit_session(&trial.plan);
    if(status){ull_ble_ccm_reset();mbedtls_platform_zeroize(&trial.plan,sizeof(trial.plan));
        mbedtls_platform_zeroize(trial.prepared,sizeof(trial.prepared));return status;}
    /* Explicit research trials only: archive this trial's ephemeral material
     * over the local USB diagnostic link so independent C3 replies can be
     * authenticated offline. Never exports the saved bond or set key. The
     * host capture is private (0600); do not print this event to the console. */
    uint8_t capture_material[29]={'U','L','L','K',1};
    memcpy(capture_material+5,trial.plan.e2+44,24);
    if(!continuous)ull_controller_diag_record(capture_material,sizeof(capture_material));
    mbedtls_platform_zeroize(capture_material,sizeof(capture_material));
    trial.setup_started_us=esp_timer_get_time();
    trial.cancel=false;trial.phase=WAIT_E2;report();return 0;
}

/* Controller task only. Stream1 keeps its immutable timeline and keys while
 * stream2 negotiates a later point on that same Air grid. A failure while an
 * event is owned must cancel it and collect its terminal callback first. */
static uint8_t advance_second_stream(void){
    if(trial.second_committed){
        if(ull_raw_session_confirmed())trial.second_confirmed=true;
        return 0;
    }
    if(esp_timer_get_time()-trial.setup_started_us>2000000)return 7;
    if(!trial.second_requested){
        /* First E2 has been acknowledged. Negotiate the second stream before
         * the first Air activation; live parent captures showed E0 remained
         * unacknowledged when this was delayed until Air was already active. */
        uint8_t status=ull_raw_request_second_stream(&trial.plan);
        if(status==0x0c)return 0;
        if(status)return 8;
        trial.second_requested=true;
        const uint8_t joined[6]={'U','L','L','J',1,1};
        ull_controller_diag_record(joined,sizeof(joined));
        return 0;
    }
    if(!ull_raw_second_stream_ready())return 0;
    struct ull_air_session_plan second={0};uint32_t crc=0;
    uint8_t status=ull_raw_prepare_second_stream(&trial.plan,&second);
    if(status || ull_raw_session_crc_init(&crc) || crc!=trial.crc_init){
        mbedtls_platform_zeroize(&second,sizeof(second));return 9;
    }
    status=ull_raw_commit_session(&second);
    if(!status){
        trial.second_committed=true;
        trial.stream_starts[1]=second.event_counter-trial.plan.event_counter;
        uint8_t joined[12]={'U','L','L','J',1,2};
        memcpy(joined+6,&second.event_counter,4);put16(joined+10,second.acl_instant);
        ull_controller_diag_record(joined,sizeof(joined));
    }
    mbedtls_platform_zeroize(&second,sizeof(second));
    return status?11:0;
}

uint8_t IRAM_ATTR ull_tone_trial_step(void){
#if !ULL_LIVE_USB
    int64_t profile_start=esp_timer_get_time();
    uint32_t profile[8]={0};
#define TRACE_PROFILE(i) (profile[i]=(uint32_t)(esp_timer_get_time()-profile_start))
#else
#define TRACE_PROFILE(i) ((void)0)
#endif
    burst_tick();
    uint32_t minimum=ull_radio_tx_min_lead();
    if(trial.phase==IDLE){report();return 0x0c;}
    if(trial.phase>=FINISHED){report();return trial.phase==FAULT?0x03:0;}
    if(trial.phase==WAIT_E2){
        if(trial.cancel){finish(CANCELLED,0);return 0;}
        if(!trial.first_confirmed && !ull_raw_session_confirmed()){
            uint32_t ahead=lead(trial.plan.start_hs);
            if(ahead<minimum+1 || ahead>3200)finish(FAILED,1);
            return 0;
        }
        trial.first_confirmed=true;
    }else{
        struct ull_radio_tx_result result;
        int collected=ull_radio_tx_collect(&result);
        if(collected<0){burst_done(&result,0);retry_probe_done(&result,0);radio_report(2,(uint8_t)result.phase,&result);finish(FAULT,2);return 0x03;}
        if(!collected){
            if(!trial.cancel){
                uint8_t error=advance_second_stream();
                if(error){trial.preparation_failure=error;trial.cancel=true;
                    ull_radio_tx_cancel();}
            }
            return 0;
        }
        burst_done(&result,0);
        retry_probe_done(&result,0);
        retry_next_mark(11);
        TRACE_PROFILE(0);
        radio_report(2,(uint8_t)result.phase,&result);
        TRACE_PROFILE(1);
        trial.reports++;
        trial.tx_count=(uint16_t)(trial.tx_count+result.tx_count);
        if(result.phase!=ULL_TX_ENDED && result.phase!=ULL_TX_CANCELLED){
            finish(FAILED,3);return 0;
        }
        if(trial.cancel){finish(trial.preparation_failure?FAILED:CANCELLED,
                               trial.preparation_failure);return 0;}
        if(result.rx_invalid){finish(FAILED,13);return 0;}
        const struct prepared_frame *completed=&trial.prepared[trial.active_frame&1u];
        bool audio_sent=completed->valid && completed->index==trial.active_frame &&
            completed->stereo_audio &&
            result.phase==ULL_TX_ENDED;
        auto_ordinary_completed(audio_sent);
        unsigned acknowledged=0,first_window_ack=0;
        bool heard_reply=false,first_window_valid=false,second_window_valid=false;
        atomic_fetch_add(&live_delivery[0],1);
        /* CS test-TX packet count remains zero with our normal packet format.
         * Count TX callbacks separately; only authenticated ACKs establish
         * receipt of the completed audio event by the headset. */
        atomic_fetch_add(&live_delivery[1],result.tx_callbacks);
        if(!result.tx_callbacks)atomic_fetch_add(&live_delivery[2],1);
        if(audio_sent)atomic_fetch_add(&live_delivery[3],1);
        if(!result.rx_snapshots)atomic_fetch_add(&live_delivery[7],1);
        for(unsigned i=0;i<result.rx_snapshots && i<3;i++){
            if(result.rx[i].copied<7)continue; /* Empty timeout descriptors. */
            struct ull_air_control_rx control={0};
            if(!ull_air_feedback_accept_frame(&trial.feedback,&trial.plan,&result.rx[i],trial.active_frame,&control)){
                heard_reply=true;
                trial.received_valid++;
                const struct prepared_frame *sent=&trial.prepared[trial.active_frame&1u];
                const struct ull_radio_rx_snapshot *rx=&result.rx[i];
                if(control.present && !sent->receive && sent->index==trial.active_frame &&
                   result.phase==ULL_TX_ENDED){
                    if(control.length){
                        unsigned now=(unsigned)(esp_timer_get_time()/1000);
                        atomic_store_explicit(&control_last_data_ms,now?now:1u,memory_order_relaxed);
                        atomic_fetch_add_explicit(&control_retry_policy[2],1,memory_order_relaxed);
                    }
                    (void)ull_raw_receive_air_control(&control,&sent->parent);
                }
                uint32_t coarse=(rx->hs-trial.requested_hs)&MASK;
                int64_t elapsed=(int64_t)coarse*625+rx->hus-trial.requested_hus;
                if(audio_sent && auto_last_submitted_repeat && elapsed>=0 && elapsed<6000){
                    if(elapsed<ULL_AIR_SUBINTERVAL_US)first_window_valid=true;
                    else second_window_valid=true;
                }
                /* Slot1 replies precede the observed slot2 expiry advance.
                 * An ACK of this transmission must not be advanced twice
                 * when deriving the next interval's expected sequence. */
                if(ULL_RX_DIAGNOSTIC>=3 &&
                   !sent->receive && sent->index==trial.active_frame &&
                   elapsed>=0 && elapsed<6000){
                    ull_air_feedback_note_tx(&trial.feedback,trial.active_frame,
                                             sent->stream_mask,sent->tx_sequence);
                    if(audio_sent)for(unsigned ch=0;ch<2;ch++){
                        const struct ull_air_feedback_channel *f=&trial.feedback.channels[ch];
                        if(f->valid && f->frame==trial.active_frame && f->after_tx &&
                           f->expected_tx==((sent->tx_sequence[ch]+1u)&15u))
                            acknowledged|=1u<<ch;
                    }
                    if(audio_sent && auto_last_submitted_repeat &&
                       elapsed<ULL_AIR_SUBINTERVAL_US)first_window_ack=acknowledged;
                }
            }else {trial.received_rejected++;atomic_fetch_add(&live_delivery[6],1);}
            mbedtls_platform_zeroize(&control,sizeof(control));
        }
        if(audio_sent && auto_last_submitted_repeat){
            atomic_fetch_add_explicit(&live_repeat_delivery[0],1,memory_order_relaxed);
            if(first_window_valid)atomic_fetch_add_explicit(&live_repeat_delivery[1],1,memory_order_relaxed);
            if(second_window_valid)atomic_fetch_add_explicit(&live_repeat_delivery[2],1,memory_order_relaxed);
            if(first_window_ack==3u)atomic_fetch_add_explicit(&live_repeat_delivery[3],1,memory_order_relaxed);
            else if(acknowledged==3u)atomic_fetch_add_explicit(&live_repeat_delivery[4],1,memory_order_relaxed);
            else atomic_fetch_add_explicit(&live_repeat_delivery[5],1,memory_order_relaxed);
        }
        if(acknowledged&1u)atomic_fetch_add(&live_delivery[4],1);
        if(acknowledged&2u)atomic_fetch_add(&live_delivery[5],1);
        if(audio_sent && completed->channel<37){
            atomic_fetch_add_explicit(&live_channels[completed->channel][0],1,memory_order_relaxed);
            if(heard_reply)atomic_fetch_add_explicit(&live_channels[completed->channel][1],1,memory_order_relaxed);
            if(acknowledged==3)atomic_fetch_add_explicit(&live_channels[completed->channel][2],1,memory_order_relaxed);
        }
        retry_next_mark(12);
        TRACE_PROFILE(2);
        /* Arbitration may cancel a future reservation when the ACL schedule
         * changes. Retry identical cached bytes only while that instant is
         * still usable; cancellation is not successful delivery of a frame. */
        if(result.phase==ULL_TX_CANCELLED && !result.tx_count){
            uint32_t ahead=lead(trial.requested_hs);
            if(ahead>=minimum+1 && ahead<=3200)trial.next_frame=trial.active_frame;
            else {trial.skipped++;burst_skip();}
        }
        report();
    }
    uint8_t join_error=advance_second_stream();
    TRACE_PROFILE(3);
    if(join_error){finish(FAILED,join_error);return 0;}
    uint32_t hs;uint16_t hus;
    bool retry_next_attempt=retry_probe_lead_ticket || burst_lead;
    burst_lead=false;
    retry_probe_lead_ticket=false;
    while(trial.next_frame<frame_limit()){
        /* The verified parent period is30ms and this Air grid begins2.5ms
         * after its anchor. Leave the preceding interval idle for this
         * coexistence test; actual ACK/E1 observations determine its effect. */
        if(ULL_CONTROL_GAP && ULL_RX_DIAGNOSTIC>=3 && trial.next_frame%6u==5u){
            trial.next_frame++;trial.skipped++;burst_skip();continue;
        }
        if(ull_air_session_time(&trial.plan,trial.next_frame,0,&hs,&hus)){
            finish(FAILED,4);return 0;
        }
        receive_time_shift(trial.next_frame,&hs,&hus);
        uint32_t ahead=lead(hs);
        /* One immediate post-probe attempt may use the controller's real
         * minimum. Both post-build and locked submit guards remain intact. */
        uint32_t preparation_min=retry_prepare_minimum(minimum,&retry_next_attempt);
        retry_attempt_begin(trial.next_frame,ahead,preparation_min);
        retry_next_mark(13);
        if(ahead>=preparation_min && ahead<=3200){
            TRACE_PROFILE(4);
            if(!prepare_frame(trial.next_frame)){finish(FAILED,5);return 0;}
            retry_next_mark(14);
            TRACE_PROFILE(5);
            const struct prepared_frame *p=&trial.prepared[trial.next_frame&1u];
            ahead=lead(p->outer_hs);
            retry_attempt_time(4);
            retry_attempt_mark(5,ahead);
            retry_attempt_mark(7,0x101);
            if(ahead>=minimum && ahead<=3200)break;
        }
        trial.next_frame++;trial.skipped++;burst_skip();
    }
    if(trial.next_frame==frame_limit()){
        bool joined=trial.second_committed && ull_raw_session_confirmed();
        finish(joined?FINISHED:FAILED,joined?0:12);return 0;
    }
    const struct prepared_frame *p=&trial.prepared[trial.next_frame&1u];
    trial.build_us=p->build_us;trial.active_frame=trial.next_frame;
    trial.requested_hs=p->inner_hs;trial.requested_hus=p->inner_hus;
    uint8_t reply_channel=0;
    bool eligible=retry_audio_eligible(p);
    if(p->stereo_audio && p->header==0x32 && p->length==205)
        atomic_fetch_add_explicit(&control_retry_policy[eligible?0:1],1,memory_order_relaxed);
    bool probe=retry_probe_take(eligible,trial.active_frame);
    bool burst=burst_take(eligible);
    bool automatic=!probe && !burst && auto_take(eligible);
    bool repeat=(ULL_AUDIO_REPEAT && eligible) || probe || burst || automatic;
    bool prequeued=repeat || (ULL_RX_DIAGNOSTIC==5 && !p->receive && p->stream_mask!=6);
    bool paired=(ULL_RX_DIAGNOSTIC==4 && !p->receive) || prequeued;
    if(prequeued && !repeat)reply_channel=p->channel;
    if(paired && (!prequeued || repeat) && ull_ble_csa2_subevent(trial.plan.access_address,
        (uint16_t)(trial.plan.event_counter+trial.next_frame),trial.plan.e2+19,
        2,&reply_channel)){finish(FAILED,4);return 0;}
    const struct ull_radio_tx_request request={
        .start_hs=p->outer_hs,.start_hus=p->outer_hus,
        .access_address=trial.plan.access_address,.crc_init=trial.crc_init,
        .payload=p->wire,.length=p->length,.channel=p->channel,.header=p->header,.phy=2,
        .receive_us=p->receive ? 600 : 0,
        .tx_receive_us=ULL_RX_DIAGNOSTIC==3 && !p->receive && !repeat ? 600 : 0,
        .followup_delay_us=repeat ? ULL_AIR_SUBINTERVAL_US : prequeued ? 950 : paired ? 2350 : 0,
        .followup_window_us=paired ? 150 : 0,
        .followup_channel=reply_channel,.prequeued_reply=(uint8_t)prequeued,
        .repeat_payload=(uint8_t)repeat};
#if !ULL_LIVE_USB
    uint8_t feedback[21]={'U','L','L','G',2};
    put16(feedback+5,trial.active_frame);put16(feedback+7,trial.received_valid);
    put16(feedback+9,trial.received_rejected);memcpy(feedback+11,&p->generation,4);
    feedback[15]=(uint8_t)p->receive;feedback[16]=p->stream_mask;
    feedback[17]=p->tx_sequence[0];feedback[18]=p->ack_sequence[0];
    feedback[19]=p->tx_sequence[1];feedback[20]=p->ack_sequence[1];
    ull_controller_diag_record(feedback,sizeof(feedback));
#endif
    TRACE_PROFILE(6);
    bool fallback_success=false,auto_failure_recorded=false;
    uint8_t status=submit_with_retry_fallback(&request,automatic,
                                                &fallback_success,&auto_failure_recorded);
    retry_attempt_time(6);
    retry_attempt_mark(7,status);
    retry_next_mark(15);
    if(status){if(!auto_failure_recorded)burst_done(NULL,status);retry_probe_done(NULL,status);}
    else {auto_last_submitted_repeat=repeat && !fallback_success;burst_submitted();}
    if(!status && retry_probe_next_pending && trial.active_frame>atomic_load(&retry_probe_values[0])){
        radio_time_t next_time=r_rwip_time_get();
        uint32_t coarse=(next_time.hs-retry_probe_start_hs)&MASK;
        int64_t elapsed=(int64_t)coarse*625+next_time.hus-retry_probe_start_hus;
        atomic_store(&retry_probe_values[8],trial.active_frame);
        atomic_store(&retry_probe_values[9],elapsed>=0 && elapsed<200000?(unsigned)(elapsed/2):UINT32_MAX);
        atomic_store(&retry_probe_values[10],trial.skipped-retry_probe_skip_start);
        retry_probe_next_pending=false;
        if(atomic_load(&retry_owner)==1)atomic_store(&retry_owner,0);
    }
    TRACE_PROFILE(7);
    radio_report(1,status,NULL);

#if !ULL_LIVE_USB
    if(ULL_PARENT_POLL && p->header==0x11){
        uint8_t poll[10]={'U','L','L','Q',1,0,0,p->poll_header,status,p->header};
        put16(poll+5,trial.next_frame);ull_controller_diag_record(poll,sizeof(poll));
        if(p->parent.length){
            uint8_t control[12]={'U','L','L','Q',2,0,0,p->parent.header,status,
                                p->header,p->parent.length,p->parent.payload[0]};
            put16(control+5,trial.next_frame);ull_controller_diag_record(control,sizeof(control));
        }
    }
    uint8_t timing[40]={'U','L','L','Z',4};
    put16(timing+5,trial.active_frame);timing[7]=status;
    memcpy(timing+8,profile,sizeof(profile));
    ull_controller_diag_record(timing,sizeof(timing));
#endif
    if(status==ULL_TX_SCHEDULE_CONFLICT || status==ULL_TX_TIME_EXPIRED){
        /* Arbitration reclaimed its storage, or the final locked deadline
         * check allocated none. Skip this instant; other guards remain fatal. */
        trial.next_frame++;trial.skipped++;burst_skip();trial.phase=WAIT_E2;
        report();return 0;
    }
    if(status){finish(FAILED,6);return 0;}
    trial.next_frame++;trial.submitted++;trial.phase=TRANSMITTING;
    /* Build the following encrypted, whitened packet while this reservation
     * is pending. Submission copies its bytes, so the other cache slot is free. */
    if(!prepare_frame(trial.next_frame) || !prepare_alternate(trial.next_frame)){
        trial.preparation_failure=5;trial.cancel=true;ull_radio_tx_cancel();
    }
    return 0;
}

uint8_t ull_tone_trial_cancel(void){
    burst_end(3);burst_lead=false;
    trial.cancel=true;
    if(trial.phase==TRANSMITTING)ull_radio_tx_cancel();
    return ull_tone_trial_step();
}

uint8_t ull_tone_trial_status(void){report();return trial.phase==FAULT?0x03:0;}
bool ull_tone_trial_active(void){return trial.phase==WAIT_E2 || trial.phase==TRANSMITTING;}

bool ull_tone_trial_rearm(void)
{
    if(trial.phase<FINISHED || trial.phase>=FAULT)return false;
    /* Repeat collection verifies ownership and releases any remaining storage
     * before the next HCI Reset can invalidate controller allocations. */
    struct ull_radio_tx_result result;
    if(ull_radio_tx_collect(&result)!=1)return false;
    memset(&trial,0,sizeof(trial));
    atomic_store(&retry_auto_values[0],0);atomic_store(&retry_auto_values[1],0);
    atomic_store(&retry_auto_values[6],0);auto_last_frame=UINT32_MAX;auto_failed_skip_pending=false;
    atomic_store(&auto_cooldown,0);atomic_store(&auto_stable_frames,0);auto_last_submitted_repeat=false;
    audio_source=NULL;continuous=false;
    for(unsigned i=0;i<8;i++)atomic_store(&stream_state[i],0);
    return true;
}
