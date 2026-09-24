/* Minimal scheduled physical TX path. All radio-global writes occur inside the
 * owned event, with an empty hardware programming queue. SDK test routines are
 * used only for the verified CS-local TX power operation, never test start/stop.
 * See radio-research/RADIO_TX_IMPLEMENTATION.md for exact ABI provenance.
 */
#include "radio_tx.h"
#include "phy_packet.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdatomic.h>
#include "esp_attr.h"
#include "sdkconfig.h"

/* Primary owned callback is the sole writer. Console reads only these RAM
 * snapshots, never CS or controller pointers. Counters are approximate across
 * concurrent reads; raw index is deliberately not interpreted as dBm. */
static DRAM_ATTR atomic_uint power_raw_index,power_event_count;
static DRAM_ATTR unsigned power_events;
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"RF power snapshot must be lock-free");
void ull_radio_tx_power_snapshot(uint32_t out[2]){
    if(!out)return;
    out[1]=atomic_load_explicit(&power_event_count,memory_order_acquire);
    out[0]=atomic_load_explicit(&power_raw_index,memory_order_relaxed);
}

#define CLOCK_MASK UINT32_C(0x0fffffff)
#define ACTIVITY 1u
#define HANDLE 0xeeu
#define TX_FORMAT 2u /* Stock connection initiator, ROM r_lld_con_start. */
#define CS_OFFSET (0x400u + 90u * ACTIVITY)
#define DESC_OFFSET (0x1400u + 126u * ACTIVITY)
#define REPEAT_DESC_OFFSET (0x1400u + 126u * 2u)
#define TEST_ENABLE (UINT32_C(1) << 14)
#define RADIO_ABORT (UINT32_C(1) << 26)
#define TX_TEST (UINT32_C(1) << 11)
#define PRBS_ENABLE (UINT32_C(1) << 12)
#define PRBS_SELECT (UINT32_C(1) << 13)
#define RX_TEST (UINT32_C(1) << 27)
#define TEST_BITS (TX_TEST | PRBS_ENABLE | PRBS_SELECT | RX_TEST)
#define RADIO_CONTROL (*(volatile uint32_t *)UINT32_C(0x60031000))
#define TEST_CONTROL (*(volatile uint32_t *)UINT32_C(0x600310d0))
#define RX_TIMING_2M (*(volatile uint32_t *)UINT32_C(0x60031094))
/* Bounded experiment, not a claimed time-in-us setting. Pinned ROM
 * r_lld_reset_reg writes0x20407 here; ULLI1 verified the same live value.
 * Related RW headers name bits22:16 RFRXTMDA1. Measure the actual effect
 * instead of assuming their units/sign apply to S3. Keeper is unaffected. */
#ifndef ULL_RX_TIMING_FIELD_ADD
#define ULL_RX_TIMING_FIELD_ADD 0u
#endif
_Static_assert(ULL_RX_TIMING_FIELD_ADD<=32u,"Bounded timing field probe");
/* The captured early control reply begins about170us after our88us poll,
 * outside the nominal150us turnaround. Test the related RW TXPATHDLY1 field
 * independently of RFRXTMDA1: its actual effect must be measured. The known
 * ROM/live baseline has low7 bits7; add24 without touching adjacent fields. */
#ifndef ULL_TX_PATH_FIELD_ADD
#define ULL_TX_PATH_FIELD_ADD 24u
#endif
_Static_assert(ULL_TX_PATH_FIELD_ADD<=32u,"Bounded TX path timing probe");
/* Air uses the BLE header's sequence bits as its stream bitmap. Related RW
 * headers map SN_DSB/NESN_DSB to19/18 in this controller's verified control
 * register. Test that inference only in our owned native Air events; leave
 * CRC/whitening and the normal encrypted parent connection unchanged. */
#define AIR_SEQUENCE_BYPASS_MASK UINT32_C(0x000c0000)
#ifndef ULL_AIR_SEQUENCE_BYPASS
#define ULL_AIR_SEQUENCE_BYPASS 0
#endif

typedef struct { uint32_t hs, hus; } radio_time_t;
typedef struct tx_arb tx_arb_t;
struct tx_arb {
    tx_arb_t *next;
    uint32_t hs, hus, limit_hs, duration_hus;
    uint16_t control;
    uint8_t priority, padding;
    void (*start)(tx_arb_t *);
    void (*stop)(tx_arb_t *);
    void (*cancel)(tx_arb_t *);
};
typedef struct tx_alarm tx_alarm_t;
struct tx_alarm {
    tx_alarm_t *next;
    uint32_t hs;
    void (*callback)(tx_alarm_t *);
};
typedef void (*frame_callback_t)(uint32_t, uint32_t, uint8_t);
struct tx_program {
    frame_callback_t callback;
    uint32_t hs, hus, duration_hus, context;
    uint8_t priority, priority1, priority2, mode, cs, alternate;
    uint8_t flag26, extension, extended, extension29, extension30, padding;
};
struct program_entry {
    uint32_t hs;
    frame_callback_t callback;
    uint32_t context;
    uint8_t used, padding[3];
};
_Static_assert(CONFIG_BT_CTRL_BLE_MAX_ACT == 6, "Pinned activity table");
_Static_assert(sizeof(tx_arb_t) == 36, "Arbitration ABI");
_Static_assert(offsetof(tx_arb_t, start) == 24, "Arbitration callback ABI");
_Static_assert(sizeof(tx_alarm_t) == 12, "Alarm ABI");
_Static_assert(sizeof(struct tx_program) == 32, "Program ABI");
_Static_assert(offsetof(struct tx_program, cs) == 24, "CS index ABI");
_Static_assert(sizeof(struct program_entry) == 16, "Program ring ABI");

extern void **r_ip_funcs_p, **r_osi_funcs_p, **r_modules_funcs_p;
extern uint8_t *p_llm_env, *lld_adv_env[], *lld_con_env[];
extern uint8_t *p_lld_env;
extern uint8_t sch_arb_env[], sch_prog_env[];
extern uint8_t rwip_prog_delay;
extern uint8_t sdk_cfg_priv_opts[];
extern radio_time_t r_rwip_time_get(void);
extern void *r_emi_get_mem_addr_by_offset(uint16_t);
extern uint8_t r_sch_arb_insert(tx_arb_t *);
extern uint8_t r_sch_arb_remove(tx_arb_t *, uint8_t);
extern void r_sch_prog_push(const struct tx_program *);
extern void r_sch_alarm_set(tx_alarm_t *);
extern uint8_t r_sch_alarm_clear(tx_alarm_t *);
extern void r_lld_rxdesc_free(void);
extern uint16_t r_ble_util_buf_get_rx_buf_size(void);

static DRAM_ATTR struct {
    tx_arb_t arb;
    tx_alarm_t alarm;
    tx_alarm_t first_alarm;
    struct tx_program program;
    struct tx_program reply_program, stop_program;
    struct ull_radio_tx_result result;
    uint8_t *activity_parameters;
    uint8_t *reply_parameters, *stop_parameters;
    volatile uint16_t *cs, *descriptor;
    volatile uint16_t *reply_cs, *stop_cs;
    volatile uint16_t *repeat_descriptor;
    uint16_t saved_repeat_descriptor[7];
    uint16_t saved_cs[45], saved_descriptor[7], payload_offset;
    uint16_t saved_reply_cs[45], saved_stop_cs[45];
    uint32_t saved_test_bits, saved_enable, saved_rx_timing;
    bool rx_timing_set;
    uint32_t saved_sequence_flags;
    bool sequence_flags_set;
    uint8_t program_index, format;
    uint8_t reply_index, stop_index;
    bool alarm_set, globals_set, storage_set;
    bool followup_pending, followup_active;
    uint16_t followup_window_us;
    uint8_t followup_channel, phy;
    bool prequeued_reply, first_ended, reply_ended, stop_ended, stop_abort_requested;
    bool repeat_payload;
    bool first_alarm_set;
} tx = { .program_index = 255 };

static void lock(void) { ((void (*)(void))r_osi_funcs_p[5])(); }
static void unlock(void) { ((void (*)(void))r_osi_funcs_p[6])(); }
uint32_t ull_radio_tx_min_lead(void)
{
    /* Use the live controller value, also captured by air_event diagnostics.
     * Never reduce the margin on an uninitialized/unexpected zero value. */
    return rwip_prog_delay ? (uint32_t)rwip_prog_delay + 2u : 8u;
}
static void IRAM_ATTR tx_release_storage(void)
{
    if (!tx.storage_set) return;
    for (unsigned i = 0; i < 45; i++) tx.cs[i] = tx.saved_cs[i];
    if(tx.prequeued_reply)
        for(unsigned i=0;i<45;i++)tx.reply_cs[i]=tx.saved_reply_cs[i];
    if(tx.repeat_payload)
        for(unsigned i=0;i<45;i++)tx.stop_cs[i]=tx.saved_stop_cs[i];
    for (unsigned i = 0; i < 7; i++) tx.descriptor[i] = tx.saved_descriptor[i];
    if(tx.repeat_payload)
        for(unsigned i=0;i<7;i++)tx.repeat_descriptor[i]=tx.saved_repeat_descriptor[i];
    ((void (*)(uint16_t))r_ip_funcs_p[51])(tx.payload_offset);
    tx.storage_set = false;
}
static bool IRAM_ATTR tx_valid_ram(const void *p, unsigned n)
{
    uintptr_t a = (uintptr_t)p;
    return a >= UINT32_C(0x3fc80000) && a <= UINT32_C(0x3fcf0000) - n;
}
static uint8_t *IRAM_ATTR tx_parameters_for(unsigned slot,uint8_t handle)
{
    if (!tx_valid_ram(p_llm_env, 12)) return NULL;
    uint8_t *activities = *(uint8_t **)(p_llm_env + 8);
    if (!tx_valid_ram(activities, 68 * 6)) return NULL;
    uint8_t *activity = activities + 68 * slot;
    uint8_t *parameters = *(uint8_t **)activity;
    if (activity[64] != 1 || !tx_valid_ram(parameters, 1) ||
        parameters[0] != handle || lld_adv_env[slot] || lld_con_env[slot])
        return NULL;
    return parameters;
}
static uint8_t *IRAM_ATTR tx_activity_parameters(void)
{
    return tx_parameters_for(ACTIVITY,HANDLE);
}
static bool IRAM_ATTR tx_active_is_ours(void)
{
    return *(tx_arb_t **)(sch_arb_env + 8) == &tx.arb;
}
static bool IRAM_ATTR tx_entry_is_ours(uint8_t index,const struct tx_program *p,
                                     bool require_consumer)
{
    if (index >= 16) return false;
    const volatile struct program_entry *e =
        (const volatile struct program_entry *)sch_prog_env + index;
    return (!require_consumer || sch_prog_env[0x100] == index) &&
        e->used && e->callback == p->callback &&
        e->context == p->context && e->hs == p->hs;
}
static bool IRAM_ATTR tx_program_is_ours(bool require_consumer)
{
    return tx_entry_is_ours(tx.program_index,&tx.program,require_consumer) ||
        (tx.prequeued_reply && tx_entry_is_ours(tx.reply_index,&tx.reply_program,require_consumer)) ||
        (tx.repeat_payload && tx_entry_is_ours(tx.stop_index,&tx.stop_program,require_consumer));
}
/* ROM4002d502..511 obtains EM base at offset0, adds index*16 and
 * reads ET word0. Only inspect our still-used, identity-matching entries. */
static DRAM_ATTR atomic_uint retry_trace[160];
static unsigned retry_records;
static DRAM_ATTR atomic_uint retry_trace_enabled=1;
void ull_radio_tx_retry_trace_enable(bool enabled){atomic_store_explicit(&retry_trace_enabled,enabled,memory_order_release);}
/* Controller-lock-owned ticket; publication contains scalar metadata only. */
static DRAM_ATTR bool retry_insert_pending, retry_insert_cancelled;
static DRAM_ATTR atomic_uint retry_insert_trace[8];
void ull_radio_tx_insert_snapshot(uint32_t out[8]){
    if(!out)return;
    out[0]=atomic_load_explicit(&retry_insert_trace[0],memory_order_acquire);
    for(unsigned i=1;i<8;i++)out[i]=atomic_load_explicit(&retry_insert_trace[i],memory_order_relaxed);
}
static DRAM_ATTR atomic_uint retry_failure[10];
void ull_radio_tx_retry_failure(uint32_t out[10]){
    out[0]=atomic_load_explicit(&retry_failure[0],memory_order_acquire);
    for(unsigned i=1;i<10;i++)out[i]=atomic_load_explicit(&retry_failure[i],memory_order_relaxed);
}
static int32_t IRAM_ATTR request_relative(radio_time_t now,uint32_t hs,uint16_t hus){
    int32_t coarse=(int32_t)(((now.hs-hs)+0x08000000u)&CLOCK_MASK)-0x08000000;
    return (int32_t)((int64_t)coarse*625+now.hus-hus);
}
static void IRAM_ATTR record_retry_failure(unsigned stage,int32_t entry,int32_t pre,int32_t post,unsigned status,unsigned required,unsigned duration,unsigned active,unsigned head){
    if(atomic_load_explicit(&retry_failure[0],memory_order_relaxed))return;
    uint32_t values[9]={stage,(uint32_t)entry,(uint32_t)pre,(uint32_t)post,status,required,duration,active,head};
    for(unsigned i=0;i<9;i++)atomic_store_explicit(&retry_failure[i+1],values[i],memory_order_relaxed);
    atomic_store_explicit(&retry_failure[0],1,memory_order_release);
}
static int32_t IRAM_ATTR insert_relative(radio_time_t now){
    int32_t coarse=(int32_t)(((now.hs-tx.arb.hs)+0x08000000u)&CLOCK_MASK)-0x08000000;
    return (int32_t)((int64_t)coarse*625+now.hus-tx.arb.hus);
}

void ull_radio_tx_retry_snapshot(uint32_t out[160]){
    if(!out)return;
    for(unsigned i=0;i<160;i++)out[i]=atomic_load_explicit(&retry_trace[i],memory_order_acquire);
}
static uint32_t IRAM_ATTR retry_et(uint8_t index,const struct tx_program *program){
    if(!tx_entry_is_ours(index,program,false))return UINT32_MAX;
    volatile const uint16_t *et=r_emi_get_mem_addr_by_offset((uint16_t)(16u*index));
    return et[0];
}
static void IRAM_ATTR retry_record(unsigned kind,unsigned stage,unsigned reason){
    if(!atomic_load_explicit(&retry_trace_enabled,memory_order_relaxed) || !tx.repeat_payload || retry_records>=24)return;
    radio_time_t now=r_rwip_time_get();
    int32_t coarse=(int32_t)(((now.hs-tx.arb.hs)+0x08000000u)&CLOCK_MASK)-0x08000000;
    uint32_t relative=(uint32_t)((int64_t)coarse*625+now.hus-tx.arb.hus);
    unsigned at=4+6*retry_records;
    uint32_t row[6]={kind,stage,reason,relative,
        retry_et(tx.program_index,&tx.program),stage==2?retry_et(tx.stop_index,&tx.stop_program):retry_et(tx.reply_index,&tx.reply_program)};
    for(unsigned i=0;i<6;i++)atomic_store_explicit(&retry_trace[at+i],row[i],memory_order_relaxed);
    atomic_store_explicit(&retry_trace[0],++retry_records,memory_order_release);
}
static void IRAM_ATTR retry_begin(void){
    if(!atomic_load_explicit(&retry_trace_enabled,memory_order_relaxed) || !tx.repeat_payload)return;
    retry_records=0;
    for(unsigned i=0;i<160;i++)atomic_store_explicit(&retry_trace[i],0,memory_order_relaxed);
    atomic_store_explicit(&retry_trace[1],rwip_prog_delay,memory_order_relaxed);
    atomic_store_explicit(&retry_trace[2],tx.program.duration_hus,memory_order_relaxed);
    atomic_store_explicit(&retry_trace[3],tx.reply_program.duration_hus,memory_order_relaxed);
    retry_record(1,255,0);
}
static void IRAM_ATTR __attribute__((noinline)) tx_restore_globals(void)
{
    if (!tx.globals_set) return;
    if(tx.rx_timing_set){
        RX_TIMING_2M=tx.saved_rx_timing;
        tx.result.rx_timing_restored=RX_TIMING_2M;
        tx.rx_timing_set=false;
    }
    TEST_CONTROL = (TEST_CONTROL & ~TEST_BITS) | tx.saved_test_bits;
    /* Abort is a hardware command, not a saved configuration bit. */
    RADIO_CONTROL = (RADIO_CONTROL & ~(TEST_ENABLE | RADIO_ABORT)) | tx.saved_enable;
    if(tx.sequence_flags_set){
        RADIO_CONTROL=(RADIO_CONTROL & ~AIR_SEQUENCE_BYPASS_MASK)|tx.saved_sequence_flags;
        tx.result.sequence_restored=RADIO_CONTROL;tx.sequence_flags_set=false;
    }
    tx.globals_set = false;
}
static void IRAM_ATTR tx_abort_active(void)
{
    /* Existing one-frame trace only: record whether the abort bit was already
     * asserted and which owned event is current; no extra register writes. */
    if(tx.repeat_payload){
        unsigned stage=tx_entry_is_ours(tx.stop_index,&tx.stop_program,true)?2u:255u;
        retry_record(5,stage,(RADIO_CONTROL & RADIO_ABORT)?1u:0u);
    }
    if (tx.result.phase != ULL_TX_ACTIVE && tx.result.phase != ULL_TX_ABORTING)
        return;
    tx.result.abort_checks=(tx_active_is_ours()?1u:0u)|(tx_program_is_ours(true)?2u:0u);
    if (tx.result.abort_checks!=3) {
        tx.result.fault_detail=1;
        tx.result.phase = ULL_TX_FAULT;
        return; /* Never abort somebody else's physical event. */
    }
    if(tx.repeat_payload && tx_entry_is_ours(tx.stop_index,&tx.stop_program,true))
        tx.stop_abort_requested=true; /* Includes cancellation/coarse fallback on stopper. */
    RADIO_CONTROL = (RADIO_CONTROL & ~TEST_ENABLE) | RADIO_ABORT;
    tx.result.abort_control=RADIO_CONTROL;
    tx.result.phase = ULL_TX_ABORTING;
}
static void IRAM_ATTR tx_deadline(tx_alarm_t *alarm)
{
    if (alarm != &tx.alarm) return;
    tx.alarm_set = false; /* ROM removed it before invoking us. */
    retry_record(6,255,0);
    tx.result.deadline_fired = 1;
    radio_time_t now=r_rwip_time_get();tx.result.deadline_hs=now.hs;
    tx.result.deadline_hus=(uint16_t)now.hus;
    tx_abort_active();
}
static void IRAM_ATTR tx_receive_owned(uint8_t activity)
{
    /* Pinned SDK r_lld_rxdesc_check_hack, env+216 and20-byte descriptors.
     * The stock checker stops if the head belongs to a different activity.
     * Never drain the parent ACL's packets or free a buffer before copying.
     * S3 stock buffer init: nine0x400-spaced slots beginning at EM0x7805. */
    if ((uintptr_t)&p_lld_env != UINT32_C(0x3fceff78) ||
        !tx_valid_ram(p_lld_env, 218) || !r_ip_funcs_p[169]) {
        tx.result.rx_invalid=1;return;
    }
    for (unsigned n=0; n<10; n++) {
        if (!((bool (*)(uint8_t))r_ip_funcs_p[169])(activity)) return;
        uint8_t slot=p_lld_env[216];
        if (slot>=10) {tx.result.rx_invalid=1;return;}
        volatile const uint16_t *d=r_emi_get_mem_addr_by_offset((uint16_t)(0x1000u+20u*slot));
        if (((d[6]>>11)&31u)!=activity) {tx.result.rx_invalid=1;return;}
        uint16_t offset=d[9];
        if (offset && (offset<0x7805 || offset>0x9805 || (offset-0x7805)%0x400)) {
            tx.result.rx_invalid=1;return;
        }
        if (activity!=3 && tx.result.rx_snapshots<3) {
            struct ull_radio_rx_snapshot *out=&tx.result.rx[tx.result.rx_snapshots++];
            radio_time_t now=r_rwip_time_get();out->hs=now.hs;out->hus=(uint16_t)now.hus;
            for(unsigned i=0;i<10;i++)out->descriptor[i]=d[i];
            unsigned bytes=d[2]>>8,capacity=r_ble_util_buf_get_rx_buf_size();
            if(bytes>capacity)bytes=capacity;
            if(bytes>sizeof(out->payload))bytes=sizeof(out->payload);
            if(!offset)bytes=0;
            out->copied=(uint8_t)bytes;
            if(bytes){
                volatile const uint8_t *payload=r_emi_get_mem_addr_by_offset(offset);
                for(unsigned i=0;i<bytes;i++)out->payload[i]=payload[i];
            }
        }
        r_lld_rxdesc_free();tx.result.rx_freed++;
    }
}
/* Called from the completed owned TX callback, after its arb element is
 * removed. The old program entry is retired by ROM when the callback returns;
 * the follow-up starts later and retains the empty-program-queue guard.
 * No allocation, crypto or USB operation occurs here. */
static void IRAM_ATTR __attribute__((noinline)) tx_queue_followup(void)
{
    tx.followup_pending=false;
    tx_arb_t *active=*(tx_arb_t **)(sch_arb_env+8);
    if(tx_activity_parameters()!=tx.activity_parameters || active==&tx.arb ||
       sch_prog_env[0x102]==0){
        tx.result.fault_detail=2;
        tx.result.phase=ULL_TX_FAULT;return;
    }
    /* Removing our completed reservation can start a due parent event.
     * That is a followup conflict, not loss of our inactive CS/storage.
     * Leave the new event untouched and collect after this entry retires. */
    if(active || sch_prog_env[0x102]>1){tx.result.followup_status=5;return;}
    radio_time_t now=r_rwip_time_get();
    uint32_t ahead=(tx.result.followup_hs-now.hs)&CLOCK_MASK;
    int64_t lead_hus=(int64_t)ahead*625+tx.result.followup_hus-now.hus;
    /* One coarse tick beyond the actual programming delay. Unlike the task
     * submission path, there is no timer/command dispatch between here and
     * insertion. Decline late events rather than programming an old instant. */
    if(!rwip_prog_delay || ahead>3200 ||
       lead_hus<(int64_t)(rwip_prog_delay+1u)*625){
        tx.result.followup_status=4;return;
    }
    tx.format=29;tx.followup_active=true;
    tx.cs[4/2]=0x1000|((tx.phy-1u)<<2);
    tx.cs[22/2]=0x8000|tx.followup_channel;
    tx.cs[26/2]=(tx.followup_window_us+1u)/2u;
    for(unsigned offset=68;offset<=78;offset+=2)tx.cs[offset/2]=0;
    uint32_t ticks=(tx.result.followup_hus+2u*tx.followup_window_us+624u)/625u+1u;
    tx.arb.next=NULL;tx.arb.hs=tx.result.followup_hs;
    tx.arb.hus=tx.result.followup_hus;tx.arb.limit_hs=0;
    tx.arb.duration_hus=(ticks+2u)*625u-tx.arb.hus;
    tx.program.hs=tx.arb.hs;tx.program.hus=tx.arb.hus;
    tx.program.duration_hus=tx.arb.duration_hus;
    tx.alarm.next=NULL;tx.alarm.hs=(tx.arb.hs+ticks)&CLOCK_MASK;
    tx.program_index=255;tx.result.phase=ULL_TX_QUEUED;
    if(r_sch_arb_insert(&tx.arb)){
        tx.result.followup_status=5;tx.result.phase=ULL_TX_ENDED;
    }else tx.result.followup_status=1;
}
static void IRAM_ATTR tx_frame_done(uint32_t hs, uint32_t context, uint8_t reason)
{
    bool stop=tx.repeat_payload && context==tx.stop_program.context;
    bool reply=tx.prequeued_reply && context==tx.reply_program.context;
    retry_record(4,stop?2:reply?1:context==tx.program.context?0:255,reason);
    const struct tx_program *program=stop?&tx.stop_program:reply?&tx.reply_program:&tx.program;
    uint8_t index=stop?tx.stop_index:reply?tx.reply_index:tx.program_index;
    bool previous_pending=stop?(!tx.first_ended || !tx.reply_ended):reply && !tx.first_ended;
    if(context!=program->context || hs!=program->hs ||
       !tx_entry_is_ours(index,program,false) || (previous_pending && reason!=4)) {
        tx.result.callback_reason=reason;
        tx.result.fault_detail=(uint8_t)(0x10u|(context!=program->context?1u:0u)|
            (hs!=program->hs?2u:0u)|(!tx_entry_is_ours(index,program,false)?4u:0u)|
            ((previous_pending && reason!=4)?8u:0u));
        tx.result.phase=ULL_TX_FAULT;return;
    }
    if(tx.prequeued_reply && (stop?tx.stop_ended:reply?tx.reply_ended:tx.first_ended)){
        if(reason==0 || reason==1 || reason==4)return;
        tx.result.fault_detail=0x20;tx.result.phase=ULL_TX_FAULT;return;
    }
    if(stop && reason==3){tx.result.fault_detail=0x22;tx.result.phase=ULL_TX_FAULT;return;}
    /* ROM end ISR: 0=completed, 1=aborted; skip ISR: 4=skipped.
     * RX/TX per-packet callbacks (2/3) do not end descriptor ownership.
     */
    tx.result.test_at_callback = TEST_CONTROL;
    if(tx.rx_timing_set)tx.result.rx_timing_callback=RX_TIMING_2M;
    if(tx.sequence_flags_set)tx.result.sequence_callback=RADIO_CONTROL;
    if(reason==2){
        tx.result.rx_callbacks++;tx_receive_owned(stop?3:reply?2:ACTIVITY);
        if(stop)return; /* Free owned RX without publishing stopper audio/control. */
        /* This diagnostic needs one complete packet. Once native TX receives
         * it, stop that owned event and omit the redundant late follow-up.
         * Otherwise the native connection format may retransmit and wait for
         * another timeout, costing the next Air interval's preparation time.
         * Authentication stays in controller task context; hardware CRC is
         * used here only to avoid stopping early on an obviously bad packet. */
        bool native=tx.followup_pending && !tx.prequeued_reply && tx.format==2;
        /* The first native event must stop after its reply too. Otherwise
         * the controller continues its normal connection exchange into the
         * already queued retry slot. Abort only the currently owned entry;
         * retain the second entry and shared storage until both retire. */
        bool repeat_first=tx.repeat_payload && !reply;
        if((tx.followup_active || reply || native || repeat_first) && !tx.result.rx_invalid){
            for(unsigned i=0;i<tx.result.rx_snapshots;i++){
                const struct ull_radio_rx_snapshot *s=&tx.result.rx[i];
                if(s->copied>=7 && s->copied==(s->descriptor[2]>>8) &&
                   !(s->descriptor[1]&9u)){
                    if(native){tx.followup_pending=false;tx.result.followup_status=7;}
                    tx_abort_active();break;
                }
            }
        }
        return;
    }
    if(reason==3){tx.result.tx_callbacks++;return;}
    if (reason != 0 && reason != 1 && reason != 4) return;
    if(reason!=4)tx_receive_owned(stop?3:reply?2:ACTIVITY);
    if(tx.prequeued_reply){
        if(stop){tx.stop_ended=true;}else if(reply){
            tx.reply_ended=true;
            tx.result.followup_status=reason==0?3:6;
        }else{
            if(tx.first_alarm_set){r_sch_alarm_clear(&tx.first_alarm);tx.first_alarm_set=false;}
            tx.first_ended=true;tx.result.tx_count=tx.cs[68/2];
            radio_time_t now=r_rwip_time_get();
            tx.result.first_end_hs=now.hs;tx.result.first_end_hus=(uint16_t)now.hus;
        }
        if(!tx.first_ended || !tx.reply_ended || (tx.repeat_payload && !tx.stop_ended))return;
        if(tx.alarm_set){r_sch_alarm_clear(&tx.alarm);tx.alarm_set=false;}
        tx.result.callback_reason=reason;
        tx.result.end_hs=r_rwip_time_get().hs;
        if(tx.result.phase!=ULL_TX_FAULT)tx.result.phase=ULL_TX_ENDED;
        /* Both callbacks happened, but ROM still owns the current entry.
         * Task collection waits for both used bits before any restoration. */
        return;
    }
    if (tx.alarm_set) {
        r_sch_alarm_clear(&tx.alarm);
        tx.alarm_set = false;
    }
    if(tx.first_alarm_set){r_sch_alarm_clear(&tx.first_alarm);tx.first_alarm_set=false;}
    if(!tx.followup_active && !reply)tx.result.tx_count = tx.cs[68 / 2];
    tx.result.callback_reason = reason;
    tx.result.end_hs = r_rwip_time_get().hs;
    tx_restore_globals();
    if (tx_active_is_ours()) r_sch_arb_remove(&tx.arb, 1);
    if(tx.result.phase!=ULL_TX_FAULT)tx.result.phase = ULL_TX_ENDED;
    if(tx.followup_active || reply)tx.result.followup_status=reason==0?3:6;
    if(tx.followup_pending && reason==0 && tx.result.phase==ULL_TX_ENDED &&
       !tx.result.rx_invalid){
        radio_time_t now=r_rwip_time_get();
        tx.result.first_end_hs=now.hs;tx.result.first_end_hus=(uint16_t)now.hus;
        tx_queue_followup();
    }
    /* Release payload/restore CS from the task after ROM retires this entry. */
}
static void IRAM_ATTR tx_cancelled(tx_arb_t *event)
{
    if (event != &tx.arb || tx.result.phase != ULL_TX_QUEUED) {
        tx.result.fault_detail=3;
        tx.result.phase = ULL_TX_FAULT;
        return;
    }
    tx.result.phase = ULL_TX_CANCELLED; /* Already unlinked by ROM. */
}
static void IRAM_ATTR tx_stopped(tx_arb_t *event)
{
    if (event == &tx.arb) tx_abort_active();
}
static void IRAM_ATTR tx_started(tx_arb_t *event)
{
    retry_begin();
    if (event != &tx.arb || tx.result.phase != ULL_TX_QUEUED ||
        !tx_active_is_ours() || tx_activity_parameters() != tx.activity_parameters ||
        (tx.prequeued_reply && tx_parameters_for(2,0xef)!=tx.reply_parameters) ||
        (tx.repeat_payload && tx_parameters_for(3,0xed)!=tx.stop_parameters) ||
        sch_prog_env[0x102] != 0 ||
        (RADIO_CONTROL & (TEST_ENABLE | RADIO_ABORT)) || (TEST_CONTROL & TEST_BITS)) {
        if (tx_active_is_ours()) r_sch_arb_remove(&tx.arb, 1);
        tx.result.phase = ULL_TX_REFUSED;
        return;
    }
    bool timing_probe=(ULL_RX_TIMING_FIELD_ADD || ULL_TX_PATH_FIELD_ADD) && tx.format==TX_FORMAT &&
                      tx.phy==2 && (!tx.prequeued_reply || tx.repeat_payload);
    if(timing_probe && RX_TIMING_2M!=UINT32_C(0x00020407)){
        r_sch_arb_remove(&tx.arb,1);tx.result.phase=ULL_TX_REFUSED;return;
    }
    tx.program_index = sch_prog_env[0x101];
    if (tx.program_index >= 16) {
        tx.result.fault_detail=4;
        r_sch_arb_remove(&tx.arb, 1);
        tx.result.phase = ULL_TX_FAULT;
        return;
    }
    uint8_t antenna7 = 0, antenna6 = 0;
    ((void (*)(uint8_t, uint8_t, uint8_t *, uint8_t *))r_modules_funcs_p[120])
        (tx.format, ACTIVITY, &antenna7, &antenna6);
    if (antenna7 > 1 || antenna6 > 1) {
        r_sch_arb_remove(&tx.arb, 1);
        tx.result.phase = ULL_TX_REFUSED;
        return;
    }
    tx.cs[0] = tx.format | ((uint16_t)antenna7 << 7) | ((uint16_t)antenna6 << 6);
    ((void (*)(uint8_t, uint8_t))r_ip_funcs_p[133])(ACTIVITY, tx.format==29 ? 29 : 28);
    atomic_store_explicit(&power_raw_index,(unsigned)(tx.cs[24/2]&0xffu),memory_order_relaxed);
    atomic_store_explicit(&power_event_count,++power_events,memory_order_release);
    if(tx.prequeued_reply){
        antenna7=antenna6=0;
        ((void (*)(uint8_t,uint8_t,uint8_t *,uint8_t *))r_modules_funcs_p[120])
            (tx.repeat_payload?TX_FORMAT:29,2,&antenna7,&antenna6);
        if(antenna7>1 || antenna6>1){
            r_sch_arb_remove(&tx.arb,1);tx.result.phase=ULL_TX_REFUSED;return;
        }
        tx.reply_cs[0]=(tx.repeat_payload?TX_FORMAT:29)|((uint16_t)antenna7<<7)|((uint16_t)antenna6<<6);
        ((void (*)(uint8_t,uint8_t))r_ip_funcs_p[133])(2,tx.repeat_payload?28:29);
    }
    if(tx.repeat_payload){
        antenna7=antenna6=0;
        ((void (*)(uint8_t,uint8_t,uint8_t *,uint8_t *))r_modules_funcs_p[120])
            (29,3,&antenna7,&antenna6);
        if(antenna7>1 || antenna6>1){r_sch_arb_remove(&tx.arb,1);tx.result.phase=ULL_TX_REFUSED;return;}
        tx.stop_cs[0]=29|((uint16_t)antenna7<<7)|((uint16_t)antenna6<<6);
        ((void (*)(uint8_t,uint8_t))r_ip_funcs_p[133])(3,29);
    }
    tx.saved_enable = RADIO_CONTROL & TEST_ENABLE;
    tx.saved_test_bits = TEST_CONTROL & TEST_BITS;
    tx.globals_set = true;
    if(timing_probe){
        tx.saved_rx_timing=RX_TIMING_2M;
        tx.result.rx_timing_before=tx.saved_rx_timing;
        RX_TIMING_2M=tx.saved_rx_timing+(ULL_RX_TIMING_FIELD_ADD<<16)+ULL_TX_PATH_FIELD_ADD;
        tx.rx_timing_set=true;tx.result.rx_timing_active=RX_TIMING_2M;
    }
    /* Ordinary data format, without test-pattern processing. Caller compensates
     * payload whitening; external RX must validate the resulting bytes. */
    RADIO_CONTROL = RADIO_CONTROL & ~TEST_ENABLE;
    TEST_CONTROL = TEST_CONTROL & ~TEST_BITS;
    if(ULL_AIR_SEQUENCE_BYPASS && tx.format==TX_FORMAT && tx.phy==2 && (!tx.prequeued_reply || tx.repeat_payload)){
        tx.result.sequence_before=RADIO_CONTROL;
        tx.saved_sequence_flags=RADIO_CONTROL&AIR_SEQUENCE_BYPASS_MASK;
        RADIO_CONTROL=RADIO_CONTROL|AIR_SEQUENCE_BYPASS_MASK;
        tx.sequence_flags_set=true;tx.result.sequence_active=RADIO_CONTROL;
    }
    tx.result.test_configured = TEST_CONTROL;
    tx.result.phase = ULL_TX_ACTIVE;
    if(tx.followup_active)tx.result.followup_status=2;
    r_sch_prog_push(&tx.program);
    retry_record(2,0,0);
    tx.result.test_programmed = TEST_CONTROL;
    if (!tx_program_is_ours(false)) {
        tx.result.fault_detail=5;
        tx.result.phase = ULL_TX_FAULT;
        return;
    }
    if(tx.prequeued_reply){
        tx.reply_index=sch_prog_env[0x101];
        if(tx.reply_index>=16 || tx.reply_index==tx.program_index ||
           sch_prog_env[0x102]!=1){tx.result.fault_detail=6;tx.result.phase=ULL_TX_FAULT;return;}
        r_sch_prog_push(&tx.reply_program);
        retry_record(3,1,0);
        if(!tx_entry_is_ours(tx.reply_index,&tx.reply_program,false)){
            tx.result.fault_detail=7;
            tx.result.phase=ULL_TX_FAULT;return;
        }
        tx.result.followup_status=1;
    }
    if(tx.repeat_payload){
        tx.stop_index=sch_prog_env[0x101];
        if(tx.stop_index>=16 || tx.stop_index==tx.program_index || tx.stop_index==tx.reply_index ||
           sch_prog_env[0x102]!=2){tx.result.fault_detail=9;tx.result.phase=ULL_TX_FAULT;return;}
        r_sch_prog_push(&tx.stop_program);retry_record(3,2,0);
        if(!tx_entry_is_ours(tx.stop_index,&tx.stop_program,false)){
            tx.result.fault_detail=10;tx.result.phase=ULL_TX_FAULT;return;
        }
    }
    r_sch_alarm_set(&tx.alarm);
    tx.alarm_set = true;
    /* Repeat probe retains native first RX; no early airtime cutoff. */
}

uint8_t IRAM_ATTR ull_radio_tx_submit(const struct ull_radio_tx_request *r)
{
    if (!r || !r->payload || !r->length || r->length > 251 || r->channel > 39 ||
        (r->header > 15 && r->header != 0x10 && r->header != 0x11 &&
         r->header != 0x20 && r->header != 0x30 &&
         !(r->header==0x32 && (r->length==205 || r->length==15) && r->phy==2)) ||
        r->phy < 1 || r->phy > 2 ||
        r->start_hs > CLOCK_MASK || r->start_hus > 624 || r->crc_init > 0xffffff ||
        r->receive_us > 1000 || r->tx_receive_us > 1000 ||
        (r->receive_us && r->tx_receive_us) ||
        (r->prequeued_reply>1) ||
        r->repeat_payload>1 ||
        (r->repeat_payload && (!r->prequeued_reply ||
          !((r->length==203 && r->header==0x30) ||
            (r->length==205 && r->header==0x32)) ||
          r->phy!=2 || r->followup_delay_us!=1420)) ||
        (r->prequeued_reply && (!r->followup_delay_us ||
          r->followup_delay_us<(r->phy==1 ? (r->length+10u)*8u : (r->length+11u)*4u)+300u)) ||
        (r->followup_delay_us && (r->receive_us || r->tx_receive_us ||
          r->followup_delay_us<(r->prequeued_reply?700:1800) || r->followup_delay_us>3500 ||
          !r->followup_window_us || r->followup_window_us>1000 ||
          r->followup_channel>39)) ||
        (!r->followup_delay_us && r->followup_window_us))
        return 0x12;
    lock();
    if (tx.result.phase != ULL_TX_IDLE || !r_ip_funcs_p || !r_modules_funcs_p ||
        (uintptr_t)sch_prog_env != UINT32_C(0x3fcefa8c) ||
        (uintptr_t)sch_arb_env != UINT32_C(0x3fcefb98)) {
        unlock(); return 0x0c;
    }
    uint8_t *parameters = tx_activity_parameters();
    uint8_t *reply_parameters=r->prequeued_reply?tx_parameters_for(2,0xef):NULL;
    uint8_t *stop_parameters=r->repeat_payload?tx_parameters_for(3,0xed):NULL;
    radio_time_t now = r_rwip_time_get();
    uint32_t ahead = (r->start_hs - now.hs) & CLOCK_MASK;
    if (!parameters || (r->prequeued_reply && !reply_parameters) || (r->repeat_payload && !stop_parameters)) {
        unlock(); return 0x0c;
    }
    if (ahead < ull_radio_tx_min_lead() || ahead > 3200) {
        if(r->repeat_payload){int32_t at=request_relative(now,r->start_hs,r->start_hus);
            record_retry_failure(1,at,at,at,ULL_TX_TIME_EXPIRED,(uint32_t)rwip_prog_delay*625u+1000u,0,0,0);}
        unlock(); return ULL_TX_TIME_EXPIRED;
    }
    uint16_t payload = ((uint16_t (*)(uint16_t))r_ip_funcs_p[49])(255);
    if (!payload) { unlock(); return 0x07; }
    memset(&tx, 0, sizeof(tx));
    retry_insert_cancelled=false;
    tx.payload_offset = payload;
    tx.activity_parameters = parameters;
    tx.program_index = 255;
    tx.stop_index=255;tx.stop_parameters=stop_parameters;
    tx.reply_index=255;tx.prequeued_reply=r->prequeued_reply!=0;
    tx.repeat_payload=r->repeat_payload!=0;
    tx.reply_parameters=reply_parameters;
    tx.format = r->receive_us ? 29 : TX_FORMAT;
    tx.phy=r->phy;
    if(r->followup_delay_us){
        uint32_t fine=r->start_hus+2u*r->followup_delay_us;
        tx.result.followup_hs=(r->start_hs+fine/625u)&CLOCK_MASK;
        tx.result.followup_hus=(uint16_t)(fine%625u);
        tx.followup_window_us=r->followup_window_us;
        tx.followup_channel=r->followup_channel;tx.followup_pending=!tx.prequeued_reply;
    }
    tx.cs = r_emi_get_mem_addr_by_offset(CS_OFFSET);
    tx.descriptor = r_emi_get_mem_addr_by_offset(DESC_OFFSET);
    /* Keep controller-initialized RF fields that the stock test driver leaves
     * intact. Zeroing the whole CS would discard unmodeled calibration state. */
    for (unsigned i = 0; i < 45; i++) tx.saved_cs[i] = tx.cs[i];
    if(tx.prequeued_reply){
        tx.reply_cs=r_emi_get_mem_addr_by_offset(0x400u+90u*2u);
        for(unsigned i=0;i<45;i++)tx.saved_reply_cs[i]=tx.reply_cs[i];
    }
    for (unsigned i = 0; i < 7; i++) tx.saved_descriptor[i] = tx.descriptor[i];
    if(tx.repeat_payload){
        tx.stop_cs=r_emi_get_mem_addr_by_offset(0x400u+90u*3u);
        for(unsigned i=0;i<45;i++)tx.saved_stop_cs[i]=tx.stop_cs[i];
        tx.repeat_descriptor=r_emi_get_mem_addr_by_offset(REPEAT_DESC_OFFSET);
        for(unsigned i=0;i<7;i++)tx.saved_repeat_descriptor[i]=tx.repeat_descriptor[i];
    }
    tx.storage_set = true;
    memcpy(r_emi_get_mem_addr_by_offset(payload), r->payload, r->length);
    memcpy(tx.result.supplied_prefix,r_emi_get_mem_addr_by_offset(payload),
           r->length<8?r->length:8);
    tx.descriptor[0] = 0; /* No link to a second descriptor. */
    tx.descriptor[1] = (uint16_t)r->header | ((uint16_t)r->length << 8);
    tx.descriptor[2] = payload;
    for(unsigned i=3;i<7;i++)tx.descriptor[i]=0;
    if(tx.repeat_payload)
        for(unsigned i=0;i<7;i++)tx.repeat_descriptor[i]=tx.descriptor[i];
    /* Stock unencrypted connection initialization: RX/TX rates and controls. */
    tx.cs[2 / 2] = 0x0800 | ACTIVITY; /* ROM4001bb49..59: link label, crypto off. */
    tx.cs[4 / 2] = r->receive_us ? 0x1000 | ((r->phy - 1) << 2) :
                                 0x1100 | (r->phy - 1) | ((r->phy - 1) << 2);
    tx.cs[12 / 2] = r->access_address;
    tx.cs[14 / 2] = r->access_address >> 16;
    tx.cs[16 / 2] = r->crc_init;
    tx.cs[18 / 2] = r->crc_init >> 16;
    tx.cs[22 / 2] = 0x8000 | r->channel;
    /* A long stock DTM window (0x8026) received authenticated replies but
     * overran its deadline by5..25ms. Test the narrow-window encoding instead:
     * normal connection value30 ended roughly60us after expected sync. The
     * proposed /2 scale must be verified against actual callback timestamps. */
    uint16_t receive_window=r->receive_us ? r->receive_us : r->tx_receive_us;
    tx.cs[26 / 2] = receive_window ? (uint16_t)((receive_window+1u)/2u) : sdk_cfg_priv_opts[26];
    tx.cs[28 / 2] = DESC_OFFSET;
    tx.cs[30 / 2] = 0;
    tx.cs[32 / 2] = 0;
    /* All37 data channels usable; CSA1 increment0 retains the requested
     * channel in this one-event experiment. No empty hardware channel map. */
    tx.cs[34 / 2] = 0xffff;
    tx.cs[36 / 2] = 0xffff;
    tx.cs[38 / 2] = 0x001f;
    tx.cs[40 / 2] = 251;
    tx.cs[42 / 2] = 0;
    for (unsigned offset = 68; offset <= 78; offset += 2) tx.cs[offset / 2] = 0;
    if(tx.prequeued_reply){
        /* Preserve CS2's calibration fields; copy only fields configured above.
         * The reply has its own CS/owner label, while the TX descriptor remains
         * owned until both events retire. Dedicated RX does not transmit it. */
        static const uint8_t offsets[]={2,4,12,14,16,18,22,26,28,30,32,34,36,38,40,42,68,70,72,74,76,78};
        for(unsigned i=0;i<sizeof(offsets);i++)tx.reply_cs[offsets[i]/2]=tx.cs[offsets[i]/2];
        tx.reply_cs[2/2]=0x0800|2;
        tx.reply_cs[4/2]=tx.repeat_payload?tx.cs[4/2]:(0x1000|((r->phy-1u)<<2));
        if(tx.repeat_payload)tx.reply_cs[28/2]=REPEAT_DESC_OFFSET;
        tx.reply_cs[22/2]=0x8000|r->followup_channel;
        tx.reply_cs[26/2]=(r->followup_window_us+1u)/2u;
        if(tx.repeat_payload){
            for(unsigned i=0;i<sizeof(offsets);i++)tx.stop_cs[offsets[i]/2]=tx.cs[offsets[i]/2];
            tx.stop_cs[2/2]=0x0800|3;tx.stop_cs[4/2]=0x1000|((r->phy-1u)<<2);
            tx.stop_cs[22/2]=0x8000|r->followup_channel;tx.stop_cs[26/2]=10;
            tx.stop_cs[28/2]=0;tx.stop_cs[30/2]=0; /* RX-only: no TX descriptor. */
        }
    }

    /* Conservative BLE-shaped packet airtime, then one coarse tick of margin.
     * Hardware test mode can repeat packets. This is a bounded burst, not yet
     * proof of one-shot Air framing. Reserve two further ticks for abort/ISR.
     */
    uint32_t air_us = r->phy == 1 ? (r->length + 10) * 8 : (r->length + 11) * 4;
    if(r->receive_us)air_us=r->receive_us;
    else if(r->prequeued_reply)
        air_us=r->followup_delay_us+r->followup_window_us+300u+
            (r->repeat_payload?air_us+150u:0u);
    else if(r->tx_receive_us){
        /* Keep the same owned native event alive for a first-slot reply.
         * Include TIFS, sync search and a maximum retained RX packet, then
         * retain the existing coarse deadline/ownership-abort margin. */
        uint32_t reply_us=r->phy==1 ? (64u+10u)*8u : (64u+11u)*4u;
        air_us+=150u+r->tx_receive_us+reply_us;
    }
    uint32_t burst_ticks = (r->start_hus + 2 * air_us + 624) / 625 + 1;
    tx.arb.hs = r->start_hs;
    tx.arb.hus = r->start_hus;
    tx.arb.duration_hus = (burst_ticks + 2) * 625 - r->start_hus;
    tx.arb.start = tx_started;
    tx.arb.stop = tx_stopped;
    tx.arb.cancel = tx_cancelled;
    tx.alarm.hs = (r->start_hs + burst_ticks) & CLOCK_MASK;
    tx.alarm.callback = tx_deadline;
    tx.program.callback = tx_frame_done;
    tx.program.hs = tx.arb.hs;
    tx.program.hus = tx.arb.hus;
    tx.program.duration_hus = tx.arb.duration_hus;
    tx.program.context = (uint32_t)(uintptr_t)&tx;
    tx.program.mode = 15;
    tx.program.cs = ACTIVITY;
    if(tx.prequeued_reply){
        tx.reply_program=tx.program;
        tx.reply_program.hs=tx.result.followup_hs;
        tx.reply_program.hus=tx.result.followup_hus;
        tx.reply_program.context=((uint32_t)(uintptr_t)&tx)^1u;
        tx.reply_program.cs=2;
        /* Single-frame probe only. Pinned SDK maps byte20 to ET[15:11].
         * Test selection/preemption of our own first event, not the ACL. */
        if(tx.repeat_payload)tx.reply_program.priority=1;
        tx.reply_program.duration_hus=tx.arb.duration_hus-2u*r->followup_delay_us;
        tx.program.duration_hus=2u*r->followup_delay_us;
        if(tx.repeat_payload){
            tx.stop_program=tx.program;
            unsigned fine=r->start_hus+5950u;
            tx.stop_program.hs=(r->start_hs+fine/625u)&CLOCK_MASK;
            tx.stop_program.hus=fine%625u;
            tx.stop_program.context=((uint32_t)(uintptr_t)&tx)^2u;
            tx.stop_program.cs=3;tx.stop_program.priority=2;tx.stop_program.duration_hus=40;
            /* Live retry03: stopper needs the owned RF-test abort and ends
             * ~154us later. Arm at the first coarse tick after its20us window,
             * retaining the longer shared reservation until terminal cleanup.
             * Fine-phase rounding and actual collection must be measured. */
            uint32_t stop_ticks=(r->start_hus+5950u+40u+624u)/625u;
            tx.alarm.hs=(r->start_hs+stop_ticks)&CLOCK_MASK;
        }

    }
    tx.result.phase = ULL_TX_QUEUED;
    bool measure=retry_insert_pending;
    retry_insert_pending=false;
    uint32_t required=0,active=0,head=0;
    int32_t before=0;
    if(measure){
        atomic_store_explicit(&retry_insert_trace[0],0,memory_order_release);
        required=(uint32_t)rwip_prog_delay*625u+1000u;
        tx_arb_t *owner=*(tx_arb_t **)(sch_arb_env+8);
        active=(owner?1u:0u)|(owner==&tx.arb?2u:0u);
        head=*(tx_arb_t **)sch_arb_env!=NULL;
        before=insert_relative(r_rwip_time_get());
    }
    int32_t repeat_before=0;
    if(r->repeat_payload)repeat_before=insert_relative(r_rwip_time_get());
    uint8_t status = r_sch_arb_insert(&tx.arb);
    if(status && r->repeat_payload){
        int32_t after=insert_relative(r_rwip_time_get());
        tx_arb_t *owner=*(tx_arb_t **)(sch_arb_env+8);
        record_retry_failure(2,request_relative(now,r->start_hs,r->start_hus),repeat_before,after,status,
            (uint32_t)rwip_prog_delay*625u+1000u,tx.arb.duration_hus,
            (owner?1u:0u)|(owner==&tx.arb?2u:0u),*(tx_arb_t **)sch_arb_env!=NULL);
    }
    if(measure){
        int32_t after=insert_relative(r_rwip_time_get());
        uint32_t values[7]={(uint32_t)before,(uint32_t)after,status,required,
                            tx.arb.duration_hus,active,head};
        for(unsigned i=0;i<7;i++)atomic_store_explicit(&retry_insert_trace[i+1],values[i],memory_order_relaxed);
        atomic_store_explicit(&retry_insert_trace[0],1,memory_order_release);
    }
    if (status) {
        tx_release_storage();
        memset(&tx, 0, sizeof(tx));
        tx.program_index = 255;
    }
    unlock();
    return status ? ULL_TX_SCHEDULE_CONFLICT : 0;
}

uint8_t ull_radio_pdu_submit(const struct ull_radio_pdu_request *r)
{
    if (!r) return 0x12;
    uint8_t bytes[ULL_PHY_PACKET_MAX];
    size_t length;
    uint32_t hs;
    uint16_t hus;
    if (ull_phy_packet_build(bytes, sizeof(bytes), &length, r->access_address,
                              r->crc_init, r->channel, r->phy, r->pdu, r->length) ||
        ull_phy_dtm_start(r->start_hs, r->start_hus, r->phy, &hs, &hus) ||
        ull_phy_outer_whitening(bytes,length,r->channel))
        return 0x12;
    const struct ull_radio_tx_request outer = {
        .start_hs = hs, .start_hus = hus,
        .access_address = UINT32_C(0x71764129), .crc_init = 0x555555,
        .payload = bytes, .length = length, .channel = r->channel,
        .header = 1, .phy = r->phy,
    };
    return ull_radio_tx_submit(&outer);
}

void ull_radio_tx_cancel(void)
{
    lock();
    retry_insert_pending=false;retry_insert_cancelled=true;
    tx.followup_pending=false;
    if (tx.result.phase == ULL_TX_QUEUED) {
        uint8_t status = r_sch_arb_remove(&tx.arb, 0);
        if (!status && !tx_active_is_ours()) tx.result.phase = ULL_TX_CANCELLED;
    } else if(tx.repeat_payload &&
              (tx.result.phase==ULL_TX_ACTIVE || tx.result.phase==ULL_TX_ABORTING)){
        /* A started repeat already owns three bounded programs. Stop requests
         * may arrive between them: do not inject global abort into a queued
         * successor. Existing RX/stopper/deadline paths still end the events.
         * Validate only our entries; never restore/free before normal collect. */
        const struct tx_program *programs[3]={&tx.program,&tx.reply_program,&tx.stop_program};
        const uint8_t indices[3]={tx.program_index,tx.reply_index,tx.stop_index};
        const bool ended[3]={tx.first_ended,tx.reply_ended,tx.stop_ended};
        const volatile struct program_entry *entries=(const volatile struct program_entry *)sch_prog_env;
        bool valid=tx_active_is_ours();
        for(unsigned i=0;i<3;i++){
            if(indices[i]>=16){valid=false;continue;}
            if(entries[indices[i]].used){
                if(!tx_entry_is_ours(indices[i],programs[i],false))valid=false;
            }else if(!ended[i])valid=false;
        }
        if(!valid){
            tx.result.fault_detail=1;tx.result.phase=ULL_TX_FAULT;
        }
    } else tx_abort_active();
    unlock();
}
int IRAM_ATTR ull_radio_tx_collect(struct ull_radio_tx_result *result)
{
    if (!result) return -1;
    lock();
    *result = tx.result;
    if (tx.result.phase == ULL_TX_FAULT ||
        (tx.storage_set && (tx_activity_parameters() != tx.activity_parameters ||
          (tx.prequeued_reply && tx_parameters_for(2,0xef)!=tx.reply_parameters) ||
          (tx.repeat_payload && tx_parameters_for(3,0xed)!=tx.stop_parameters)))) {
        retry_insert_pending=false;
        unlock(); return -1;
    }
    /* Repeat-only fine timeout. Identity plus consumer index is insufficient
     * for a queued entry: require ET state2 (active), observed in owned probes.
     * Keep the coarse owned alarm as an independent fallback. */
    if(tx.repeat_payload && tx.first_ended && tx.reply_ended && !tx.stop_ended &&
       !tx.stop_abort_requested && (tx.result.phase==ULL_TX_ACTIVE || tx.result.phase==ULL_TX_ABORTING) &&
       tx_active_is_ours() && tx_entry_is_ours(tx.stop_index,&tx.stop_program,true) &&
       ((retry_et(tx.stop_index,&tx.stop_program)>>3)&7u)==2u){
        radio_time_t now=r_rwip_time_get();
        int32_t coarse=(int32_t)(((now.hs-tx.stop_program.hs)+0x08000000u)&CLOCK_MASK)-0x08000000;
        int64_t elapsed=(int64_t)coarse*625+now.hus-tx.stop_program.hus;
        if(elapsed>=40){
            tx.stop_abort_requested=true;
            tx_abort_active(); /* Rechecks both arb and current entry before MMIO. */
            *result=tx.result;
        }
    }
    if(tx.prequeued_reply && tx.first_ended && tx.reply_ended && (!tx.repeat_payload || tx.stop_ended)){
        const volatile struct program_entry *entries=(const volatile struct program_entry *)sch_prog_env;
        bool first_used=entries[tx.program_index].used,reply_used=entries[tx.reply_index].used;
        bool stop_used=tx.repeat_payload && entries[tx.stop_index].used;
        if((first_used && !tx_entry_is_ours(tx.program_index,&tx.program,false)) ||
           (reply_used && !tx_entry_is_ours(tx.reply_index,&tx.reply_program,false)) ||
           (stop_used && !tx_entry_is_ours(tx.stop_index,&tx.stop_program,false)) ||
           (*(tx_arb_t **)(sch_arb_env+8) && !tx_active_is_ours())){
            tx.result.fault_detail=0x21;tx.result.phase=ULL_TX_FAULT;
            *result=tx.result;retry_insert_pending=false;unlock();return -1; /* Never restore over a foreign event. */
        }
        if(first_used || reply_used || stop_used){unlock();return 0;}
        tx_restore_globals();
        if(tx_active_is_ours())r_sch_arb_remove(&tx.arb,1);
        *result=tx.result;
    }
    if (tx.result.phase == ULL_TX_QUEUED || tx.result.phase == ULL_TX_ACTIVE ||
        tx.result.phase == ULL_TX_ABORTING || tx.globals_set || tx.alarm_set || tx.first_alarm_set ||
        tx_active_is_ours() || tx_program_is_ours(false)) {
        unlock(); return 0;
    }
    retry_record(7,255,0);
    if(atomic_load_explicit(&retry_trace_enabled,memory_order_relaxed) && tx.repeat_payload && !retry_insert_cancelled && tx.result.phase==ULL_TX_ENDED)retry_insert_pending=true;
    tx_release_storage();
    memset(&tx, 0, sizeof(tx));
    tx.program_index = 255;
    unlock(); return 1;
}
