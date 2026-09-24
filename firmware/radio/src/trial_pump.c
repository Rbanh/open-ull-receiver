#include "trial_pump.h"
#include "esp_bt.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include <stdatomic.h>
#include <string.h>

extern void ull_controller_diag_record(const uint8_t *, unsigned);
#ifndef ULL_TRIAL_TRACE_CAPACITY
#define ULL_TRIAL_TRACE_CAPACITY 98304
#endif
_Static_assert(ULL_TRIAL_TRACE_CAPACITY>=256 && ULL_TRIAL_TRACE_CAPACITY<=98304,
               "Bounded diagnostic trace allocation");
enum {TRACE_CAPACITY=ULL_TRIAL_TRACE_CAPACITY, TRACE_RESERVE=128, PERIOD_US=100};
static atomic_flag sender=ATOMIC_FLAG_INIT;
static atomic_bool active, pending, continuous;
static atomic_uint tick_running;
bool ull_trial_pump_continuous(bool value){
    if(ull_trial_pump_busy())return false;
    atomic_store(&continuous,value);return true;
}
static atomic_uint requests, steps, max_dispatch_us;
static int64_t pending_since, started;
static esp_timer_handle_t timer;
static portMUX_TYPE trace_mux=portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR uint8_t trace[TRACE_CAPACITY];
static uint32_t trace_used, trace_read, trace_lost;
static int64_t trace_base;
static bool recording, frozen;

bool ull_hci_try_send(uint8_t *data,uint16_t length)
{
    if(atomic_flag_test_and_set_explicit(&sender,memory_order_acquire))return false;
    bool sent=esp_vhci_host_check_send_available();
    if(sent)esp_vhci_host_send_packet(data,length);
    atomic_flag_clear_explicit(&sender,memory_order_release);
    return sent;
}

bool ull_trial_trace_record(const uint8_t *data,unsigned length)
{
    portENTER_CRITICAL_SAFE(&trace_mux);
    if(!recording){portEXIT_CRITICAL_SAFE(&trace_mux);return false;}
    bool terminal=length>=8 && ((!memcmp(data,"ULLP\3",5)) ||
        (!memcmp(data,"ULLA\1",5) && data[5]>=3) ||
        (length==12 && !memcmp(data,"ULL!\1",5)));
#if ULL_LIVE_USB
    if(!terminal){portEXIT_CRITICAL_SAFE(&trace_mux);return true;}
#endif
    unsigned limit=terminal ? TRACE_CAPACITY : TRACE_CAPACITY-TRACE_RESERVE;
    int64_t elapsed=esp_timer_get_time()-trace_base;
    if(!length || length>103 || elapsed<0 || elapsed>UINT32_MAX ||
       trace_used+5u+length>limit){trace_lost++;}
    else{
        uint32_t delta=(uint32_t)elapsed;
        memcpy(trace+trace_used,&delta,4);trace[trace_used+4]=(uint8_t)length;
        memcpy(trace+trace_used+5,data,length);trace_used+=5u+length;
    }
    portEXIT_CRITICAL_SAFE(&trace_mux);return true;
}

bool ull_trial_trace_next(uint8_t *data,unsigned capacity,unsigned *length,
                          int64_t *timestamp_us)
{
    portENTER_CRITICAL_SAFE(&trace_mux);
    if(!frozen || trace_read==trace_used){portEXIT_CRITICAL_SAFE(&trace_mux);return false;}
    uint32_t delta;memcpy(&delta,trace+trace_read,4);
    unsigned n=trace[trace_read+4];
    if(n>capacity || trace_read+5u+n>trace_used){
        portEXIT_CRITICAL_SAFE(&trace_mux);return false;
    }
    *timestamp_us=trace_base+delta;*length=n;
    memcpy(data,trace+trace_read+5,n);
    memset(trace+trace_read,0,5u+n);trace_read+=5u+n;
    if(trace_read==trace_used)frozen=false;
    portEXIT_CRITICAL_SAFE(&trace_mux);return true;
}

static void summary(uint8_t state,uint8_t status)
{
    uint8_t p[28]={'U','L','L','P',3,state,0,status};
    p[6]=(uint8_t)atomic_load(&pending);
    uint32_t n=atomic_load(&requests);memcpy(p+8,&n,4);
    n=atomic_load(&steps);memcpy(p+12,&n,4);
    portENTER_CRITICAL_SAFE(&trace_mux);
    memcpy(p+16,&trace_lost,4);memcpy(p+20,&trace_used,4);
    portEXIT_CRITICAL_SAFE(&trace_mux);
    n=atomic_load(&max_dispatch_us);memcpy(p+24,&n,4);
    ull_controller_diag_record(p,sizeof(p));
}

void ull_trial_pump_finish(uint8_t status)
{
    if(!atomic_exchange(&active,false))return;
    esp_timer_stop(timer);
    summary(status ? 3 : 2,status);
    portENTER_CRITICAL_SAFE(&trace_mux);
    recording=false;frozen=trace_used!=0;
    portEXIT_CRITICAL_SAFE(&trace_mux);
}

static void tick_body(void *unused)
{
    (void)unused;
    if(!atomic_load(&active))return;
    int64_t now=esp_timer_get_time();
    if(!atomic_load(&continuous) && now-started>
#if ULL_LIVE_USB
       310000000
#else
       3000000
#endif
       ){ull_trial_pump_finish(3);return;}
    if(atomic_load_explicit(&pending,memory_order_acquire)){
        if(now-pending_since>20000)ull_trial_pump_finish(2);
        return;
    }
    /* Timer task submits a normal internal HCI message. Only its controller
     * handler touches the trial state or the radio scheduler. */
    uint8_t command[4]={1,0xfe,0xfc,0};
    pending_since=now;
    atomic_store_explicit(&pending,true,memory_order_release);
    atomic_fetch_add(&requests,1);
    if(!ull_hci_try_send(command,sizeof(command))){
        atomic_fetch_sub(&requests,1);atomic_store(&pending,false);
    }
}

static void tick(void *unused)
{
    atomic_fetch_add(&tick_running,1);
    tick_body(unused);
    atomic_fetch_sub(&tick_running,1);
}

bool ull_trial_pump_rearm(void)
{
    if(ull_trial_pump_busy())return false;
    portENTER_CRITICAL_SAFE(&trace_mux);
    bool ok=!recording && !ull_trial_pump_busy();
    if(ok){frozen=false;trace_used=trace_read=trace_lost=0;}
    portEXIT_CRITICAL_SAFE(&trace_mux);
    return ok;
}

void ull_trial_pump_init(void)
{
    const esp_timer_create_args_t args={.callback=tick,.name="ull_trial_tick",
        .dispatch_method=ESP_TIMER_TASK,.skip_unhandled_events=true};
    ESP_ERROR_CHECK(esp_timer_create(&args,&timer));
}

bool ull_trial_pump_start(void)
{
    if(!timer || ull_trial_pump_busy())return false;
    portENTER_CRITICAL_SAFE(&trace_mux);
    bool unavailable=recording || frozen;
    if(!unavailable){trace_used=trace_read=trace_lost=0;trace_base=esp_timer_get_time();}
    portEXIT_CRITICAL_SAFE(&trace_mux);
    if(unavailable)return false;
    atomic_store(&requests,0);atomic_store(&steps,0);atomic_store(&max_dispatch_us,0);
    summary(1,0); /* Public pump capability before buffering; AA/key already emitted. */
    portENTER_CRITICAL_SAFE(&trace_mux);recording=true;portEXIT_CRITICAL_SAFE(&trace_mux);
    started=esp_timer_get_time();atomic_store(&active,true);
    if(esp_timer_start_periodic(timer,PERIOD_US)!=ESP_OK){
        ull_trial_pump_finish(1);return false;
    }
    return true;
}

bool ull_trial_pump_busy(void){
    /* Read callback ownership before pending: an old callback may publish a
     * command and retire between these loads. Once inactive, callbacks that
     * enter later cannot publish. Sequentially consistent loads preserve this
     * boundary; reading pending first could miss that final publication. */
    return atomic_load(&active)||atomic_load(&tick_running)||atomic_load(&pending);
}

bool ull_trial_pump_claim(void)
{
    if(!atomic_load(&active) || !atomic_load_explicit(&pending,memory_order_acquire))return false;
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-pending_since);
    if(elapsed>atomic_load(&max_dispatch_us))atomic_store(&max_dispatch_us,elapsed);
    atomic_fetch_add(&steps,1);return true;
}

bool ull_trial_pump_receive(const uint8_t *data,uint16_t length)
{
    if(length!=7 || data[0]!=4 || data[1]!=0x0e || data[2]!=4 ||
       data[4]!=0xfe || data[5]!=0xfc)return false;
    atomic_store_explicit(&pending,false,memory_order_release);
    if(data[6])ull_trial_pump_finish(data[6]);
    return true; /* Internal completions do not enter the USB logging queue. */
}
