#include "control_log.h"
#include "mic_stream.h"
// Autonomous saved-bond control and verified USB audio startup. No bond writes.
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdatomic.h>
#include "esp_bt.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "tone_trial.h"
#include "pcm_stream.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "auto_link.h"
#include "trial_pump.h"
#include "raw_llcp.h"
#include "bond.h"
#include "device.h"

typedef struct {uint16_t length;uint8_t data[512];} event_t;
static QueueHandle_t events;
static atomic_bool enabled=true, busy=false, lost=false, pause_requested=false;
static const char *state="starting";
static struct {
    uint16_t handle, acl_limit, assembled;
    uint8_t fragment[4096];
    bool encrypted, encryption_requested, setup, peer_verified, idle_sent, holding, disconnected, failed;
    int64_t connected_at, next_action, last_feedback;
    uint32_t previous_feedback;
    uint16_t max_tx_octets;
    uint8_t raw_guard, raw_status, raw_tries, startup_step;
    bool activity_valid, raw_acked, air_ready, started, stream_attempted;
    uint8_t activity[30], control_slot;
    bool diagnostic_ok;

} peer;
/* Playback-quality 45s capture: channels17/18/19/28 had88/64/46/51%
 * stereo ACKs. Preserve23channels, negotiate the same map through the
 * existing host classification and verified session setup. */
static const uint8_t clean_map[5]={0xff,0xff,0x01,0xe0,0x13};
static bool command(uint16_t,const uint8_t *,unsigned,uint8_t *,unsigned *);
static uint16_t le16(const uint8_t *p){return p[0]|(uint16_t)p[1]<<8;}
static void put16(uint8_t *p,uint16_t v){p[0]=v;p[1]=v>>8;}
bool ull_auto_enabled(void){return atomic_load(&enabled);}
const char *ull_auto_state(void){return __atomic_load_n(&state,__ATOMIC_ACQUIRE);}
static void set_state(const char *s){
    if(!strcmp(ull_auto_state(),s))return;
    __atomic_store_n(&state,s,__ATOMIC_RELEASE);
    printf("{\"auto_state\":\"%s\",\"audio_transport\":%s}\n",s,peer.started?"true":"false");
    fflush(stdout);fsync(fileno(stdout));
}
void ull_auto_enable(bool value){
    if(!value){
        atomic_store(&pause_requested,true);
        while(atomic_load(&busy))vTaskDelay(pdMS_TO_TICKS(10));
        atomic_store(&enabled,false);set_state("paused_for_manual_session");
    }else{
        atomic_store(&pause_requested,false);atomic_store(&enabled,true);
    }
}

bool ull_auto_receive(const uint8_t *data,uint16_t length){
    if(!ull_auto_enabled())return false;
    event_t p;
    if(!events || length>sizeof(p.data)){atomic_store(&lost,true);return true;}
    p.length=length;memcpy(p.data,data,length);
    if(xQueueSend(events,&p,0)!=pdTRUE)atomic_store(&lost,true);
    return true;
}
static bool send_packet(uint8_t *p,unsigned n){
    for(unsigned i=0;i<200 && ull_auto_enabled();i++){
        if(ull_hci_try_send(p,n))return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}
static bool send_acl(uint16_t cid,const uint8_t *payload,unsigned size){
    uint8_t body[300],packet[305];
    if(peer.handle==0xffff || size>sizeof(body)-4)return false;
    put16(body,size);put16(body+2,cid);memcpy(body+4,payload,size);
    for(unsigned offset=0;offset<size+4;){
        unsigned n=size+4-offset;if(n>peer.acl_limit)n=peer.acl_limit;
        packet[0]=2;put16(packet+1,peer.handle|(offset?0x1000:0));put16(packet+3,n);
        memcpy(packet+5,body+offset,n);
        if(!send_packet(packet,n+5))return false;
        offset+=n;
    }
    return true;
}
static void l2cap(uint16_t cid,const uint8_t *p,unsigned n){
    if(cid==0x101){
        if(!peer.encrypted){peer.failed=true;return;}
        if(n==45 && p[0]==14 && p[1]==2){
            static const uint8_t mode[12]={0,0,0,0,0,0x77,1,0,0,0x7d,0,0};
            if(p[2]!=1 || p[3]!=1 || memcmp(p+4,saved_sirk,16) || memcmp(p+25,mode,12)){
                peer.failed=true;set_state("peer_configuration_mismatch");return;
            }
            peer.peer_verified=true;set_state("proprietary_peer_verified");
        }else if(n==9 && p[0]==17 && !peer.idle_sent){
            static const uint8_t expected[9]={17,0,0,0,0,3,0,0,0};
            uint8_t reply[9]={17,1,0,0,0,3,0,0,0}, idle[281]={15};
            if(!peer.peer_verified || memcmp(p,expected,9)){peer.failed=true;return;}
            peer.idle_sent=true;
            if(!send_acl(0x101,reply,9) || !send_acl(0x101,idle,281))peer.failed=true;
        }else if(n==281 && p[0]==15 && peer.peer_verified && peer.idle_sent){
            peer.holding=true;set_state("holding_proprietary_connection");
        }else ull_control_log_acl(p,n);
        return;
    }
    if(cid==4 && n==3 && p[0]==2){uint8_t reply[3]={3,23,0};send_acl(4,reply,3);}
    else if(cid==4 && n>=7 && p[0]==6){uint8_t reply[5]={1,6,p[1],p[2],10};send_acl(4,reply,5);}
    else if(cid==6 && n && p[0]==1){uint8_t reply[2]={5,5};send_acl(6,reply,2);}
    else if(cid==5 && n>=4 && p[0]==0x12){uint8_t reply[6]={0x13,p[1],2,0,1,0};send_acl(5,reply,6);}
}
static void process(const uint8_t *p,unsigned n){
    if(n<3)return;
    if(p[0]==4){
        if(n!=3u+p[2]){peer.failed=true;return;}
        const uint8_t *d=p+3;unsigned len=p[2];
        if(p[1]==0x3e && len>=12 && (d[0]==1 || d[0]==10)){
            if(d[1]){peer.disconnected=true;return;}
            if(memcmp(d+6,target,6)){peer.failed=true;return;}
            peer.handle=le16(d+2);peer.connected_at=esp_timer_get_time();set_state("connected_encrypting");
        }else if(p[1]==5 && len>=4){
            /* The headset retires the original ACL after the Air handoff.
             * Continued authenticated Air replies establish liveness then. */
            if(peer.started && !d[0] && peer.handle!=0xffff && le16(d+1)==peer.handle)
                ull_raw_parent_retired();
            if(!peer.started)peer.disconnected=true;
            peer.handle=0xffff;
        }else if(p[1]==0x3e && len==11 && d[0]==7){peer.max_tx_octets=le16(d+3);}
        else if(p[1]==0xff){
            if(len==38 && !memcmp(d,"ULLC\1",5)){
                peer.activity_valid=!d[5] && d[6]==6;peer.control_slot=d[7];
                memcpy(peer.activity,d+8,30);
            }else if(len==12 && !memcmp(d,"ULLD\1",5)){
                peer.diagnostic_ok=!d[6] && d[7] && le16(d+8)==27 && le16(d+10)==513;
            }else if(len==12 && !memcmp(d,"ULLT\1",5)){
                if(d[5]==1){peer.raw_guard=d[8];peer.raw_status=d[9];}
                if(d[5]==2 && d[6]==0xe4){
                    if(d[8]!=7 || d[9]!=1)peer.failed=true;else peer.raw_acked=true;
                }
            }else if(len==16 && !memcmp(d,"ULLR\1\1",6))peer.air_ready=true;
            else if(len==28 && !memcmp(d,"ULLP\3",5) && d[5]==3)peer.failed=true;
        }
        else if(p[1]==8 && len>=4){
            peer.encrypted=d[0]==0 && d[3]!=0;if(!peer.encrypted)peer.failed=true;
        }else if(p[1]==0x3e && len>=3 && d[0]==5){
            uint8_t reject[6]={1,0x1b,0x20,2,d[1],d[2]};send_packet(reject,sizeof(reject));
        }
        return;
    }
    if(p[0]!=2 || n<5 || le16(p+3)!=n-5)return;
    uint16_t flags=le16(p+1);unsigned pb=(flags>>12)&3;
    if((flags&0xfff)!=peer.handle)return;
    if(pb==0 || pb==2)peer.assembled=0;
    else if(pb!=1 || !peer.assembled)return;
    if(peer.assembled+n-5>sizeof(peer.fragment)){peer.failed=true;return;}
    memcpy(peer.fragment+peer.assembled,p+5,n-5);peer.assembled+=n-5;
    if(peer.assembled<4)return;
    unsigned expected=4u+le16(peer.fragment);
    if(expected>sizeof(peer.fragment) || peer.assembled>expected){peer.failed=true;return;}
    if(peer.assembled==expected){
        l2cap(le16(peer.fragment+2),peer.fragment+4,expected-4);peer.assembled=0;
    }
}
static bool command(uint16_t opcode,const uint8_t *params,unsigned size,uint8_t *reply,unsigned *reply_size){
    uint8_t p[64]={1};if(size>sizeof(p)-4)return false;
    if(opcode==0x0c03)ull_raw_reset_sequence();
    put16(p+1,opcode);p[3]=size;if(size)memcpy(p+4,params,size);
    if(!send_packet(p,size+4))return false;
    int64_t until=esp_timer_get_time()+5000000;
    event_t e;
    while(ull_auto_enabled() && esp_timer_get_time()<until){
        if(xQueueReceive(events,&e,pdMS_TO_TICKS(50))!=pdTRUE)continue;
        uint8_t *b=e.data;
        if(e.length>=7 && b[0]==4 && e.length==3u+b[2]){
            if(b[1]==14 && le16(b+4)==opcode){
                unsigned n=e.length-6;
                if(reply && reply_size){if(n>*reply_size)return false;memcpy(reply,b+6,n);*reply_size=n;}
                return b[6]==0;
            }
            if(b[1]==15 && le16(b+5)==opcode)return b[3]==0;
        }
        if(opcode!=0x0c03)process(b,e.length);
        if(atomic_load(&lost))return false;
    }
    return false;
}
#define CMD(op,p,n) command(op,p,n,NULL,NULL)
static bool start_connection(void){
    ull_mic_stream_new_session();
    memset(&peer,0,sizeof(peer));peer.handle=0xffff;peer.acl_limit=27;
    xQueueReset(events);atomic_store(&lost,false);ull_feature_set(false);
    if(!CMD(0x0c03,NULL,0))return false;
    uint8_t reply[16];unsigned n=sizeof(reply);
    if(!command(0x2002,NULL,0,reply,&n) || n!=4 || le16(reply+1)<27 || !reply[3])return false;
    peer.acl_limit=le16(reply+1);if(peer.acl_limit>251)peer.acl_limit=251;
    ull_feature_set(true);n=sizeof(reply);
    static const uint8_t features[9]={0,0xff,0xf9,1,8,0,0,0,0x40};
    if(!command(0x2003,NULL,0,reply,&n) || n!=9 || memcmp(reply,features,9))return false;
    const uint8_t mask[8]={255,255,255,255,255,255,255,63}, lemask[8]={255,255,31,0,0,0,0,0};
    if(!CMD(0x0c01,mask,8) || !CMD(0x2001,lemask,8) || !CMD(0x2005,identity,6) || !CMD(0x2014,clean_map,5))return false;
    uint8_t params[26]={0,1,0};memcpy(params+3,target,6);params[9]=1;
    const uint16_t timing[8]={16,16,24,24,0,500,2,2};
    for(unsigned i=0;i<8;i++)put16(params+10+i*2,timing[i]);
    if(!CMD(0x2043,params,sizeof(params)))return false;
    set_state("searching_for_headset");return true;
}
/* Same command order and checks as the successful host-owned startup. */
static bool verify_map(void){
    uint8_t params[2],reply[8];unsigned n=sizeof(reply);put16(params,peer.handle);
    return command(0x2015,params,2,reply,&n) && n==8 && le16(reply+1)==peer.handle && !memcmp(reply+3,clean_map,5);
}
static bool reserve_audio(void){
    peer.activity_valid=false;
    if(!CMD(0xfcfa,NULL,0) || !peer.activity_valid || peer.control_slot!=0)return false;
    uint8_t expected[30];memcpy(expected,peer.activity,30);
    const uint8_t unused[5]={0,0,255,0,0};
    if(memcmp(expected+5,unused,5) || memcmp(expected+10,unused,5) || memcmp(expected+15,unused,5))return false;
    uint8_t params[25]={0xee,0,0,160,0,0,160,0,0,7,0,0,0,0,0,0,0,0,0,127,1,0,1,0,0};
    for(unsigned slot=1;slot<=3;slot++){
        params[0]=slot==1?0xee:slot==2?0xef:0xed;
        peer.activity_valid=false;
        if(!CMD(0x2036,params,25) || !CMD(0xfcfa,NULL,0) || !peer.activity_valid || peer.control_slot!=0)return false;
        uint8_t reserved[5]={1,1,params[0],0,0};memcpy(expected+slot*5,reserved,5);
        if(memcmp(peer.activity,expected,30))return false;
    }
    return true;
}
static bool audio_startup(int64_t now){
    if(now<peer.next_action)return true;
    switch(peer.startup_step){
    case 0:{
        if(!ull_pcm_stream_source())return true;
        if(!verify_map() || !CMD(0xfcf0,NULL,0) || !peer.diagnostic_ok || !reserve_audio())return false;
        const uint8_t open[3]={2,1,0},volume[6]={4,1,0,100,100,0};
        if(!send_acl(0x101,open,3) || !send_acl(0x101,volume,6))return false;
        /* Open the microphone stream; USB capture activity controls decoding. */
        const uint8_t mic_open[3]={2,2,0};
        if(!send_acl(0x101,mic_open,3))return false;
        uint8_t params[6];put16(params,peer.handle);put16(params+2,251);put16(params+4,2120);
        if(!CMD(0x2022,params,6))return false;
        peer.startup_step=1;set_state("negotiating_audio");return true;
    }
    case 1:
        if(peer.max_tx_octets<182)return true;
        peer.raw_guard=peer.raw_status=255;
        if(!CMD(0xfcf3,NULL,0)){
            if(peer.raw_guard!=4 || ++peer.raw_tries>=12)return false;
        }else{peer.startup_step=2;peer.raw_tries=0;}
        peer.next_action=now+250000;return true;
    case 2:
        if(!peer.raw_acked)return true;
        peer.raw_guard=peer.raw_status=255;
        if(!CMD(0xfcf4,NULL,0)){
            if(peer.raw_guard!=4 || ++peer.raw_tries>=12)return false;
        }else{peer.startup_step=3;peer.raw_tries=0;}
        peer.next_action=now+1000;return true;
    case 3:{
        if(!peer.air_ready)return true;
        if(!verify_map() || !ull_tone_trial_continuous(true) || !ull_trial_pump_continuous(true))return false;
        peer.stream_attempted=true;
        if(!CMD(0xfcfb,NULL,0)){
            if(++peer.raw_tries>=10)return false;
            peer.next_action=now+1000;return true;
        }
        peer.started=true;
        const uint8_t unmute[3]={6,1,0};
        if(!send_acl(0x101,unmute,3))return false;
        const uint8_t mic_unmute[3]={6,2,0};
        if(!send_acl(0x101,mic_unmute,3))return false;
        peer.last_feedback=esp_timer_get_time();
        set_state("streaming_usb_audio");return true;
    }
    default:return false;
    }
}
static bool audio_live(int64_t now){
    uint32_t stream[8];ull_tone_trial_stream_state(stream);
    if(stream[5]!=peer.previous_feedback){peer.previous_feedback=stream[5];peer.last_feedback=now;}
    return ull_trial_pump_busy() && now-peer.last_feedback<=5000000;
}
/* Failure-only metadata, before the existing full-reset safety boundary. */
static bool stop_audio_failed(unsigned stage){
    uint32_t stream[8];ull_tone_trial_stream_state(stream);
    printf("{\"shutdown_failure\":{\"stage\":%u,\"phase\":%u,\"reason\":%u,\"pump_busy\":%s}}\n",
        stage,(unsigned)stream[0],(unsigned)stream[1],ull_trial_pump_busy()?"true":"false");
    fflush(stdout);return false;
}
static bool stop_audio(void){
    if(!peer.stream_attempted)return true;
    /* Even a stopped/expired pump can leave a radio event awaiting collection.
     * Drive cancellation through the controller dispatcher until terminal. */
    int64_t end=esp_timer_get_time()+500000;
    do {
        if(!CMD(0xfcfd,NULL,0))return stop_audio_failed(1);
        uint32_t stream[8];ull_tone_trial_stream_state(stream);
        if(!ull_trial_pump_busy() && stream[0]>=3)break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }while(esp_timer_get_time()<end);
    uint32_t stream[8];ull_tone_trial_stream_state(stream);
    if(ull_trial_pump_busy() || stream[0]<3 || stream[0]>=6)return stop_audio_failed(2);
    if(!ull_pcm_stream_pause_for_reconnect())return stop_audio_failed(3);
    if(!CMD(0xfcff,NULL,0))return stop_audio_failed(4);
    ull_pcm_stream_resume_after_reconnect();
    return true;
}
static void run(void *unused){
    (void)unused;vTaskDelay(pdMS_TO_TICKS(400));
    for(;;){
        if(!ull_auto_enabled() || atomic_load(&pause_requested)){atomic_store(&busy,false);vTaskDelay(pdMS_TO_TICKS(50));continue;}
        atomic_store(&busy,true);
        if(!ull_auto_enabled() || atomic_load(&pause_requested)){atomic_store(&busy,false);continue;}
        if(!start_connection()){if(ull_auto_enabled())set_state("retrying_controller_setup");goto retry;}
        int64_t started=esp_timer_get_time();
        while(ull_auto_enabled() && !atomic_load(&pause_requested) && !peer.failed && !peer.disconnected && !atomic_load(&lost)){
            event_t e;
            if(xQueueReceive(events,&e,pdMS_TO_TICKS(peer.holding && !peer.started?1:50))==pdTRUE)process(e.data,e.length);
            int64_t now=esp_timer_get_time();
            if(peer.started){
                if(!audio_live(now))peer.failed=true;
                continue;
            }
            if(peer.handle!=0xffff){
                if(!peer.encryption_requested && now-peer.connected_at>=1000000){
                    uint8_t params[28];put16(params,peer.handle);
                    memcpy(params+2,saved_rand,8);memcpy(params+10,saved_ediv,2);memcpy(params+12,saved_ltk,16);
                    peer.encryption_requested=true;
                    if(!CMD(0x2019,params,sizeof(params)))peer.failed=true;
                    memset(params,0,sizeof(params));
                }
                if(peer.encrypted && !peer.setup){
                    uint8_t setup[45]={14,1};peer.setup=true;
                    if(!send_acl(0x101,setup,sizeof(setup)))peer.failed=true;
                }
                if(peer.holding && !audio_startup(now))peer.failed=true;
                if(!peer.started && now-peer.connected_at>20000000)peer.failed=true;
            }else if(now-started>20000000)break;
        }
        if(!stop_audio()){
            set_state("restarting_unclean_audio_session");vTaskDelay(pdMS_TO_TICKS(100));esp_restart();
        }
        if(ull_auto_enabled())set_state("reconnecting");
retry:
        atomic_store(&busy,false);
        for(unsigned i=0;i<10 && ull_auto_enabled() && !atomic_load(&pause_requested);i++)vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void ull_auto_init(void){
    events=xQueueCreate(32,sizeof(event_t));configASSERT(events);
    BaseType_t ok=xTaskCreate(run,"ull_keeper",6144,NULL,6,NULL);configASSERT(ok==pdPASS);
}
