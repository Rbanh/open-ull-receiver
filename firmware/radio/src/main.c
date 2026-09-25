// Autonomous ULL control keeper with an explicitly selected manual HCI bridge.
// No audio profiles, new bonding, filesystem flashing or eFuse writes.
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include "esp_bt.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "auto_link.h"
#include "raw_llcp.h"
#include "trial_pump.h"
#include "tone_trial.h"
#include "usb_audio.h"
#include "usb_controls.h"
#include "usb_recovery.h"
#include "pcm_stream.h"
#include "mic_stream.h"
#include "control_log.h"
#include "status_probe.h"
#include "radio_tx.h"
bool ull_usb_bootloader_allowed(void){return !ull_trial_pump_busy();}
static void usb_diagnostics(uint16_t page,uint32_t out[15]){
    if(page==0){
        uint32_t stream[8],delivery[8],retry[8],recovery[4];
        ull_tone_trial_stream_state(stream);
        ull_tone_trial_delivery(delivery);
        ull_tone_trial_retry_auto_status(retry);
        ull_tone_trial_retry_auto_recovery_status(recovery);
        out[0]=stream[0];out[1]=stream[2];out[2]=stream[3];
        out[3]=stream[4];out[4]=stream[5];out[5]=stream[6];
        out[6]=delivery[3];out[7]=delivery[4];out[8]=delivery[5];
        out[9]=retry[3];out[10]=retry[4];out[11]=retry[5];
        out[12]=retry[6];out[13]=retry[7];
        out[14]=(retry[0]?1u:0u)|(retry[1]?2u:0u)|
                (retry[2]?4u:0u)|(recovery[0]?8u:0u)|
                ((recovery[1]&255u)<<8);
    }else if(page==1){
        ull_pcm_stream_diagnostics(out);
        ull_usb_audio_stats_t usb;ull_usb_audio_stats(&usb);
        out[14]=usb.playback_dropped_frames;
    }else if(page==2){
        uint32_t repeat[6];ull_tone_trial_retry_delivery(repeat);
        for(unsigned i=0;i<6;i++)out[i]=repeat[i];
        uint32_t fallback[3];ull_tone_trial_retry_fallback(fallback);
        for(unsigned i=0;i<3;i++)out[6+i]=fallback[i];
        uint32_t errors[2];ull_tone_trial_retry_error_counts(errors);
        out[9]=errors[0];out[10]=errors[1];
    }else if(page==3){
        ull_radio_tx_retry_failure(out);
    }else if(page==4){
        uint32_t cache[16];ull_tone_trial_retry_cache_failure(cache);
        for(unsigned i=0;i<15;i++)out[i]=cache[i];
    }else if(page>=5 && page<=12){
        /* Read-only channel histogram: sent, authenticated reply, stereo ACK. */
        uint32_t channels[37][3];ull_tone_trial_channel_stats(channels);
        unsigned base=(page-5u)*15u;
        for(unsigned i=0;i<15 && base+i<37u*3u;i++)
            out[i]=channels[(base+i)/3u][(base+i)%3u];
    }else if(page==13){
        ull_raw_detached_parent_counters(out);
        ull_tone_trial_control_retry_status(out+9);
    }else if(page==14){
        ull_status_probe_snapshot(out);
    }
}

typedef struct { int64_t us; uint16_t size; uint8_t bytes[512]; } packet_t;
typedef struct {
    int64_t us;
    uint16_t counter, dest;
    uint8_t size, opcode, captured;
    uint8_t bytes[255];
} llcp_trace_t;
static QueueHandle_t llcp_rx;
static volatile bool llcp_trace_enabled;
static volatile uint32_t llcp_drops;
// Passive observation before the stock decoder rejects vendor opcodes. ABI and
// indication offsets verified against this pinned S3 controller build:
// +0 uint16 counter; +2 uint8 length; +4 uint16 buffer id; +8 data pointer.
// Expected E1 can use a guarded proprietary consumer with the same buffer-free
// and coexistence exit. Every other indication goes through the real handler.
extern int __real_r_lld_llcp_rx_ind_handler_hack(uint16_t, const void *, uint16_t, uint16_t);
int __wrap_r_lld_llcp_rx_ind_handler_hack(uint16_t msg, const void *ind,
                                        uint16_t dest, uint16_t src) {
    if (llcp_trace_enabled && llcp_rx && ind) {
        const uint8_t *s = ind;
        const uint8_t *payload;
        memcpy(&payload, s + 8, sizeof(payload));
        if (payload && s[2]) {
            llcp_trace_t t = {.us=esp_timer_get_time(), .dest=dest, .size=s[2]};
            memcpy(&t.counter, s, sizeof(t.counter));
            t.opcode=payload[0];
            // Standard encryption PDUs are metadata-only. Vendor payloads go
            // only into the private host capture, with bounded actual length.
            if (t.opcode == 0x07 || t.opcode==0x0d || t.opcode==0x11 || (t.opcode >= 0xe0 && t.opcode <= 0xef && t.opcode != 0xe2) ||
                (t.size==9 && (t.opcode==8 || t.opcode==9 || t.opcode==14 || t.opcode==20 || t.opcode==21))) {
                t.captured=t.size;
                memcpy(t.bytes,payload,t.captured);
            }
            if (xQueueSend(llcp_rx,&t,0)!=pdTRUE) llcp_drops++;
        }
    }
    if(ull_raw_receive_e1(ind,dest))return 0;
    return __real_r_lld_llcp_rx_ind_handler_hack(msg,ind,dest,src);
}
// Link-time wrapper around the SDK's existing feature getter. No ROM writes.
// Original eight-byte feature bitmap is guarded against the verified build.
extern const uint8_t *__real_r_llm_le_features_get_hack(void);
static volatile bool ull_feature_enabled;
void ull_feature_set(bool enabled){ull_feature_enabled=enabled;}
static DRAM_ATTR uint8_t ull_features[8];
const uint8_t *__wrap_r_llm_le_features_get_hack(void) {
    const uint8_t *source=__real_r_llm_le_features_get_hack();
    static const uint8_t expected[8]={0xff,0xf9,0x01,0x08,0,0,0,0};
    if(!ull_feature_enabled || memcmp(source,expected,8)!=0)return source;
    memcpy(ull_features,source,8);ull_features[7]|=0x40;return ull_features;
}
static QueueHandle_t rx;
static volatile uint32_t drops;
static volatile int64_t last_host;
static int receive(uint8_t *data, uint16_t len) {
    if(ull_trial_pump_receive(data,len))return 0;
    if(ull_auto_receive(data,len))return 0;
    packet_t p;
    if (len > sizeof p.bytes) { drops++; return 0; }
    p.us = esp_timer_get_time(); p.size = len; memcpy(p.bytes, data, len);
    if (xQueueSend(rx, &p, 0) != pdTRUE) drops++;
    return 0;
}
void ull_controller_diag_record(const uint8_t *data, unsigned size) {
    // Local instrumentation event; never transmitted over the radio.
    if(ull_trial_trace_record(data,size))return;
    uint8_t p[106]={4,0xff,0};
    if (size>sizeof(p)-3) return;
    p[2]=size;memcpy(p+3,data,size);receive(p,size+3);
}
static void ready(void) { }
static esp_vhci_host_callback_t callbacks = {ready, receive};
static void print_rx(void *unused) {
    setvbuf(stdout,NULL,_IOLBF,0);
    packet_t p;
    for (;;) {
        bool available=xQueueReceive(rx,&p,0)==pdTRUE;
        unsigned buffered_size=0;
        if(!available && ull_trial_trace_next(p.bytes+3,sizeof(p.bytes)-3,&buffered_size,&p.us)){
            p.bytes[0]=4;p.bytes[1]=0xff;p.bytes[2]=(uint8_t)buffered_size;
            p.size=(uint16_t)(buffered_size+3);available=true;
        }
        if(!available)available=xQueueReceive(rx,&p,pdMS_TO_TICKS(10))==pdTRUE;
        if (available) {
            flockfile(stdout);
            printf("{\"rx\":\"");
            for (unsigned i=0;i<p.size;i++) printf("%02x",p.bytes[i]);
            printf("\",\"us\":%" PRId64 ",\"drops\":%" PRIu32 "}\n",p.us,drops);
            fflush(stdout);
            fsync(fileno(stdout)); // Terminate an exact 64-byte USB transfer too.
            funlockfile(stdout);
        }
        llcp_trace_t t;
        for (unsigned budget=0; budget<8 && xQueueReceive(llcp_rx,&t,0)==pdTRUE; budget++) {
            flockfile(stdout);
            printf("{\"llcp_rx\":{\"opcode\":%u,\"length\":%u,\"dest\":%u,\"counter_candidate\":%u,\"payload\":\"",
                   t.opcode,t.size,t.dest,t.counter);
            for (unsigned i=0;i<t.captured;i++) printf("%02x",t.bytes[i]);
            printf("\"},\"us\":%" PRId64 ",\"llcp_drops\":%" PRIu32 "}\n",t.us,llcp_drops);
            fflush(stdout);fsync(fileno(stdout));
            funlockfile(stdout);
        }
        if (!ull_auto_enabled() && esp_timer_get_time()-last_host > 90000000) {
            printf("{\"watchdog\":\"local_restart\"}\n"); fflush(stdout); esp_restart();
        }
    }
}
static int nibble(char c) {
    if(c>='0'&&c<='9')return c-'0';
    if(c>='a'&&c<='f')return c-'a'+10;
    if(c>='A'&&c<='F')return c-'A'+10;
    return -1;
}
void app_main(void) {
    ESP_ERROR_CHECK(ull_usb_audio_init());
    ull_usb_controls_set_diagnostics(usb_diagnostics);
    setvbuf(stdout,NULL,_IOLBF,0);
    // Do not erase NVS on error; the spare board has a preserved backup.
    ESP_ERROR_CHECK(nvs_flash_init());
    rx=xQueueCreate(16,sizeof(packet_t)); configASSERT(rx);
    llcp_rx=xQueueCreate(32,sizeof(llcp_trace_t)); configASSERT(llcp_rx);
    ull_trial_pump_init();
    esp_bt_controller_config_t cfg=BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&callbacks));
    last_host=esp_timer_get_time();
    ull_auto_init();
    configASSERT(ull_pcm_stream_init());
    if(!ull_mic_stream_init())printf("{\"dsp_task_failed\":true}\n");
    xTaskCreate(print_rx,"rx_print",4096,NULL,5,NULL);
    printf("{\"firmware\":\"blackshark-ull-link-0.24-retry27ctrlslot\",\"radio\":\"autonomous_keeper\"}\n");
    fflush(stdout);fsync(fileno(stdout));
    static char line[2050]; unsigned used=0; bool overflow=false; static uint8_t data[1024];
    for (;;) {
        int c=getchar();
        if(c==EOF){clearerr(stdin);vTaskDelay(pdMS_TO_TICKS(1));continue;}
        if(c=='\r')continue;
        if(c!='\n'){
            if(used<sizeof(line)-1)line[used++]=(char)c;else overflow=true;
            continue;
        }
        last_host=esp_timer_get_time();
        line[used]=0;
        if(!overflow && used==1 && line[0]=='?') {
            printf("{\"firmware\":\"blackshark-ull-link-0.24-retry27ctrlslot\",\"owner\":\"%s\",\"auto_state\":\"%s\",\"drops\":%" PRIu32 ",\"llcp_drops\":%" PRIu32 "}\n",
                   ull_auto_enabled()?"autonomous":"manual",ull_auto_state(),drops,llcp_drops);
        } else if(!overflow && used==1 && line[0]=='u') {
            ull_usb_audio_print_stats();
        } else if(!overflow && (!strcmp(line,"mic0") || !strcmp(line,"mic1"))) {
            ull_mic_stream_enable(line[3]=='1');
            printf("{\"mic_enabled\":%s}\n",line[3]=='1'?"true":"false");
        } else if(!overflow && (!strcmp(line,"controls1") || !strcmp(line,"controls0"))) {
            ull_control_log_enable(line[8]=='1');ull_control_log_status();
        } else if(!overflow && !strcmp(line,"parentstatus")) {
            ull_raw_detached_parent_status();
        } else if(!overflow && (!strcmp(line,"controlprobe") || !strcmp(line,"controlprobe1") || !strcmp(line,"controlprobe0"))) {
            if(strcmp(line,"controlprobe"))ull_raw_control_probe_set(line[12]=='1');
            printf("{\"control_probe\":{\"remaining_frames\":%u}}\n",ull_raw_control_probe_remaining());
        } else if(!overflow && (!strcmp(line,"controlauto") || !strcmp(line,"controlauto1") || !strcmp(line,"controlauto0"))) {
            if(strcmp(line,"controlauto"))ull_raw_control_auto_set(line[11]=='1');
            printf("{\"control_auto\":%s}\n",ull_raw_control_auto_get()?"true":"false");
        } else if(!overflow && !strcmp(line,"controlsstatus")) {
            ull_control_log_status();
        } else if(!overflow && !strcmp(line,"controlsdrain")) {
            ull_control_log_drain();
        } else if(!overflow && strcmp(line,"pcm")==0) {
            ull_pcm_stream_stats();
            ull_mic_stream_stats();
            ull_usb_audio_print_level();
        } else if(!overflow && (!strcmp(line,"rfpower") || !strcmp(line,"rfpower9") || !strcmp(line,"rfpower15"))) {
            bool setting=strcmp(line,"rfpower")!=0;
            esp_err_t result=ESP_OK;
            int requested=!strcmp(line,"rfpower15")?15:9;
            if(setting)result=esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT,0,
                requested==15?ESP_PWR_LVL_P15:ESP_PWR_LVL_P9);
            esp_power_level_t level=esp_ble_tx_power_get_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT,0);
            uint32_t power[2];ull_radio_tx_power_snapshot(power);
            printf("{\"rfpower\":{\"setting\":%s,\"requested_dbm\":%d,\"set_error\":%d,\"default_level\":%d,\"default_valid\":%s,\"raw_index\":%u,\"events\":%u,\"observed\":%s}}\n",
                setting?"true":"false",setting?requested:0,(int)result,(int)level,
                level!=ESP_PWR_LVL_INVALID?"true":"false",(unsigned)power[0],(unsigned)power[1],power[1]?"true":"false");
        } else if(!overflow && (!strcmp(line,"retryauto") || !strcmp(line,"retryauto0") || !strcmp(line,"retryauto1"))) {
            if(strcmp(line,"retryauto"))ull_tone_trial_retry_auto_enable(line[9]=='1');
            uint32_t status[8],recovery[4];ull_tone_trial_retry_auto_status(status);
            ull_tone_trial_retry_auto_recovery_status(recovery);
            printf("{\"retry_auto\":{\"enabled\":%s,\"active\":%s,\"session_blocked\":%s,\"attempted\":%u,\"completed\":%u,\"failures\":%u,\"skips\":%u,\"warmup\":%u,\"cooldown\":%s,\"stable_frames\":%u,\"recoveries\":%u,\"cooldown_remaining_ms\":%u}}\n",
                status[0]?"true":"false",status[1]?"true":"false",status[2]?"true":"false",
                (unsigned)status[3],(unsigned)status[4],(unsigned)status[5],(unsigned)status[6],(unsigned)status[7],
                recovery[0]?"true":"false",(unsigned)recovery[1],(unsigned)recovery[2],(unsigned)recovery[3]);
        } else if(!overflow && !strcmp(line,"retrycooldownprobe")) {
            bool armed=ull_tone_trial_retry_cooldown_probe_arm();
            printf("{\"retry_cooldown_probe\":{\"armed\":%s}}\n",armed?"true":"false");
        } else if(!overflow && !strcmp(line,"retrycachefailure")) {
            uint32_t cache[16];ull_tone_trial_retry_cache_failure(cache);
            flockfile(stdout);printf("{\"retry_cache_failure\":[");
            for(unsigned i=0;i<16;i++)printf("%s%u",i?",":"",(unsigned)cache[i]);
            printf("]}\n");funlockfile(stdout);
        } else if(!overflow && !strcmp(line,"retryfailure")) {
            uint32_t failed[10];ull_radio_tx_retry_failure(failed);
            flockfile(stdout);printf("{\"retry_failure\":[");
            for(unsigned i=0;i<10;i++)printf("%s%u",i?",":"",(unsigned)failed[i]);
            printf("]}\n");funlockfile(stdout);
        } else if(!overflow && (!strcmp(line,"retryburst") || !strcmp(line,"retryburststatus"))) {
            bool armed=!strcmp(line,"retryburst") && ull_tone_trial_retry_burst_arm();
            uint32_t burst[6];ull_tone_trial_retry_burst_status(burst);
            printf("{\"retry_burst\":{\"armed\":%s,\"state\":%u,\"reason\":%u,\"attempted\":%u,\"completed\":%u,\"submitfail\":%u,\"skips\":%u}}\n",
                armed?"true":"false",(unsigned)burst[0],(unsigned)burst[1],(unsigned)burst[2],(unsigned)burst[3],(unsigned)burst[4],(unsigned)burst[5]);
        } else if(!overflow && (!strcmp(line,"retryprobe") || !strcmp(line,"retrystatus"))) {
            bool armed=!strcmp(line,"retryprobe") && ull_tone_trial_retry_probe_arm();
            uint32_t result[17],trace[160],attempts[64],insert[8];
            ull_tone_trial_retry_probe_status(result);ull_radio_tx_retry_snapshot(trace);ull_tone_trial_retry_attempts(attempts);ull_radio_tx_insert_snapshot(insert);
            flockfile(stdout);
            printf("{\"retry_probe\":{\"armed\":%s,\"result\":[",armed?"true":"false");
            for(unsigned i=0;i<17;i++)printf("%s%u",i?",":"",(unsigned)result[i]);
            printf("],\"trace\":[");
            for(unsigned i=0;i<160;i++)printf("%s%u",i?",":"",(unsigned)trace[i]);
            printf("],\"attempts\":[");
            for(unsigned i=0;i<64;i++)printf("%s%u",i?",":"",(unsigned)attempts[i]);
            printf("],\"insert\":[");
            for(unsigned i=0;i<8;i++)printf("%s%u",i?",":"",(unsigned)insert[i]);
            printf("]}}\n");funlockfile(stdout);
        } else if(!overflow && strcmp(line,"rfmap")==0) {
            uint32_t channels[37][3];ull_tone_trial_channel_stats(channels);
            flockfile(stdout);
            printf("{\"rf_channels\":[");
            for(unsigned ch=0;ch<37;ch++)printf("%s[%u,%u,%u]",ch?",":"",
                (unsigned)channels[ch][0],(unsigned)channels[ch][1],(unsigned)channels[ch][2]);
            printf("]}\n");funlockfile(stdout);
        } else if(!overflow && strcmp(line,"memory")==0) {
            if(ull_trial_pump_busy())printf("{\"error\":\"trial_active\"}\n");
            else {
                const uint32_t caps=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;
                printf("{\"memory\":{\"internal_free_bytes\":%u,\"internal_largest_block_bytes\":%u,\"internal_minimum_free_bytes\":%u}}\n",
                       (unsigned)heap_caps_get_free_size(caps),
                       (unsigned)heap_caps_get_largest_free_block(caps),
                       (unsigned)heap_caps_get_minimum_free_size(caps));
            }
        } else if(!overflow && strcmp(line,"bootloader")==0) {
            if(ull_usb_bootloader_allowed()) {
                printf("{\"restarting_to_bootloader\":true}\n");fflush(stdout);fsync(fileno(stdout));
                vTaskDelay(pdMS_TO_TICKS(100));ull_usb_enter_bootloader();
            } else printf("{\"error\":\"trial_active\"}\n");
        } else if(!overflow && used==1 && line[0]=='z' && !ull_auto_enabled()) {
            // Full spare-board restart is the recovery boundary for a custom
            // radio ownership fault. Do not reset only HCI with an owned event.
            printf("{\"restarting_to_keeper\":true}\n");fflush(stdout);
            esp_restart();
        } else if(!overflow && used==1 && (line[0]=='h'||line[0]=='a')) {
            if(line[0]=='a' && ull_trial_pump_busy()){
                printf("{\"error\":\"trial_active\"}\n");used=0;overflow=false;continue;
            }
            ull_auto_enable(line[0]=='a');
            printf("{\"owner\":\"%s\"}\n",ull_auto_enabled()?"autonomous":"manual");
        } else if(!overflow && used==1 && (line[0]=='l'||line[0]=='m')) {
            llcp_trace_enabled=line[0]=='l';
            printf("{\"llcp_trace_enabled\":%s}\n",llcp_trace_enabled?"true":"false");
        } else if(!overflow && used==1 && (line[0]=='f'||line[0]=='g')) {
            ull_feature_enabled=line[0]=='f';
            printf("{\"ull_feature_requested\":%s}\n",ull_feature_enabled?"true":"false");
        } else if(!overflow && used>=8 && used%2==0) {
            if(ull_auto_enabled()) {
                printf("{\"error\":\"manual_takeover_required\"}\n");
                fflush(stdout);fsync(fileno(stdout));used=0;overflow=false;continue;
            }
            unsigned n=used/2; bool valid=true;
            for(unsigned i=0;i<n;i++){
                int a=nibble(line[2*i]),b=nibble(line[2*i+1]);
                if(a<0||b<0){valid=false;break;}data[i]=(a<<4)|b;
            }
            valid=valid && ((data[0]==1 && n==4u+data[3]) ||
                (data[0]==2 && n>=5 && n==5u+data[3]+256u*data[4]));
            if(valid){
                uint16_t opcode=data[0]==1 ? (uint16_t)(data[1]|(uint16_t)data[2]<<8) : 0;
                if(opcode==0xfcfe || (ull_trial_pump_busy() && opcode && opcode!=0xfcfc && opcode!=0xfcfd)){
                    printf("{\"error\":\"trial_command_guard\"}\n");used=0;overflow=false;continue;
                }
                if(n==4 && memcmp(data,"\x01\x03\x0c\x00",4)==0) {
                    ull_feature_enabled=false;llcp_trace_enabled=false;
                    ull_raw_reset_sequence();
                }
                int tries=200;
                while(!ull_hci_try_send(data,n)&&--tries)vTaskDelay(pdMS_TO_TICKS(10));
                if(!tries)printf("{\"error\":\"tx_timeout\"}\n");
            }else printf("{\"error\":\"invalid_packet\"}\n");
        } else printf("{\"error\":\"invalid_line\"}\n");
        fflush(stdout);fsync(fileno(stdout));
        used=0;overflow=false;
    }
}
