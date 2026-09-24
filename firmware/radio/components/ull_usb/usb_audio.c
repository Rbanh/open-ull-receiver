#include "usb_audio.h"
#include "usb_pcm.h"
#include "usb_feedback.h"
#include "usb_recovery.h"
#include "usb_controls.h"
#include "usb_descriptors.h"
#include "tusb.h"
#include "esp_private/usb_phy.h"
#include "esp_vfs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static portMUX_TYPE audio_lock=portMUX_INITIALIZER_UNLOCKED;
static ull_pcm_ring_t playback,capture;
static ull_usb_audio_stats_t stats;
static int16_t playback_volume,capture_volume;
static bool playback_mute,capture_mute,headset_mic_mute;
static int32_t playback_gain=32768,capture_gain=32768;
static uint32_t playback_nonzero;
bool ull_usb_audio_consumer_key(uint16_t usage){
    return (usage==ULL_USB_PLAY_PAUSE || usage==ULL_USB_VOLUME_UP || usage==ULL_USB_VOLUME_DOWN) &&
           ull_usb_controls_enqueue(usage);
}
void ull_usb_audio_set_headset_mic_mute(bool muted){
    portENTER_CRITICAL(&audio_lock);headset_mic_mute=muted;portEXIT_CRITICAL(&audio_lock);
}
bool ull_usb_audio_headset_mic_muted(void){
    portENTER_CRITICAL(&audio_lock);bool value=headset_mic_mute;portEXIT_CRITICAL(&audio_lock);
    return value;
}
static ull_usb_feedback_t playback_feedback;
static bool feedback_seed_pending;
void ull_usb_audio_print_level(void){
    portENTER_CRITICAL(&audio_lock);
    uint32_t n=playback_nonzero;int32_t gain=playback_gain;bool mute=playback_mute;
    portEXIT_CRITICAL(&audio_lock);
    printf("{\"usb_playback_level\":{\"nonzero_samples\":%u,\"gain_q15\":%ld,\"mute\":%s}}\n",(unsigned)n,(long)gain,mute?"true":"false");
    printf("{\"usb_headset_mic_mute\":%s}\n",ull_usb_audio_headset_mic_muted()?"true":"false");
}
static usb_phy_handle_t phy;
static TaskHandle_t usb_task;
static StreamBufferHandle_t console_rx,console_tx_queue;
static SemaphoreHandle_t usb_ready,usb_stopped;
static esp_err_t usb_init_result;
static volatile bool console_connected;
static SemaphoreHandle_t console_tx;
static bool baud_1200,boot_pending;
static volatile bool shutting_down;

size_t ull_usb_audio_read_playback(int16_t *out,size_t frames){
    if(!out)return 0;
    portENTER_CRITICAL(&audio_lock);
    size_t got=ull_pcm_pop(&playback,out,frames);stats.radio_playback_frames+=got;
    int32_t gain=playback_mute?0:playback_gain;
    portEXIT_CRITICAL(&audio_lock);
    ull_pcm_gain(out,got*2,gain);return got;
}
size_t ull_usb_audio_write_capture(const int16_t *in,size_t frames){
    if(!in)return 0;
    portENTER_CRITICAL(&audio_lock);
    if(!stats.capture_active || !stats.mounted){portEXIT_CRITICAL(&audio_lock);return 0;}
    stats.capture_dropped_frames+=ull_pcm_push(&capture,in,frames);
    stats.radio_capture_frames+=frames;
    portEXIT_CRITICAL(&audio_lock);return frames;
}
void ull_usb_audio_stats(ull_usb_audio_stats_t *out){
    portENTER_CRITICAL(&audio_lock);*out=stats;
    out->playback_buffered_frames=playback.count;out->capture_buffered_frames=capture.count;
    portEXIT_CRITICAL(&audio_lock);
}
void ull_usb_audio_print_stats(void){
    portENTER_CRITICAL(&audio_lock);
    uint32_t feedback_value=playback_feedback.value_q16;bool feedback_active=playback_feedback.active;
    portEXIT_CRITICAL(&audio_lock);
    printf("{\"usb_feedback\":{\"value_q16\":%u,\"target_frames\":960,\"active\":%s}}\n",
        (unsigned)feedback_value,feedback_active?"true":"false");
    ull_usb_audio_stats_t s;ull_usb_audio_stats(&s);flockfile(stdout);
    printf("{\"usb_audio\":{\"mounted\":%s,\"playback_active\":%s,\"capture_active\":%s,"
        "\"rate_hz\":48000,\"playback_channels\":2,\"capture_channels\":1,\"sample_bits\":16,"
        "\"playback_packets\":%"PRIu32",\"playback_frames\":%"PRIu32",\"playback_dropped_frames\":%"PRIu32",\"playback_buffered_frames\":%"PRIu32","
        "\"capture_packets\":%"PRIu32",\"capture_frames\":%"PRIu32",\"capture_underflow_frames\":%"PRIu32",\"capture_buffered_frames\":%"PRIu32",\"capture_dropped_frames\":%"PRIu32","
        "\"radio_playback_frames\":%"PRIu32",\"radio_capture_frames\":%"PRIu32",\"cdc_dropped_bytes\":%"PRIu32",\"malformed_playback_packets\":%"PRIu32",\"usb_task_core\":%"PRIu32",\"usb_irq_core\":%"PRIu32",\"cdc_tx_buffered_bytes\":%"PRIu32",\"cdc_fifo_free\":%"PRIu32"}}\n",
        s.mounted?"true":"false",s.playback_active?"true":"false",s.capture_active?"true":"false",
        s.playback_packets,s.playback_frames,s.playback_dropped_frames,s.playback_buffered_frames,
        s.capture_packets,s.capture_frames,s.capture_underflow_frames,s.capture_buffered_frames,s.capture_dropped_frames,
        s.radio_playback_frames,s.radio_capture_frames,s.cdc_dropped_bytes,s.malformed_playback_packets,
        s.usb_task_core,s.usb_irq_core,s.cdc_tx_buffered_bytes,s.cdc_fifo_free);
    fflush(stdout);fsync(fileno(stdout));funlockfile(stdout);
}
static void reset_streams(bool mounted){
    portENTER_CRITICAL(&audio_lock);stats.mounted=mounted;
    stats.playback_active=false;stats.capture_active=false;
    ull_usb_feedback_reset(&playback_feedback,stats.radio_playback_frames);feedback_seed_pending=false;
    ull_pcm_reset(&playback,2);ull_pcm_reset(&capture,1);portEXIT_CRITICAL(&audio_lock);
}
void tud_mount_cb(void){reset_streams(true);}
void tud_umount_cb(void){console_connected=false;reset_streams(false);}
void tud_suspend_cb(bool remote_wakeup_en){
    (void)remote_wakeup_en;console_connected=false;
    portENTER_CRITICAL(&audio_lock);stats.mounted=false;
    ull_usb_feedback_reset(&playback_feedback,stats.radio_playback_frames);feedback_seed_pending=false;
    ull_pcm_reset(&playback,2);ull_pcm_reset(&capture,1);portEXIT_CRITICAL(&audio_lock);
    // Alternate settings survive bus suspend and resume without SET_INTERFACE.
}
void tud_resume_cb(void){console_connected=tud_cdc_connected();portENTER_CRITICAL(&audio_lock);stats.mounted=true;feedback_seed_pending=stats.playback_active;portEXIT_CRITICAL(&audio_lock);}

bool tud_audio_set_itf_cb(uint8_t rhport,tusb_control_request_t const *request){
    (void)rhport;uint8_t itf=(uint8_t)tu_le16toh(request->wIndex),alt=(uint8_t)tu_le16toh(request->wValue);
    if(alt>1)return false;
    portENTER_CRITICAL(&audio_lock);
    if(itf==ITF_PLAYBACK){ull_pcm_reset(&playback,2);stats.playback_active=alt==1;
        ull_usb_feedback_reset(&playback_feedback,stats.radio_playback_frames);feedback_seed_pending=alt==1;}
    if(itf==ITF_CAPTURE){ull_pcm_reset(&capture,1);stats.capture_active=alt==1;}
    portEXIT_CRITICAL(&audio_lock);return true;
}
bool tud_audio_set_itf_close_EP_cb(uint8_t rhport,tusb_control_request_t const *request){return tud_audio_set_itf_cb(rhport,request);}
bool tud_audio_rx_done_post_read_cb(uint8_t rhport,uint16_t size,uint8_t func,uint8_t ep,uint8_t alt){
    (void)rhport;(void)func;(void)ep;(void)alt;
    int16_t samples[98];uint16_t got=tud_audio_read(samples,sizeof samples);uint32_t feedback=0;
    portENTER_CRITICAL(&audio_lock);stats.playback_packets++;
    if(size!=got || (got%4)){stats.malformed_playback_packets++;}
    else {for(unsigned i=0;i<got/2;i++)playback_nonzero+=samples[i]!=0;
        stats.playback_frames+=got/4;stats.playback_dropped_frames+=ull_pcm_push(&playback,samples,got/4);
        if(stats.mounted && stats.playback_active)
            feedback=ull_usb_feedback_step(&playback_feedback,playback.count,stats.radio_playback_frames);}
    portEXIT_CRITICAL(&audio_lock);if(feedback)tud_audio_fb_set(feedback);return true;
}
void tud_audio_feedback_params_cb(uint8_t func,uint8_t alt,audio_feedback_params_t *p){
    (void)func;(void)alt;memset(p,0,sizeof(*p));
    p->method=AUDIO_FEEDBACK_METHOD_DISABLED;p->sample_freq=48000;
}
static void feedback_service(void){
    portENTER_CRITICAL(&audio_lock);
    bool seed=feedback_seed_pending && stats.mounted && stats.playback_active;
    feedback_seed_pending=false;
    uint32_t value=playback_feedback.value_q16;
    portEXIT_CRITICAL(&audio_lock);
    if(seed)tud_audio_fb_set(value);
}
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport,uint8_t func,uint8_t ep,uint8_t alt){
    (void)rhport;(void)func;(void)ep;(void)alt;
    int16_t samples[48]={0};
    portENTER_CRITICAL(&audio_lock);
    size_t got=ull_pcm_pop(&capture,samples,48);stats.capture_underflow_frames+=48-got;
    int32_t gain=(capture_mute||headset_mic_mute)?0:capture_gain;
    portEXIT_CRITICAL(&audio_lock);ull_pcm_gain(samples,48,gain);
    return tud_audio_write(samples,sizeof samples)==sizeof samples;
}
bool tud_audio_tx_done_post_load_cb(uint8_t rhport,uint16_t bytes,uint8_t func,uint8_t ep,uint8_t alt){
    (void)rhport;(void)func;(void)ep;(void)alt;
    portENTER_CRITICAL(&audio_lock);stats.capture_packets++;stats.capture_frames+=bytes/2;portEXIT_CRITICAL(&audio_lock);return true;
}
static bool reply(uint8_t rhport,const tusb_control_request_t *request,const void *data,uint16_t size){
    return tud_audio_buffer_and_schedule_control_xfer(rhport,request,(void*)data,size);
}
bool tud_audio_get_req_entity_cb(uint8_t rhport,const tusb_control_request_t *req){
    const audio_control_request_t *r=(const audio_control_request_t*)req;
    if(r->bInterface!=ITF_AUDIO_CONTROL || r->bChannelNumber!=0)return false;
    if(r->bEntityID==UAC_CLOCK || r->bEntityID==UAC_CAPTURE_CLOCK){
        if(r->bControlSelector==AUDIO_CS_CTRL_SAM_FREQ){
            if(r->bRequest==AUDIO_CS_REQ_CUR){uint32_t rate=tu_htole32(48000);return reply(rhport,req,&rate,4);}
            if(r->bRequest==AUDIO_CS_REQ_RANGE){audio_control_range_4_n_t(1) range={.wNumSubRanges=tu_htole16(1),.subrange={{48000,48000,0}}};return reply(rhport,req,&range,sizeof range);}
        }
        if(r->bControlSelector==AUDIO_CS_CTRL_CLK_VALID && r->bRequest==AUDIO_CS_REQ_CUR){uint8_t valid=1;return reply(rhport,req,&valid,1);}
        return false;
    }
    bool input=r->bEntityID==UAC_CAPTURE_FEATURE;
    if(!input && r->bEntityID!=UAC_PLAYBACK_FEATURE)return false;
    if(r->bControlSelector==AUDIO_FU_CTRL_MUTE && r->bRequest==AUDIO_CS_REQ_CUR){uint8_t mute=input?(capture_mute||headset_mic_mute):playback_mute;return reply(rhport,req,&mute,1);}
    if(r->bControlSelector==AUDIO_FU_CTRL_VOLUME){
        if(r->bRequest==AUDIO_CS_REQ_CUR){int16_t volume=tu_htole16(input?capture_volume:playback_volume);return reply(rhport,req,&volume,2);}
        if(r->bRequest==AUDIO_CS_REQ_RANGE){audio_control_range_2_n_t(1) range={.wNumSubRanges=tu_htole16(1),.subrange={{(int16_t)tu_htole16(-60*256),0,tu_htole16(256)}}};return reply(rhport,req,&range,sizeof range);}
    }
    return false;
}
bool tud_audio_set_req_entity_cb(uint8_t rhport,const tusb_control_request_t *req,uint8_t *buf){
    (void)rhport;const audio_control_request_t *r=(const audio_control_request_t*)req;
    if(r->bInterface!=ITF_AUDIO_CONTROL || r->bChannelNumber!=0 || r->bRequest!=AUDIO_CS_REQ_CUR)return false;
    // Fixed read-only clock: no unsupported rate can ever be accepted.
    bool input=r->bEntityID==UAC_CAPTURE_FEATURE;
    if(!input && r->bEntityID!=UAC_PLAYBACK_FEATURE)return false;
    if(r->bControlSelector==AUDIO_FU_CTRL_MUTE && tu_le16toh(r->wLength)==1 && buf[0]<=1){
        portENTER_CRITICAL(&audio_lock);if(input)capture_mute=buf[0];else playback_mute=buf[0];portEXIT_CRITICAL(&audio_lock);return true;
    }
    if(r->bControlSelector==AUDIO_FU_CTRL_VOLUME && tu_le16toh(r->wLength)==2){
        int16_t volume=(int16_t)((uint16_t)buf[0]|(uint16_t)buf[1]<<8);
        if(volume>0||volume< -60*256 || volume%256)return false;
        int32_t gain=(int32_t)lroundf(32768.0f*powf(10.0f,(float)volume/(256.0f*20.0f)));
        portENTER_CRITICAL(&audio_lock);if(input){capture_volume=volume;capture_gain=gain;}else{playback_volume=volume;playback_gain=gain;}portEXIT_CRITICAL(&audio_lock);return true;
    }
    return false;
}

// Existing newline-framed diagnostics run over software CDC on the same connector.
// These callbacks and all audio callbacks execute in the USB task, never radio ISR.
void tud_cdc_rx_cb(uint8_t itf){
    uint8_t buf[64];while(tud_cdc_n_available(itf)){
        size_t got=tud_cdc_n_read(itf,buf,sizeof buf),put=xStreamBufferSend(console_rx,buf,got,0);
        if(put<got){portENTER_CRITICAL(&audio_lock);stats.cdc_dropped_bytes+=got-put;portEXIT_CRITICAL(&audio_lock);}
    }
}
static void reboot_task(void *unused){(void)unused;vTaskDelay(pdMS_TO_TICKS(100));ull_usb_enter_bootloader();}
void tud_cdc_line_coding_cb(uint8_t itf,const cdc_line_coding_t *coding){(void)itf;baud_1200=coding->bit_rate==1200;}
void tud_cdc_line_state_cb(uint8_t itf,bool dtr,bool rts){
    (void)itf;(void)rts;console_connected=dtr&&tud_ready();
    if(baud_1200&&!dtr&&!boot_pending&&ull_usb_bootloader_allowed()){
        boot_pending=true;if(xTaskCreate(reboot_task,"usb_boot",3072,NULL,5,NULL)!=pdPASS)boot_pending=false;
    }
}
static int cdc_open(const char *path,int flags,int mode){(void)path;(void)flags;(void)mode;return 0;}
static int cdc_close(int fd){(void)fd;return 0;}
static int cdc_fstat(int fd,struct stat *s){(void)fd;memset(s,0,sizeof *s);s->st_mode=S_IFCHR;return 0;}
static int cdc_fcntl(int fd,int cmd,int arg){(void)fd;(void)arg;if(cmd==F_GETFL)return O_NONBLOCK;if(cmd==F_SETFL)return 0;errno=EINVAL;return -1;}
static ssize_t cdc_read(int fd,void *data,size_t size){(void)fd;if(!size)return 0;size_t got=xStreamBufferReceive(console_rx,data,size,0);if(!got){errno=EAGAIN;return -1;}return got;}
// Producer-side fsync never calls TinyUSB. The USB owner flushes every service pass.
static int cdc_fsync(int fd){(void)fd;return 0;}
static ssize_t cdc_write(int fd,const void *data,size_t size){
    (void)fd;if(!size)return 0;size_t sent=0;
    if(shutting_down)return size;
    if(xSemaphoreTake(console_tx,pdMS_TO_TICKS(100))==pdTRUE){
        int64_t deadline=esp_timer_get_time()+100000;
        while(sent<size && !shutting_down && console_connected){
            size_t count=xStreamBufferSend(console_tx_queue,(const uint8_t*)data+sent,size-sent,pdMS_TO_TICKS(1));
            sent+=count;if(sent==size || esp_timer_get_time()>=deadline)break;
        }
        xSemaphoreGive(console_tx);
    }
    if(sent<size){portENTER_CRITICAL(&audio_lock);stats.cdc_dropped_bytes+=size-sent;portEXIT_CRITICAL(&audio_lock);}
    return size;
}
// Only the USB task calls TinyUSB TX APIs; app/trace tasks merely enqueue bytes.
static void console_service(void){
    console_connected=tud_cdc_connected();
    uint8_t bytes[256];size_t budget=2048;
    while(budget){
        uint32_t available=console_connected?tud_cdc_write_available():sizeof bytes;
        if(!available)break;
        size_t wanted=sizeof bytes;if(wanted>available)wanted=available;if(wanted>budget)wanted=budget;
        size_t got=xStreamBufferReceive(console_tx_queue,bytes,wanted,0);
        if(!got)break;
        uint32_t sent=console_connected?tud_cdc_write(bytes,got):0;
        if(sent<got){portENTER_CRITICAL(&audio_lock);stats.cdc_dropped_bytes+=got-sent;portEXIT_CRITICAL(&audio_lock);}
        budget-=got;
    }
    if(console_connected)tud_cdc_write_flush();
    uint32_t queued=xStreamBufferBytesAvailable(console_tx_queue),free=tud_cdc_write_available();
    portENTER_CRITICAL(&audio_lock);stats.cdc_tx_buffered_bytes=queued;stats.cdc_fifo_free=free;portEXIT_CRITICAL(&audio_lock);
}
static void usb_loop(void *unused){
    (void)unused;
    stats.usb_task_core=xPortGetCoreID();stats.usb_irq_core=stats.usb_task_core;
    const usb_phy_config_t config={.controller=USB_PHY_CTRL_OTG,.target=USB_PHY_TARGET_INT,.otg_mode=USB_OTG_MODE_DEVICE,.otg_speed=USB_PHY_SPEED_FULL};
    usb_init_result=usb_new_phy(&config,&phy);
    if(usb_init_result==ESP_OK){
        const tusb_rhport_init_t init={.role=TUSB_ROLE_DEVICE,.speed=TUSB_SPEED_FULL};
        if(!tusb_init(0,&init))usb_init_result=ESP_FAIL;
    }
    xSemaphoreGive(usb_ready);
    if(usb_init_result!=ESP_OK){usb_task=NULL;vTaskDelete(NULL);return;}
    while(!shutting_down){tud_task_ext(1,false);feedback_service();ull_usb_controls_service();console_service();}
    // Tear down on the same core that owns the stack and its interrupt.
    tud_disconnect();tud_deinit(0);
    if(phy){usb_del_phy(phy);phy=NULL;}
    usb_task=NULL;xSemaphoreGive(usb_stopped);vTaskDelete(NULL);
}
void ull_usb_disconnect_phy(void){
    shutting_down=true;console_connected=false;
    // Wait for a bounded producer to leave its queue write, then request owner shutdown.
    xSemaphoreTake(console_tx,portMAX_DELAY);xSemaphoreGive(console_tx);
    if(usb_task)xSemaphoreTake(usb_stopped,portMAX_DELAY);
}
esp_err_t ull_usb_audio_init(void){
    ull_pcm_reset(&playback,2);ull_pcm_reset(&capture,1);
    if(!ull_usb_controls_init())return ESP_ERR_NO_MEM;
    console_rx=xStreamBufferCreate(4096,1);console_tx=xSemaphoreCreateMutex();
    console_tx_queue=xStreamBufferCreate(8192,1);usb_ready=xSemaphoreCreateBinary();usb_stopped=xSemaphoreCreateBinary();
    if(!console_rx||!console_tx||!console_tx_queue||!usb_ready||!usb_stopped)return ESP_ERR_NO_MEM;
    const esp_vfs_t vfs={.flags=ESP_VFS_FLAG_DEFAULT,.write=cdc_write,.open=cdc_open,.close=cdc_close,.read=cdc_read,.fstat=cdc_fstat,.fcntl=cdc_fcntl,.fsync=cdc_fsync};
    esp_err_t err=esp_vfs_register("/dev/ullcdc",&vfs,NULL);if(err!=ESP_OK)return err;
    if(!freopen("/dev/ullcdc","r",stdin)||!freopen("/dev/ullcdc","w",stdout)||!freopen("/dev/ullcdc","w",stderr))return ESP_FAIL;
    setvbuf(stdin,NULL,_IONBF,0);setvbuf(stdout,NULL,_IOLBF,0);setvbuf(stderr,NULL,_IONBF,0);
    // Initializing here would allocate DWC2 IRQ on app_main/core0 alongside radio.
    if(xTaskCreatePinnedToCore(usb_loop,"usb_audio",6144,NULL,6,&usb_task,1)!=pdPASS)return ESP_ERR_NO_MEM;
    xSemaphoreTake(usb_ready,portMAX_DELAY);return usb_init_result;
}
