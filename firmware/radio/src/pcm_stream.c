#include "pcm_stream.h"
#include "mic_stream.h"
#include "audio_source.h"
#include "tone_trial.h"
#include "usb_audio.h"
#include "codec_wire.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

enum {CLK=4,MOSI=5,MISO=6,CS=7,READY=2,Q=16};
static struct ull_audio_source source;
static atomic_bool ready,mic_enabled=true,mic_active;
static atomic_int fault;
static atomic_uint epoch,received,decoded,mic_dropped,duplicates,usb_frames,encoded,peak,stack_free;
static atomic_uint crc_errors,sequence_errors,timeouts,exchanges,codec_errors,queue_drops,reset_count;
static atomic_uint written,read_pos,last_us,max_us,playback_frames;
struct mic_packet {uint32_t frame,epoch,at_us;uint8_t seq,payload[40];};
static struct mic_packet packets[Q];
static DMA_ATTR cw_frame_t tx,rx;
static spi_device_handle_t spi;
static TaskHandle_t worker;
static uint32_t previous_session,previous_seq;
static bool have_previous;
static struct ull_reconnect_barrier reconnect;
bool ull_pcm_stream_pause_for_reconnect(void){
 /* Reject a previous acknowledgment that has not yet been retired. */
 if(atomic_load(&reconnect.acknowledged) || ull_reconnect_requested(&reconnect))return false;
 ull_reconnect_request(&reconnect);
 int64_t end=esp_timer_get_time()+500000;
 while(!ull_reconnect_paused(&reconnect) && esp_timer_get_time()<end)vTaskDelay(pdMS_TO_TICKS(1));
 return ull_reconnect_paused(&reconnect);
}
bool ull_pcm_stream_paused_for_reconnect(void){return ull_reconnect_paused(&reconnect);}
bool ull_pcm_stream_rearm(void){return ull_audio_source_rearm(&source,&reconnect);}
void ull_pcm_stream_resume_after_reconnect(void){ull_reconnect_resume(&reconnect);}
struct ull_audio_source *ull_pcm_stream_source(void){return atomic_load(&ready)?&source:NULL;}
bool ull_mic_stream_init(void){return true;}
void ull_mic_stream_enable(bool v){atomic_store(&mic_enabled,v);}
void ull_mic_stream_new_session(void){atomic_fetch_add(&epoch,1);}
void ull_air_microphone_frame(uint32_t f,uint8_t seq,const uint8_t data[40]){
 atomic_fetch_add(&received,1);if(!atomic_load_explicit(&mic_active,memory_order_acquire))return;
 unsigned w=atomic_load_explicit(&written,memory_order_relaxed),r=atomic_load_explicit(&read_pos,memory_order_acquire);
 if(w-r>=Q){atomic_fetch_add(&mic_dropped,1);return;}
 struct mic_packet *p=&packets[w%Q];p->frame=f;p->seq=seq;p->epoch=atomic_load(&epoch);p->at_us=(uint32_t)esp_timer_get_time();memcpy(p->payload,data,40);
 atomic_store_explicit(&written,w+1,memory_order_release);
}
static void IRAM_ATTR ready_isr(void *unused){(void)unused;BaseType_t wake=pdFALSE;if(worker)vTaskNotifyGiveFromISR(worker,&wake);if(wake)portYIELD_FROM_ISR();}
static bool IRAM_ATTR exchange(void){
 if(!ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(120)) && !gpio_get_level(READY)){atomic_fetch_add(&timeouts,1);return false;}
 if(!gpio_get_level(READY))return false;
 spi_transaction_t t={.length=CW_BYTES*8,.rxlength=CW_BYTES*8,.tx_buffer=&tx,.rx_buffer=&rx};
 esp_err_t e=spi_device_transmit(spi,&t);if(e){atomic_store(&fault,e);return false;}atomic_fetch_add(&exchanges,1);return true;
}
static esp_err_t init_bus(void){
 if(esp_rom_crc32_le(0,(const uint8_t *)"123456789",9)!=UINT32_C(0xcbf43926))return ESP_FAIL;
 gpio_config_t g={.pin_bit_mask=0x3ffe,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
 esp_err_t e=gpio_config(&g);if(e)return e;gpio_set_level(CS,1);gpio_set_direction(CS,GPIO_MODE_OUTPUT);
 spi_bus_config_t b={.mosi_io_num=MOSI,.miso_io_num=MISO,.sclk_io_num=CLK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=CW_BYTES,.isr_cpu_id=ESP_INTR_CPU_AFFINITY_1,.intr_flags=ESP_INTR_FLAG_IRAM};
 e=spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_CH_AUTO);if(e)return e;
 spi_device_interface_config_t d={.clock_speed_hz=16000000,.mode=0,.spics_io_num=CS,.queue_size=1,.cs_ena_pretrans=2,.cs_ena_posttrans=2};
 e=spi_bus_add_device(SPI2_HOST,&d,&spi);if(e)return e;
 e=gpio_install_isr_service(ESP_INTR_FLAG_IRAM);if(e!=ESP_OK && e!=ESP_ERR_INVALID_STATE)return e;
 gpio_set_intr_type(READY,GPIO_INTR_POSEDGE);e=gpio_isr_handler_add(READY,ready_isr,NULL);if(e)return e;
 if(gpio_get_level(READY))xTaskNotifyGive(worker);
 return ESP_OK;
}
static bool IRAM_ATTR accept_response(uint32_t current_session){
 if(!cw_valid(&rx,CW_RESPONSE)){atomic_fetch_add(&crc_errors,1);return false;}
 if(!have_previous)return false;
 if(cw_get32(rx.b+8)!=previous_session || cw_get32(rx.b+12)!=previous_seq){atomic_fetch_add(&sequence_errors,1);return false;}
 if(previous_session!=current_session)return false;
 if(rx.b[6])atomic_fetch_add(&codec_errors,1);
 if(rx.b[5]&CW_PLAY){
  uint8_t frame[2][95];memcpy(frame,rx.b+CW_PLAY_OUT,190);
  if(ull_audio_source_push(&source,frame))atomic_fetch_add(&encoded,1);else atomic_fetch_add(&queue_drops,1);
 }
 if((rx.b[5]&(CW_MIC|CW_PLC)) && atomic_load(&mic_active)){
  int16_t pcm[240];unsigned pk=0;memcpy(pcm,rx.b+CW_MIC_OUT,sizeof pcm);
  for(unsigned i=0;i<240;i++){unsigned n=pcm[i]<0?-(int)pcm[i]:pcm[i];if(n>pk)pk=n;}
  atomic_store(&peak,pk);atomic_fetch_add(&decoded,1);atomic_fetch_add(&usb_frames,ull_usb_audio_write_capture(pcm,240));
 }
 if(rx.b[5]&CW_RESET)atomic_fetch_add(&reset_count,1);
 return true;
}
static void IRAM_ATTR run(void *unused){
 (void)unused;worker=xTaskGetCurrentTaskHandle();int e=init_bus();if(e){atomic_store(&fault,e);vTaskDelete(NULL);return;}
 uint32_t base=esp_random()|1u,seq=0,reset_epoch=UINT32_MAX,last_mic_frame=0;uint8_t last_mic_seq=0;
 bool have_mic=false,was_capture=false;int64_t next_silence=0,last_transfer=0;
 for(;;){
  if(ull_reconnect_requested(&reconnect)){
   /* Radio has been collected before requesting pause. No ISR can append to
    * the microphone ring; the preceding SPI transaction has returned. */
   atomic_store(&mic_active,false);have_previous=false;have_mic=false;was_capture=false;
   atomic_store(&read_pos,atomic_load(&written));
   ull_reconnect_acknowledge(&reconnect);
   while(ull_reconnect_requested(&reconnect))vTaskDelay(pdMS_TO_TICKS(1));
   ull_reconnect_depart(&reconnect);
   continue;
  }
  uint32_t current_epoch=atomic_load(&epoch),session=base+current_epoch;
  if(!atomic_load(&ready)){
   cw_begin(&tx,CW_REQUEST,0,0,session,++seq,0);cw_seal(&tx);
   if(!exchange()){vTaskDelay(pdMS_TO_TICKS(20));continue;}
   bool ok=accept_response(session) && rx.b[6]==CW_OK;previous_session=session;previous_seq=seq;have_previous=true;
   if(ok)atomic_store(&ready,true);else vTaskDelay(pdMS_TO_TICKS(1));continue;
  }
  if(!atomic_load_explicit(&source.requested,memory_order_acquire)){atomic_store(&mic_active,false);was_capture=false;vTaskDelay(pdMS_TO_TICKS(1));continue;}
  unsigned w=atomic_load_explicit(&source.written,memory_order_relaxed),r=atomic_load_explicit(&source.read,memory_order_acquire);
  ull_usb_audio_stats_t usb;ull_usb_audio_stats(&usb);int64_t now=esp_timer_get_time();
  /* Decoder history belongs to the radio session. USB capture can stop and
   * restart without discarding that history or blocking playback on reinit.
   * The USB writer discards PCM while its capture endpoint is inactive. */
  bool capture=atomic_load(&mic_enabled);
  atomic_store_explicit(&mic_active,capture,memory_order_release);
  bool play=w-r<ULL_AUDIO_QUEUE_FRAMES-1;
  if(usb.mounted && usb.playback_active && usb.playback_buffered_frames<240)play=false;
  if(!usb.playback_active && now<next_silence)play=false;
  /* Keep microphone and already-prepared replies moving if playback is stalled. */
  if(!play && (!capture || now-last_transfer<5000)){vTaskDelay(pdMS_TO_TICKS(1));continue;}
  uint8_t flags=play?CW_PLAY:0;
  if(capture && (!was_capture || reset_epoch!=current_epoch)){flags|=CW_RESET;reset_epoch=current_epoch;have_mic=false;}
  was_capture=capture;
  cw_begin(&tx,CW_REQUEST,flags,0,session,++seq,0);
  if(play){
   int16_t pcm[480]={0};size_t got=ull_usb_audio_read_playback(pcm,240);atomic_fetch_add(&playback_frames,got);memcpy(tx.b+CW_PLAY_IN,pcm,sizeof pcm);next_silence=now+5000;
  }
  unsigned mr=atomic_load_explicit(&read_pos,memory_order_relaxed),mw=atomic_load_explicit(&written,memory_order_acquire);
  if(!capture){atomic_store_explicit(&read_pos,mw,memory_order_release);have_mic=false;}
  else while(mr!=mw){
   struct mic_packet p=packets[mr%Q];++mr;atomic_store_explicit(&read_pos,mr,memory_order_release);
   if(p.epoch!=current_epoch || (uint32_t)now-p.at_us>40000u){atomic_fetch_add(&mic_dropped,1);continue;}
   uint32_t delta=p.frame-last_mic_frame;
   if(have_mic && (!delta || (p.seq==last_mic_seq && delta<16u))){atomic_fetch_add(&duplicates,1);continue;}
   memcpy(tx.b+CW_MIC_IN,p.payload,40);tx.b[5]|=CW_MIC;cw_put32(tx.b+16,p.frame);last_mic_frame=p.frame;last_mic_seq=p.seq;have_mic=true;break;
  }
  cw_seal(&tx);int64_t start=esp_timer_get_time();
  if(!exchange()){atomic_store(&ready,false);atomic_store(&mic_active,false);have_previous=false;was_capture=false;continue;}
  last_transfer=esp_timer_get_time();
  unsigned dt=(unsigned)(last_transfer-start);atomic_store(&last_us,dt);if(dt>atomic_load(&max_us))atomic_store(&max_us,dt);
  accept_response(base+atomic_load(&epoch));previous_session=session;previous_seq=seq;have_previous=true;
  if(!(seq%100u))atomic_store(&stack_free,uxTaskGetStackHighWaterMark(NULL));
 }
}
bool ull_pcm_stream_init(void){ull_audio_source_init(&source);return xTaskCreatePinnedToCore(run,"codec_spi",8192,NULL,5,&worker,1)==pdPASS;}
void ull_pcm_stream_diagnostics(uint32_t out[14]){
 struct ull_audio_source_stats s;ull_audio_source_stats(&source,&s);
 out[0]=s.underflows;out[1]=s.discarded;
 out[2]=atomic_load(&exchanges);out[3]=atomic_load(&timeouts);
 out[4]=atomic_load(&crc_errors);out[5]=atomic_load(&sequence_errors);
 out[6]=atomic_load(&codec_errors);out[7]=atomic_load(&queue_drops);
 out[8]=atomic_load(&reset_count);out[9]=atomic_load(&last_us);
 out[10]=atomic_load(&max_us);out[11]=atomic_load(&received);
 out[12]=atomic_load(&decoded);out[13]=atomic_load(&mic_dropped);
}
void ull_pcm_stream_stats(void){
 struct ull_audio_source_stats s;ull_audio_source_stats(&source,&s);
 printf("{\"pcm_source\":{\"selected\":%u,\"underflows\":%u,\"discarded\":%u,\"queued\":%u,\"queue_valid\":%s}}\n",(unsigned)s.selected,(unsigned)s.underflows,(unsigned)s.discarded,(unsigned)s.queued,s.queue_valid?"true":"false");
 uint32_t d[8];ull_tone_trial_delivery(d);printf("{\"radio_delivery\":{\"completions\":%u,\"audio_completed\":%u,\"ack_left\":%u,\"ack_right\":%u,\"rejected\":%u}}\n",(unsigned)d[0],(unsigned)d[3],(unsigned)d[4],(unsigned)d[5],(unsigned)d[6]);
 uint32_t t[8];ull_tone_trial_stream_state(t);printf("{\"standalone_stream\":{\"phase\":%u,\"reason\":%u,\"frame\":%u,\"submitted\":%u,\"skipped\":%u,\"valid_rx\":%u,\"rejected_rx\":%u,\"continuous\":%u}}\n",(unsigned)t[0],(unsigned)t[1],(unsigned)t[2],(unsigned)t[3],(unsigned)t[4],(unsigned)t[5],(unsigned)t[6],(unsigned)t[7]);
 printf("{\"pcm_stream\":{\"ready\":%s,\"fault\":%d,\"remote_encoder\":true,\"encoded\":%u,\"usb_frames\":%u,\"stack_free_bytes\":%u}}\n",atomic_load(&ready)?"true":"false",atomic_load(&fault),atomic_load(&encoded),atomic_load(&playback_frames),atomic_load(&stack_free));
 printf("{\"bridge_stats\":{\"exchanges\":%u,\"timeouts\":%u,\"crc_errors\":%u,\"sequence_errors\":%u,\"codec_errors\":%u,\"playback_queue_drops\":%u,\"resets\":%u,\"exchange_us\":%u,\"max_exchange_us\":%u}}\n",atomic_load(&exchanges),atomic_load(&timeouts),atomic_load(&crc_errors),atomic_load(&sequence_errors),atomic_load(&codec_errors),atomic_load(&queue_drops),atomic_load(&reset_count),atomic_load(&last_us),atomic_load(&max_us));
}
void ull_mic_stream_stats(void){printf("{\"mic_stream\":{\"ready\":%s,\"active\":%s,\"received\":%u,\"decoded\":%u,\"dropped\":%u,\"duplicates\":%u,\"usb_frames\":%u,\"peak\":%u}}\n",atomic_load(&ready)?"true":"false",atomic_load(&mic_active)?"true":"false",atomic_load(&received),atomic_load(&decoded),atomic_load(&mic_dropped),atomic_load(&duplicates),atomic_load(&usb_frames),atomic_load(&peak));}
