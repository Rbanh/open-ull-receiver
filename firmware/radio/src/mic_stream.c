#include "mic_stream.h"
#define DSP_USE_ESP_ROM_CRC 1
#include "spi_protocol.h"
#include "usb_audio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

enum {PIN_CLK=10,PIN_MOSI=9,PIN_MISO=8,PIN_CS=7,PIN_READY=12,Q=16};
struct packet {uint32_t frame,epoch,at_us;uint8_t sequence,bytes[40];};
static struct packet packets[Q];
static atomic_uint written,read_pos,epoch;
static atomic_bool enabled=true,active,ready;
static atomic_int fault;
static atomic_uint received,dropped,duplicates,decoded,invalid,usb_frames,peak;
static atomic_uint spi_errors,timeouts,crc_errors,sequence_errors,pings,last_us,max_us,stack_free,resets;
static atomic_uint stage,exchanges,edges;
static TaskHandle_t worker;
static spi_device_handle_t spi;
static DMA_ATTR dsp_frame_t tx_frame,rx_frame;
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"Internal queue atomics required");
void ull_mic_stream_enable(bool value){atomic_store(&enabled,value);}
void ull_mic_stream_new_session(void){atomic_fetch_add(&epoch,1);}
void ull_air_microphone_frame(uint32_t frame,uint8_t sequence,const uint8_t payload[40]){
 atomic_fetch_add(&received,1);
 if(!atomic_load_explicit(&active,memory_order_acquire))return;
 unsigned w=atomic_load_explicit(&written,memory_order_relaxed),r=atomic_load_explicit(&read_pos,memory_order_acquire);
 if(w-r>=Q){atomic_fetch_add(&dropped,1);return;}
 struct packet *p=&packets[w%Q];p->frame=frame;p->sequence=sequence;p->epoch=atomic_load(&epoch);p->at_us=(uint32_t)esp_timer_get_time();memcpy(p->bytes,payload,40);
 atomic_store_explicit(&written,w+1,memory_order_release);
}
static void IRAM_ATTR ready_isr(void *unused){
 (void)unused;BaseType_t wake=pdFALSE;atomic_fetch_add(&edges,1);
 if(worker)vTaskNotifyGiveFromISR(worker,&wake);
 if(wake)portYIELD_FROM_ISR();
}
static bool IRAM_ATTR exchange(void){
 atomic_store(&stage,1);
 /* One READY rising edge authorizes exactly one512byte transfer. The slave
  * lowers it before processing, then raises it after queuing its reply. */
 if(!ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20))){
  atomic_fetch_add(&timeouts,1);
  if(!gpio_get_level(PIN_READY))return false; /* bounded lost-edge recovery */
 }
 if(!gpio_get_level(PIN_READY))return false;
 atomic_store(&stage,2);
 spi_transaction_t t={.length=DSP_BYTES*8,.rxlength=DSP_BYTES*8,.tx_buffer=&tx_frame,.rx_buffer=&rx_frame};
 esp_err_t err=spi_device_transmit(spi,&t);
 atomic_fetch_add(&exchanges,1);atomic_store(&stage,3);
 if(err!=ESP_OK){atomic_fetch_add(&spi_errors,1);return false;}
 return true;
}
static bool IRAM_ATTR response(uint8_t op,uint32_t session,uint32_t seq,uint16_t length){
 if(!dsp_valid(&rx_frame,DSP_RESPONSE_MAGIC)){atomic_fetch_add(&crc_errors,1);return false;}
 if(rx_frame.bytes[5]!=op || dsp_get32(rx_frame.bytes+8)!=session || dsp_get32(rx_frame.bytes+12)!=seq){atomic_fetch_add(&sequence_errors,1);return false;}
 if(dsp_get16(rx_frame.bytes+20)!=length || dsp_get16(rx_frame.bytes+22)!=DSP_OK){atomic_fetch_add(&invalid,1);return false;}
 return true;
}
static bool IRAM_ATTR request(uint8_t op,uint32_t session,uint32_t seq,uint32_t frame,const uint8_t *payload,unsigned length,unsigned reply_length){
 for(unsigned attempt=0;attempt<2;attempt++){
  dsp_begin(&tx_frame,DSP_REQUEST_MAGIC,op,session,seq,frame,(uint16_t)length,DSP_OK);
  if(length)memcpy(tx_frame.bytes+DSP_PAYLOAD,payload,length);
  dsp_seal(&tx_frame);
  if(!exchange())continue;
  dsp_begin(&tx_frame,DSP_REQUEST_MAGIC,DSP_POLL,session,seq,frame,0,DSP_OK);dsp_seal(&tx_frame);
  if(exchange() && response(op,session,seq,(uint16_t)reply_length))return true;
 }
 return false;
}
static esp_err_t init_bus(void){
 if(dsp_crc32((const uint8_t *)"123456789",9)!=UINT32_C(0xcbf43926))return ESP_FAIL;
 gpio_config_t quiet={.pin_bit_mask=UINT64_C(0x3ffe),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
 esp_err_t e=gpio_config(&quiet);if(e)return e;
 gpio_set_level(PIN_CS,1);gpio_set_direction(PIN_CS,GPIO_MODE_OUTPUT);
 spi_bus_config_t bus={.mosi_io_num=PIN_MOSI,.miso_io_num=PIN_MISO,.sclk_io_num=PIN_CLK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=DSP_BYTES};
 e=spi_bus_initialize(SPI2_HOST,&bus,SPI_DMA_CH_AUTO);if(e)return e;
 spi_device_interface_config_t dev={.clock_speed_hz=8000000,.mode=0,.spics_io_num=PIN_CS,.queue_size=1,.cs_ena_pretrans=2,.cs_ena_posttrans=2};
 e=spi_bus_add_device(SPI2_HOST,&dev,&spi);if(e)return e;
 e=gpio_install_isr_service(ESP_INTR_FLAG_IRAM);if(e!=ESP_OK && e!=ESP_ERR_INVALID_STATE)return e;
 gpio_set_intr_type(PIN_READY,GPIO_INTR_POSEDGE);e=gpio_isr_handler_add(PIN_READY,ready_isr,NULL);if(e)return e;
 if(gpio_get_level(PIN_READY))xTaskNotifyGive(worker);
 return ESP_OK;
}
static void IRAM_ATTR run(void *unused){
 (void)unused;worker=xTaskGetCurrentTaskHandle();
 int err=init_bus();if(err){atomic_store(&fault,err);vTaskDelete(NULL);return;}
 uint32_t base=esp_random()|1u,seq=0,previous_frame=0,previous_epoch=0;uint8_t previous_sequence=0;bool have_previous=false,was_capture=false;uint32_t codec_epoch=0;
 for(;;){
  if(!atomic_load(&ready)){
   uint8_t test[32];for(unsigned i=0;i<sizeof test;i++)test[i]=(uint8_t)(i*37u+seq);
   bool ok=request(DSP_PING,base,++seq,0,test,sizeof test,sizeof test) && !memcmp(rx_frame.bytes+DSP_PAYLOAD,test,sizeof test);
   if(!ok){atomic_store(&active,false);vTaskDelay(pdMS_TO_TICKS(100));continue;}
   atomic_fetch_add(&pings,1);atomic_store_explicit(&ready,true,memory_order_release);
  }
  ull_usb_audio_stats_t usb;ull_usb_audio_stats(&usb);
  bool capture=atomic_load(&enabled) && usb.mounted && usb.capture_active;
  uint32_t current_epoch=atomic_load(&epoch);
  if(capture && (!was_capture || codec_epoch!=current_epoch)){
   atomic_store(&active,false);
   if(!request(DSP_RESET,base+current_epoch,++seq,0,NULL,0,0)){atomic_store(&ready,false);was_capture=false;continue;}
   atomic_fetch_add(&resets,1);codec_epoch=current_epoch;have_previous=false;
  }
  was_capture=capture;atomic_store_explicit(&active,capture,memory_order_release);
  unsigned r=atomic_load_explicit(&read_pos,memory_order_relaxed),w=atomic_load_explicit(&written,memory_order_acquire);
  if(!capture){atomic_store_explicit(&read_pos,w,memory_order_release);have_previous=false;vTaskDelay(pdMS_TO_TICKS(1));continue;}
  if(r==w){vTaskDelay(pdMS_TO_TICKS(1));continue;}
  if(w-r>4){unsigned skip=w-r-4;r+=skip;atomic_fetch_add(&dropped,skip);have_previous=false;}
  struct packet p=packets[r%Q];atomic_store_explicit(&read_pos,r+1,memory_order_release);
  if(p.epoch!=atomic_load(&epoch) || (uint32_t)esp_timer_get_time()-p.at_us>40000u){atomic_fetch_add(&dropped,1);have_previous=false;continue;}
  uint32_t delta=p.frame-previous_frame;
  if(have_previous && p.epoch==previous_epoch && (!delta || (p.sequence==previous_sequence && delta<16u))){atomic_fetch_add(&duplicates,1);continue;}
  int64_t start=esp_timer_get_time();
  bool ok=request(DSP_DECODE,base+p.epoch,++seq,p.frame,p.bytes,40,480);
  unsigned elapsed=(unsigned)(esp_timer_get_time()-start);atomic_store(&last_us,elapsed);if(elapsed>atomic_load(&max_us))atomic_store(&max_us,elapsed);
  if(!ok){atomic_fetch_add(&dropped,1);atomic_store(&ready,false);atomic_store(&active,false);have_previous=false;was_capture=false;continue;}
  if(p.epoch!=atomic_load(&epoch)){atomic_fetch_add(&dropped,1);have_previous=false;continue;}
  int16_t samples[240];unsigned pk=0;
  for(unsigned i=0;i<240;i++){samples[i]=(int16_t)dsp_get16(rx_frame.bytes+DSP_PAYLOAD+2*i);unsigned v=samples[i]<0?-(int)samples[i]:samples[i];if(v>pk)pk=v;}
  atomic_store(&peak,pk);atomic_fetch_add(&usb_frames,(unsigned)ull_usb_audio_write_capture(samples,240));
  unsigned count=atomic_fetch_add(&decoded,1)+1;
  if(count==1 || !(count%100))atomic_store(&stack_free,uxTaskGetStackHighWaterMark(NULL));
  previous_frame=p.frame;previous_epoch=p.epoch;previous_sequence=p.sequence;have_previous=true;
 }
}
bool ull_mic_stream_init(void){return xTaskCreatePinnedToCore(run,"mic_spi",6144,NULL,5,&worker,1)==pdPASS;}
void ull_mic_stream_stats(void){
 unsigned pins=0;for(unsigned i=1;i<=13;i++)pins|=(unsigned)gpio_get_level(i)<<i;
 printf("{\"spi_diag\":{\"ready_pin\":%d,\"stage\":%u,\"exchanges\":%u,\"edges\":%u,\"input_pins\":%u,\"resets\":%u}}\n",gpio_get_level(PIN_READY),atomic_load(&stage),atomic_load(&exchanges),atomic_load(&edges),pins,atomic_load(&resets));
 printf("{\"mic_stream\":{\"ready\":%s,\"active\":%s,\"fault\":%d,\"received\":%u,\"dropped\":%u,\"duplicates\":%u,\"decoded\":%u,\"invalid\":%u,\"usb_frames\":%u,\"peak\":%u,\"spi_errors\":%u,\"timeouts\":%u,\"crc_errors\":%u,\"sequence_errors\":%u,\"pings\":%u,\"roundtrip_us\":%u,\"max_roundtrip_us\":%u,\"stack_free_bytes\":%u}}\n",atomic_load(&ready)?"true":"false",atomic_load(&active)?"true":"false",atomic_load(&fault),atomic_load(&received),atomic_load(&dropped),atomic_load(&duplicates),atomic_load(&decoded),atomic_load(&invalid),atomic_load(&usb_frames),atomic_load(&peak),atomic_load(&spi_errors),atomic_load(&timeouts),atomic_load(&crc_errors),atomic_load(&sequence_errors),atomic_load(&pings),atomic_load(&last_us),atomic_load(&max_us),atomic_load(&stack_free));
}
