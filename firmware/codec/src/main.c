#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "pcm_codec.h"
#include "mic_codec.h"
#include "codec_wire.h"
#define READY GPIO_NUM_12
#define DONE_ENC 1u
#define DONE_DEC 2u
static DMA_ATTR cw_frame_t rx,tx;
static cw_frame_t cached;
static TaskHandle_t coordinator,encoder_task,decoder_task;
static pcm_codec_t *encoder;
static mic_codec_t *decoder;
static int16_t play_in[480],mic_out[240];
static uint8_t mic_in[40],play_out[190];
static bool decode_reset,decode_audio,decode_plc;
/* Decoder-worker owned: failed decode attempts may also mutate history. */
static bool decoder_dirty;
static atomic_bool online,enc_ready,dec_ready;
static atomic_int fault,enc_result,dec_result;
static atomic_uint transactions,duplicates,errors,setups,completions,encoded,decoded,resets;
static atomic_uint resample_us,codec_us,encoder_core,decoder_core;
static atomic_uint enc_us,dec_us,max_enc_us,max_dec_us,enc_stack,dec_stack,enc_arena,mic_arena;
static atomic_uint last_bits,bad_crc,bad_header;
static uint32_t last_session,last_seq,decoder_session;
static bool have_cached,have_decoder_session;
/* IDF PSRAM atomic stubs use hardware atomics for these internal globals. */
void ull_pcm_stage_time(unsigned stage,uint32_t elapsed){if(stage==0)atomic_store(&resample_us,elapsed);else atomic_store(&codec_us,elapsed);}
void ull_pcm_detail_time(unsigned stage,uint32_t elapsed){(void)stage;(void)elapsed;}
static void IRAM_ATTR post_setup(spi_slave_transaction_t*t){(void)t;atomic_fetch_add(&setups,1);gpio_set_level(READY,1);}
static void IRAM_ATTR post_trans(spi_slave_transaction_t*t){(void)t;gpio_set_level(READY,0);atomic_fetch_add(&completions,1);}
static void done(unsigned bits){xTaskNotify(coordinator,bits,eSetBits);}
static void wait_done(unsigned mask){unsigned got=0;while((got&mask)!=mask){uint32_t bits=0;xTaskNotifyWait(0,UINT32_MAX,&bits,portMAX_DELAY);got|=bits;}}
static void memory(const char*stage){printf("MEM %s internal_free=%u largest=%u psram_free=%u\n",stage,(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));}
static void encode_worker(void*unused){
 (void)unused;atomic_store(&encoder_core,xPortGetCoreID());pcm_codec_layout_t l=pcm_codec_layout();atomic_store(&enc_arena,l.arena_bytes);
 void*a=heap_caps_aligned_alloc(16,l.arena_bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
 int rc=a?pcm_codec_init(&encoder,a,l.arena_bytes):ESP_ERR_NO_MEM;
 if(!esp_ptr_internal(&rc))rc=ESP_FAIL;
 atomic_store(&enc_result,rc);atomic_store(&enc_ready,rc==0);memory("encoder_init");done(DONE_ENC);
 if(rc){vTaskDelete(NULL);return;}
 for(;;){ulTaskNotifyTake(pdTRUE,portMAX_DELAY);int64_t start=esp_timer_get_time();rc=pcm_codec_encode(encoder,play_in,play_out);unsigned us=esp_timer_get_time()-start;
  atomic_store(&enc_us,us);if(us>atomic_load(&max_enc_us))atomic_store(&max_enc_us,us);atomic_store(&enc_result,rc);
  unsigned n=atomic_fetch_add(&encoded,1)+1;if(n==1||n%100==0)atomic_store(&enc_stack,uxTaskGetStackHighWaterMark(NULL));
  done(DONE_ENC);if((n&31u)==0)vTaskDelay(1);
 }
}
static void decode_worker(void*unused){
 (void)unused;atomic_store(&decoder_core,xPortGetCoreID());size_t n=mic_codec_size();atomic_store(&mic_arena,n);
 void*a=heap_caps_aligned_alloc(16,n,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
 int rc=a?mic_codec_init(&decoder,a,n):ESP_ERR_NO_MEM;
 if(!esp_ptr_internal(&rc)|| (a&&!esp_ptr_external_ram(a)))rc=ESP_FAIL;
 atomic_store(&dec_result,rc);atomic_store(&dec_ready,rc==0);memory("decoder_init");done(DONE_DEC);
 if(rc){vTaskDelete(NULL);return;}
 unsigned jobs=0;
 for(;;){ulTaskNotifyTake(pdTRUE,portMAX_DELAY);int64_t start=esp_timer_get_time();rc=0;
  if(!atomic_load(&dec_ready))rc=-1;
  if(!rc&&decode_reset){
   if(decoder_dirty) rc=mic_codec_reset(decoder);
   if(!rc){decoder_dirty=false;atomic_fetch_add(&resets,1);}else atomic_store(&dec_ready,false);
  }
  if(!rc&&decode_audio){decoder_dirty=true;rc=mic_codec_decode(decoder,decode_plc?NULL:mic_in,mic_out);if(rc==2)rc=0;if(!rc)atomic_fetch_add(&decoded,1);}
  unsigned us=esp_timer_get_time()-start;atomic_store(&dec_us,us);if(us>atomic_load(&max_dec_us))atomic_store(&max_dec_us,us);atomic_store(&dec_result,rc);
  ++jobs;if(jobs==1||jobs%100==0)atomic_store(&dec_stack,uxTaskGetStackHighWaterMark(NULL));done(DONE_DEC);if((jobs&31u)==0)vTaskDelay(1);
 }
}
static void process(size_t bits){
 atomic_store(&last_bits,bits);uint8_t flags=rx.b[5];uint32_t session=cw_get32(rx.b+8),seq=cw_get32(rx.b+12),frame=cw_get32(rx.b+16);
 bool crc=cw_get32(rx.b+CW_CRC)==esp_rom_crc32_le(0,rx.b,CW_CRC);
 bool header=bits==CW_BYTES*8 && cw_get32(rx.b)==CW_REQUEST && rx.b[4]==2 && rx.b[6]==0 && rx.b[7]==0 && !(flags&~15u) && (flags&(CW_MIC|CW_PLC))!=(CW_MIC|CW_PLC);
 for(unsigned i=20;i<32;i++)if(rx.b[i])header=false;
 if(!crc) {atomic_fetch_add(&bad_crc,1);}
 if(!header) {atomic_fetch_add(&bad_header,1);}
 if(!crc||!header){cw_begin(&tx,CW_RESPONSE,0,CW_BAD_WIRE,session,seq,frame);cw_seal(&tx);atomic_fetch_add(&errors,1);return;}
 if(have_cached&&session==last_session&&seq==last_seq){tx=cached;atomic_fetch_add(&duplicates,1);return;}
 cw_begin(&tx,CW_RESPONSE,0,CW_OK,session,seq,frame);unsigned pending=0;
 if(flags&CW_PLAY){for(unsigned i=0;i<480;i++)play_in[i]=(int16_t)((uint16_t)rx.b[CW_PLAY_IN+2*i]|(uint16_t)rx.b[CW_PLAY_IN+2*i+1]<<8);pending|=DONE_ENC;}
 decode_audio=(flags&(CW_MIC|CW_PLC))!=0;decode_plc=(flags&CW_PLC)!=0;
 decode_reset=(flags&CW_RESET)!=0 || (decode_audio&&(!have_decoder_session||decoder_session!=session));
 if(decode_audio||decode_reset){memcpy(mic_in,rx.b+CW_MIC_IN,40);pending|=DONE_DEC;}
 if(pending&DONE_ENC)xTaskNotifyGive(encoder_task);
 if(pending&DONE_DEC)xTaskNotifyGive(decoder_task);
 if(pending)wait_done(pending);
 if(pending&DONE_ENC){if(atomic_load(&enc_result))tx.b[6]|=CW_ENCODE_ERROR;else{tx.b[5]|=CW_PLAY;memcpy(tx.b+CW_PLAY_OUT,play_out,190);}}
 if(pending&DONE_DEC){if(atomic_load(&dec_result))tx.b[6]|=CW_DECODE_ERROR;else{
   if(decode_reset){have_decoder_session=true;decoder_session=session;}
   if(flags&CW_RESET)tx.b[5]|=CW_RESET;
   if(decode_audio){tx.b[5]|=flags&(CW_MIC|CW_PLC);for(unsigned i=0;i<240;i++){tx.b[CW_MIC_OUT+2*i]=(uint8_t)mic_out[i];tx.b[CW_MIC_OUT+2*i+1]=(uint8_t)((uint16_t)mic_out[i]>>8);}}
 }}
 if(tx.b[6]) {atomic_fetch_add(&errors,1);}
 cw_seal(&tx);cached=tx;have_cached=true;last_session=session;last_seq=seq;
}
static void serve(void*unused){
 (void)unused;coordinator=xTaskGetCurrentTaskHandle();memory("startup");int rc=0;
 if(xTaskCreatePinnedToCore(encode_worker,"encode",24576,NULL,5,&encoder_task,0)!=pdPASS){rc=ESP_ERR_NO_MEM;goto failed;}wait_done(DONE_ENC);rc=atomic_load(&enc_result);if(rc)goto failed;
 if(xTaskCreatePinnedToCore(decode_worker,"decode",24576,NULL,5,&decoder_task,1)!=pdPASS){rc=ESP_ERR_NO_MEM;goto failed;}wait_done(DONE_DEC);rc=atomic_load(&dec_result);if(rc)goto failed;
 spi_bus_config_t bus={.mosi_io_num=9,.miso_io_num=8,.sclk_io_num=10,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=CW_BYTES};
 spi_slave_interface_config_t slave={.spics_io_num=7,.queue_size=1,.mode=0,.post_setup_cb=post_setup,.post_trans_cb=post_trans};
 rc=spi_slave_initialize(SPI2_HOST,&bus,&slave,SPI_DMA_CH_AUTO);if(rc)goto failed;
 cw_begin(&tx,CW_RESPONSE,0,CW_NOT_READY,0,0,0);cw_seal(&tx);atomic_store(&online,true);memory("spi_ready");
 for(;;){spi_slave_transaction_t t={.length=CW_BYTES*8,.tx_buffer=&tx,.rx_buffer=&rx};memset(&rx,0,sizeof(rx));rc=spi_slave_queue_trans(SPI2_HOST,&t,portMAX_DELAY);if(rc)break;spi_slave_transaction_t*done_t=NULL;rc=spi_slave_get_trans_result(SPI2_HOST,&done_t,portMAX_DELAY);if(rc||done_t!=&t){if(!rc)rc=ESP_FAIL;break;}unsigned n=atomic_fetch_add(&transactions,1)+1;process(t.trans_len);if((n&31u)==0)vTaskDelay(1);}
failed:atomic_store(&fault,rc);atomic_store(&online,false);gpio_set_level(READY,0);vTaskDelete(NULL);
}
static void stats(void){printf("{\"codec_slave\":{\"online\":%s,\"fault\":%d,\"transactions\":%u,\"duplicates\":%u,\"errors\":%u,\"encoded\":%u,\"decoded\":%u,\"resets\":%u,\"encode_us\":%u,\"decode_us\":%u,\"max_encode_us\":%u,\"max_decode_us\":%u,\"encoder_arena\":%u,\"mic_arena\":%u,\"enc_stack_free\":%u,\"dec_stack_free\":%u,\"setups\":%u,\"completions\":%u,\"last_bits\":%u,\"bad_crc\":%u,\"bad_header\":%u,\"ready_gpio\":%d,\"cs_gpio\":%d,\"heap_internal_free\":%u,\"heap_internal_largest\":%u,\"heap_internal_min\":%u,\"psram_free\":%u,\"resample_us\":%u,\"codec_us\":%u,\"encoder_core\":%u,\"decoder_core\":%u}}\n",atomic_load(&online)?"true":"false",atomic_load(&fault),atomic_load(&transactions),atomic_load(&duplicates),atomic_load(&errors),atomic_load(&encoded),atomic_load(&decoded),atomic_load(&resets),atomic_load(&enc_us),atomic_load(&dec_us),atomic_load(&max_enc_us),atomic_load(&max_dec_us),atomic_load(&enc_arena),atomic_load(&mic_arena),atomic_load(&enc_stack),atomic_load(&dec_stack),atomic_load(&setups),atomic_load(&completions),atomic_load(&last_bits),atomic_load(&bad_crc),atomic_load(&bad_header),gpio_get_level(READY),gpio_get_level(GPIO_NUM_7),(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT),atomic_load(&resample_us),atomic_load(&codec_us),atomic_load(&encoder_core),atomic_load(&decoder_core));}
void app_main(void){gpio_config_t quiet={.pin_bit_mask=0x3ffe,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};gpio_config(&quiet);gpio_set_level(READY,0);gpio_set_direction(READY,GPIO_MODE_INPUT_OUTPUT);if(xTaskCreatePinnedToCore(serve,"codec_spi",4096,NULL,4,NULL,1)!=pdPASS)atomic_store(&fault,ESP_ERR_NO_MEM);printf("codec-slave04 ZERO CLK10 MOSI9 MISO8 CS7 READY12 v2 1280B; ?/stats\n");char line[16];unsigned n=0;for(;;){int c=getchar();if(c=='?'){stats();n=0;}else if(c=='\n'||c=='\r'){line[n]=0;if(!strcmp(line,"stats"))stats();n=0;}else if(c>=32&&c<127){if(n<15)line[n++]=c;else n=0;}if(c==EOF){clearerr(stdin);vTaskDelay(pdMS_TO_TICKS(20));}}}
