#ifndef ULL_DUAL_SPI_PROTOCOL_H
#define ULL_DUAL_SPI_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#ifdef DSP_USE_ESP_ROM_CRC
#include "esp_rom_crc.h"
#endif
enum {DSP_BYTES=512,DSP_PAYLOAD=24,DSP_CAPACITY=480,DSP_CRC_OFFSET=508};
enum {DSP_POLL=0,DSP_DECODE=1,DSP_PING=2,DSP_RESET=3,DSP_ERROR=255};
enum {DSP_OK=0,DSP_BAD_FRAME=1,DSP_BAD_LENGTH=2,DSP_CODEC_ERROR=3,DSP_NOT_READY=4};
#define DSP_REQUEST_MAGIC UINT32_C(0x31515044)
#define DSP_RESPONSE_MAGIC UINT32_C(0x31525044)
typedef struct {uint8_t bytes[DSP_BYTES];} dsp_frame_t;
static inline uint16_t dsp_get16(const uint8_t *p){return (uint16_t)p[0]|(uint16_t)p[1]<<8;}
static inline uint32_t dsp_get32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static inline void dsp_put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static inline void dsp_put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static inline uint32_t dsp_crc32(const uint8_t *p,size_t n){
#ifdef DSP_USE_ESP_ROM_CRC
 return esp_rom_crc32_le(0,p,(uint32_t)n);
#else
 uint32_t crc=UINT32_MAX;
 for(size_t i=0;i<n;i++){crc^=p[i];for(unsigned j=0;j<8;j++)crc=(crc>>1)^(UINT32_C(0xedb88320)&(0u-(crc&1u)));}
 return ~crc;
#endif
}
static inline void dsp_begin(dsp_frame_t *f,uint32_t magic,uint8_t op,uint32_t session,uint32_t seq,uint32_t radio_frame,uint16_t length,uint16_t status){
 memset(f,0,sizeof(*f));dsp_put32(f->bytes,magic);f->bytes[4]=1;f->bytes[5]=op;
 dsp_put32(f->bytes+8,session);dsp_put32(f->bytes+12,seq);dsp_put32(f->bytes+16,radio_frame);
 dsp_put16(f->bytes+20,length);dsp_put16(f->bytes+22,status);
}
static inline void dsp_seal(dsp_frame_t *f){dsp_put32(f->bytes+DSP_CRC_OFFSET,dsp_crc32(f->bytes,DSP_CRC_OFFSET));}
static inline int dsp_valid(const dsp_frame_t *f,uint32_t magic){return dsp_get32(f->bytes)==magic && f->bytes[4]==1 && dsp_get16(f->bytes+20)<=DSP_CAPACITY && dsp_get32(f->bytes+DSP_CRC_OFFSET)==dsp_crc32(f->bytes,DSP_CRC_OFFSET);}
#endif
