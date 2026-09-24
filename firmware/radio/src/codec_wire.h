#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_rom_crc.h"
enum {CW_BYTES=1280,CW_CRC=1276,CW_PLAY_IN=32,CW_MIC_IN=992,CW_PLAY_OUT=32,CW_MIC_OUT=224};
enum {CW_PLAY=1,CW_MIC=2,CW_PLC=4,CW_RESET=8};
enum {CW_OK=0,CW_BAD_WIRE=1,CW_ENCODE_ERROR=2,CW_DECODE_ERROR=4,CW_NOT_READY=8};
#define CW_REQUEST UINT32_C(0x31513244)
#define CW_RESPONSE UINT32_C(0x31523244)
typedef struct {uint8_t b[CW_BYTES];} cw_frame_t;
static inline uint32_t cw_get32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static inline void cw_put32(uint8_t*p,uint32_t x){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(x>>(8*i));}
static inline void cw_begin(cw_frame_t*f,uint32_t magic,uint8_t flags,uint8_t status,uint32_t session,uint32_t seq,uint32_t radio_frame){memset(f,0,sizeof(*f));cw_put32(f->b,magic);f->b[4]=2;f->b[5]=flags;f->b[6]=status;cw_put32(f->b+8,session);cw_put32(f->b+12,seq);cw_put32(f->b+16,radio_frame);}
static inline void cw_seal(cw_frame_t*f){cw_put32(f->b+CW_CRC,esp_rom_crc32_le(0,f->b,CW_CRC));}
static inline int cw_valid(const cw_frame_t*f,uint32_t magic){return cw_get32(f->b)==magic && f->b[4]==2 && f->b[7]==0 && (f->b[5]&~15u)==0 && cw_get32(f->b+CW_CRC)==esp_rom_crc32_le(0,f->b,CW_CRC);}
