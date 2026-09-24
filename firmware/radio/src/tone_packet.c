#include "esp_attr.h"
#include "tone_packet.h"
#include "tone_source.h"
#include "air_frame.h"
#include "mbedtls/ccm.h"
#include "mbedtls/platform_util.h"
#include <string.h>
#include <stdbool.h>

_Static_assert(ULL_AIR_AUDIO_CHANNEL_BYTES==ULL_TONE_CHANNEL_BYTES,"Confirmed audio profile");

/* Only the serialized controller task calls these helpers. Keep the expanded
 * session key across packet operations; CCM starts a fresh nonce/MAC state for
 * each call. Release the context on trial completion, cancellation or failure. */
static mbedtls_ccm_context session_ccm;
static uint8_t session_key_le[16];
static bool session_ccm_ready;

void ull_ble_ccm_reset(void)
{
    if(session_ccm_ready)mbedtls_ccm_free(&session_ccm);
    mbedtls_platform_zeroize(&session_ccm,sizeof(session_ccm));
    mbedtls_platform_zeroize(session_key_le,sizeof(session_key_le));
    session_ccm_ready=false;
}

static int IRAM_ATTR ccm_session_key(const uint8_t key_le[16])
{
    if(session_ccm_ready && !memcmp(session_key_le,key_le,16))return 0;
    ull_ble_ccm_reset();
    uint8_t key[16];
    for(unsigned i=0;i<16;i++)key[i]=key_le[15-i];
    mbedtls_ccm_init(&session_ccm);
    int result=mbedtls_ccm_setkey(&session_ccm,MBEDTLS_CIPHER_ID_AES,key,128);
    mbedtls_platform_zeroize(key,sizeof(key));
    if(result){mbedtls_ccm_free(&session_ccm);return result;}
    memcpy(session_key_le,key_le,16);session_ccm_ready=true;return 0;
}

static uint16_t permute(uint16_t x)
{
    x = (uint16_t)(((x & 0xaaaau) >> 1) | ((x & 0x5555u) << 1));
    x = (uint16_t)(((x & 0xccccu) >> 2) | ((x & 0x3333u) << 2));
    return (uint16_t)(((x & 0xf0f0u) >> 4) | ((x & 0x0f0fu) << 4));
}
int ull_ble_csa2_subevent(uint32_t aa, uint16_t counter,
                         const uint8_t map[5], uint8_t subevent, uint8_t *channel)
{
    if (!map || !channel || (map[4] & 0xe0) || subevent > 3) return -1;
    uint8_t channels[37];
    unsigned used = 0;
    for (unsigned i = 0; i < 37; i++)
        if (map[i / 8] & (1u << (i % 8))) channels[used++] = (uint8_t)i;
    if (used < 2) return -1;
    uint16_t id = (uint16_t)((aa >> 16) ^ aa), prn = counter ^ id;
    for (unsigned i = 0; i < 3; i++) prn = (uint16_t)(17u * permute(prn) + id);
    uint16_t event_prn = prn ^ id;
    uint8_t unmapped = (uint8_t)(event_prn % 37u);
    unsigned index;
    if (map[unmapped / 8] & (1u << (unmapped % 8))) {
        for (index = 0; channels[index] != unmapped; ++index) {}
    } else {
        index = (used * event_prn) >> 16;
    }
    /* Core Vol6B4.5.8.3.5/6. Keep prn_s before the identifier XOR. */
    unsigned distance = 1;
    unsigned a = used > 5 ? used - 5 : 0;
    unsigned b = used > 10 ? (used - 10) / 2 : 0;
    if (a > 3) a = 3;
    if (b > 11) b = 11;
    if (a > distance) distance = a;
    if (b > distance) distance = b;
    for (unsigned i = 0; i < subevent; ++i) {
        prn = (uint16_t)(17u * permute(prn) + id);
        unsigned width = used + 1 - 2 * distance;
        index = (index + distance + ((width * (prn ^ id)) >> 16)) % used;
    }
    *channel = channels[index];
    return 0;
}
int ull_ble_csa2_first(uint32_t aa, uint16_t counter,
                       const uint8_t map[5], uint8_t *channel)
{
    return ull_ble_csa2_subevent(aa, counter, map, 0, channel);
}

int IRAM_ATTR ull_ble_ccm_encrypt(const uint8_t key_le[16], const uint8_t iv_le[8],
                        uint64_t counter, uint8_t direction, uint8_t aad,
                        const uint8_t *payload, size_t length,
                        uint8_t *out, size_t capacity)
{
    if (!key_le || !iv_le || !payload || !out || !length || length > 240 ||
        capacity < length + 4 || counter >= (UINT64_C(1) << 39) || direction > 1)
        return -1;
    uint8_t nonce[13], encrypted[244];
    for (unsigned i = 0; i < 5; i++) nonce[i] = (uint8_t)(counter >> (8 * i));
    nonce[4] |= direction << 7;
    memcpy(nonce + 5, iv_le, 8);
    int result = ccm_session_key(key_le);
    if (!result) result = mbedtls_ccm_encrypt_and_tag(&session_ccm, length,
        nonce, sizeof(nonce), &aad, 1, payload, encrypted, encrypted + length, 4);
    if (!result) memcpy(out, encrypted, length + 4);
    mbedtls_platform_zeroize(nonce, sizeof(nonce));
    mbedtls_platform_zeroize(encrypted, sizeof(encrypted));
    return result ? -2 : 0;
}

int ull_ble_ccm_decrypt(const uint8_t key_le[16], const uint8_t iv_le[8],
                        uint64_t counter, uint8_t direction, uint8_t aad,
                        const uint8_t *payload, size_t length,
                        uint8_t *out, size_t capacity)
{
    if (!key_le || !iv_le || !payload || !out || length < 5 || length > 244 ||
        capacity < length - 4 || counter >= (UINT64_C(1) << 39) || direction > 1)
        return -1;
    uint8_t nonce[13], plain[240];
    for (unsigned i=0;i<5;i++) nonce[i]=(uint8_t)(counter>>(8*i));
    nonce[4]|=direction<<7;
    memcpy(nonce+5,iv_le,8);
    int result=ccm_session_key(key_le);
    if(!result) result=mbedtls_ccm_auth_decrypt(&session_ccm,length-4,nonce,sizeof(nonce),
        &aad,1,payload,plain,payload+length-4,4);
    if(!result) memcpy(out,plain,length-4);
    mbedtls_platform_zeroize(nonce,sizeof(nonce));
    mbedtls_platform_zeroize(plain,sizeof(plain));
    return result ? -2 : 0;
}

static int build_packet(const struct ull_air_session_plan *plan,
                        const struct ull_tone_packet_options *o,
                        struct ull_tone_packet *out, bool empty, int poll_header, bool stereo_ack,
                        const uint8_t *control,size_t control_length,
                        const uint8_t audio[2][ULL_AIR_AUDIO_CHANNEL_BYTES])
{
    if (!plan || !o || !out || plan->e2[0] != 0xe2 ||
        (!audio && !empty && o->event_index >= ULL_TONE_FRAME_COUNT)) return -1;
    struct ull_tone_packet packet = {0};
    uint8_t aggregate[201], mask;
    size_t length;
    uint8_t requested = o->stream_mask ? o->stream_mask : 6;
    if(requested != 2 && requested != 4 && requested != 6)return -1;
    if(empty || audio){
        struct air_tx_record records[2] = {0};
        size_t count = 0;
        const uint8_t sizes[1] = {ULL_AIR_AUDIO_CHANNEL_BYTES};
        for(unsigned i=0;i<2;i++)if(requested & (1u<<(i+1))){
            records[count++] = (struct air_tx_record){.stream_id=(uint8_t)(i+1),
                .tx_sequence=o->tx_sequence[i],.ack_sequence=o->ack_sequence[i],
                .kind=0,.length=empty?0:ULL_AIR_AUDIO_CHANNEL_BYTES,
                .payload=empty?NULL:audio[i]};
        }
        if(air_downlink_build(aggregate,sizeof(aggregate),records,count,
                             sizes,1,0,&length,&mask))return -1;
    }else if(ull_tone_build_mask(o->event_index,o->tx_sequence,o->ack_sequence,
                                requested,aggregate,sizeof(aggregate),&length,&mask) ||
             (length != 100 && length != 199))return -1;
    if (((o->header >> 3) & 15u) != mask ||
        ull_air_session_time(plan, o->event_index, 0, &packet.start_hs, &packet.start_hus) ||
        /* Original full-audio RF capture: physical slot0 uses CSA subevent1.
         * The initial hardware generator step precedes its first on-air slot.
         * See original-reference-comprehensive-01/audio-slot-summary.json. */
        ull_ble_csa2_subevent(plan->access_address, (uint16_t)(plan->event_counter + o->event_index),
                            plan->e2 + 19, 1, &packet.channel)) return -1;
    if(poll_header>=0){
        /* Original Air control trailer is inside the same CCM envelope. */
        if((stereo_ack ? (requested!=6 || o->header!=0x32 || o->aad!=0x22 || control_length) :
                         (!empty || requested!=2 || o->header!=0x11 || o->aad!=1)) ||
           control_length>68 || (control_length && !control) ||
           (poll_header & ~12)!=(control_length?3:1) ||
           length+2+control_length>sizeof(aggregate))return -1;
        aggregate[length++]=(uint8_t)poll_header;
        aggregate[length++]=(uint8_t)control_length;
        if(control_length){memcpy(aggregate+length,control,control_length);length+=control_length;}
    }
    packet.pdu[0] = o->header;
    packet.pdu[1] = (uint8_t)(length + 4);
    if (ull_ble_ccm_encrypt(plan->e2 + 52, plan->e2 + 44,
                            o->packet_counter, o->direction, o->aad,
                            aggregate, length, packet.pdu + 2, sizeof(packet.pdu) - 2))
        return -2;
    packet.length = (uint8_t)(length+6);
    *out = packet;
    return 0;
}

int ull_tone_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          struct ull_tone_packet *out)
{ return build_packet(plan,o,out,false,-1,false,NULL,0,NULL); }

int ull_audio_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES],
                          struct ull_tone_packet *out)
{ return frames ? build_packet(plan,o,out,false,-1,false,NULL,0,frames) : -1; }

int ull_empty_packet_build(const struct ull_air_session_plan *plan,
                           const struct ull_tone_packet_options *o,
                           struct ull_tone_packet *out)
{ return build_packet(plan,o,out,true,-1,false,NULL,0,NULL); }

int ull_poll_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          uint8_t parent_header,struct ull_tone_packet *out)
{ return build_packet(plan,o,out,true,parent_header,false,NULL,0,NULL); }

int ull_control_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          uint8_t parent_header,const uint8_t *payload,size_t length,
                          struct ull_tone_packet *out)
{ return length ? build_packet(plan,o,out,true,parent_header,false,payload,length,NULL) : -1; }

int ull_audio_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES],
                          uint8_t parent_header,struct ull_tone_packet *out)
{ return frames ? build_packet(plan,o,out,false,parent_header,true,NULL,0,frames) : -1; }
int ull_empty_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          uint8_t parent_header,struct ull_tone_packet *out)
{ return build_packet(plan,o,out,true,parent_header,true,NULL,0,NULL); }
int ull_tone_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *o,
                          uint8_t parent_header,struct ull_tone_packet *out)
{ return build_packet(plan,o,out,false,parent_header,true,NULL,0,NULL); }
