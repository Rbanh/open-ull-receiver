#ifndef BLACKSHARK_TONE_PACKET_H
#define BLACKSHARK_TONE_PACKET_H
#include <stddef.h>
#include <stdint.h>
#include "air_session.h"

/* CCM and CSA2 verified against original radio traffic. Fixed-cue S3 playback
 * was human-confirmed in air-melody-icache32-clean-01; continuous PCM remains
 * a separate integration requirement. */
#define ULL_AIR_AUDIO_CHANNEL_BYTES 95u
int ull_ble_csa2_first(uint32_t aa, uint16_t event_counter,
                       const uint8_t map[5], uint8_t *channel);
int ull_ble_csa2_subevent(uint32_t aa, uint16_t event_counter,
                         const uint8_t map[5], uint8_t subevent, uint8_t *channel);
int ull_ble_ccm_encrypt(const uint8_t key_le[16], const uint8_t iv_le[8],
                        uint64_t counter, uint8_t direction, uint8_t aad,
                        const uint8_t *payload, size_t length,
                        uint8_t *out, size_t capacity);
int ull_ble_ccm_decrypt(const uint8_t key_le[16], const uint8_t iv_le[8],
                        uint64_t counter, uint8_t direction, uint8_t aad,
                        const uint8_t *payload, size_t length,
                        uint8_t *out, size_t capacity);
/* Serialized controller task only, like encrypt/decrypt. Frees and zeroizes
 * the cached expanded key at the end of a trial; no radio operation. */
void ull_ble_ccm_reset(void);

struct ull_tone_packet_options {
    uint32_t event_index;     /* Session time; tone wrapper also selects a stored frame. */
    uint64_t packet_counter; /* Explicit hypothesis; never inferred from ACL. */
    uint8_t direction;
    uint8_t header;          /* Air bitmap/mic-selection header hypothesis. */
    uint8_t aad;             /* Explicit masked header hypothesis. */
    uint8_t tx_sequence[2];
    uint8_t ack_sequence[2];
    uint8_t stream_mask;     /* 0 retains legacy stereo; otherwise2,4,6. */
};
struct ull_tone_packet {
    uint32_t start_hs;
    uint16_t start_hus;
    uint8_t channel;
    uint8_t length;
    uint8_t pdu[207];
};
/* Build one candidate tone PDU for the first subevent: actual LC3plus data,
 * recovered Air records/checksums, standard CCM under explicit inputs, CSA#2
 * subevent1 channel and plan-derived timing. No TX, ACK or state advancement.
 * The first physical Air slot follows the initial CSA2 generator step.
 * The fixed-cue path is retained as a reproducible diagnostic.
 */
int ull_tone_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          struct ull_tone_packet *out);
/* Caller-owned encoded frames, one95-byte channel each. Uses the same framing,
 * crypto and absolute session grid as the verified cue, without its201-frame
 * limit. Does not consume a USB buffer or advance codec/radio state. Input
 * frames are copied into the resulting packet; output is unchanged on error. */
int ull_audio_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES],
                          struct ull_tone_packet *out);
/* Same timing/crypto, but valid empty Air records. Used to isolate stream
 * establishment from LC3 payload delivery; this is not an audible tone. */
int ull_empty_packet_build(const struct ull_air_session_plan *plan,
                           const struct ull_tone_packet_options *options,
                           struct ull_tone_packet *out);
/* Stream1 empty records plus an empty embedded parent poll. Header must be
 * LLID1 with only current SN/NESN supplied; Air header0x11/AAD1. */
int ull_poll_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          uint8_t parent_header,struct ull_tone_packet *out);
/* Valid empty stream1 plus a bounded LLID3 control message, under the same CCM. */
int ull_control_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          uint8_t parent_header,const uint8_t *payload,size_t length,
                          struct ull_tone_packet *out);
/* Steady stereo plus an empty parent ACK. Downlink low three header bits
 * select stream2: require header0x32/AAD0x22 (stock downlink convention).
 * Parent header is LLID1 with only SN/NESN. No control payload/gain commands. */
int ull_audio_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES],
                          uint8_t parent_header,struct ull_tone_packet *out);
int ull_empty_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          uint8_t parent_header,struct ull_tone_packet *out);
int ull_tone_parent_ack_packet_build(const struct ull_air_session_plan *plan,
                          const struct ull_tone_packet_options *options,
                          uint8_t parent_header,struct ull_tone_packet *out);
#endif
