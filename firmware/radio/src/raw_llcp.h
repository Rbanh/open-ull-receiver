#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "air_session.h"
#include "air_feedback.h"
struct ull_parent_pdu {
    uint8_t header, length;
    uint8_t payload[68];
};
/* Controller task, after owned RX and authentication. No ROM RX-buffer free.
 * This bounded Air transaction never writes the parent's CS/queues; a change
 * of the retained parent hardware sequence invalidates it. */
bool ull_raw_receive_air_control(const struct ull_air_control_rx *received,
                                 const struct ull_parent_pdu *sent);
uint8_t ull_raw_parent_air_pdu(struct ull_parent_pdu *pdu);
bool ull_raw_free_hook_ready(void);
uint8_t ull_raw_send(uint8_t opcode);
void ull_raw_reset_sequence(void);
bool ull_raw_receive_e1(const void *indication,uint16_t destination);
uint8_t ull_radio_timing_snapshot(void);
/* Controller task only. Retains actual E0/E1 and binds E2 to that connection's
 * original scheduled anchor. Prepare performs no TX. Commit rechecks the
 * instant, encryption, empty queue and exact prepared plan before enqueueing.
 * Caller must prepare the matching RF path first; neither API is host-enabled.
 * Returned E2 contains session keys and must not be logged.
 */
uint8_t ull_raw_prepare_session(const struct ull_air_session_seed *seed,
                               uint32_t offset_us, struct ull_air_session_plan *plan);
uint8_t ull_raw_commit_session(const struct ull_air_session_plan *plan);
bool ull_raw_session_confirmed(void);
/* Join exactly stock stream 2 to the just-committed stream-1 group. Preserves
 * group key/IV/AA/map and advances its Air counter by the measured whole-event
 * difference between the two future ACL instants. No host-controlled payload. */
uint8_t ull_raw_request_second_stream(const struct ull_air_session_plan *first);
bool ull_raw_second_stream_ready(void);
uint8_t ull_raw_prepare_second_stream(const struct ull_air_session_plan *first,
                                     struct ull_air_session_plan *second);
/* Read the prepared session's live ACL CRC seed, as the stock Air role does.
 * No controller memory is changed. Requires the same encrypted E0/E1 link. */
uint8_t ull_raw_session_crc_init(uint32_t *crc_init);
/* Read current parent map after E1, before preparing the Air session. */
uint8_t ull_raw_session_channel_map(uint8_t channel_map[5]);
/* Read-only empty-poll header from this session's encrypted parent CS.
 * No controller writes, queued-message consumption or sequence advancement. */
uint8_t ull_raw_parent_poll_header(uint8_t *header);

/* HCI task: successful matching disconnect only after native peer.started. */
void ull_raw_parent_retired(void);
/* Controller task only, confirmed stereo mask6. False zeroes output. */
bool ull_raw_detached_parent_air_pdu(struct ull_parent_pdu *pdu);
void ull_raw_control_probe_set(bool enabled);
void ull_raw_control_auto_set(bool enabled);
bool ull_raw_control_auto_get(void);
unsigned ull_raw_control_probe_remaining(void);
/* Console-only sanitized, individually atomic approximate counters. */
void ull_raw_detached_parent_status(void);
