#ifndef BLACKSHARK_AIR_FEEDBACK_H
#define BLACKSHARK_AIR_FEEDBACK_H
#include "air_session.h"
#include "radio_tx.h"

struct ull_air_feedback_channel {
    uint32_t frame;
    uint8_t expected_tx, next_ack, valid, after_tx;
};
struct ull_air_feedback {
    uint32_t generation;
    struct ull_air_feedback_channel channels[2];
};

/* Owned copy; never retains a pointer into the decrypted stack buffer. */
struct ull_air_control_rx {
    uint8_t present, header, length;
    uint8_t payload[60];
};
int ull_air_feedback_accept_control(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot,
                             struct ull_air_control_rx *control);

/* Streaming caller supplies the actual owned event's absolute frame index.
 * The snapshot must fall within that exact5ms slot. This disambiguates the
 *28-bit hardware clock wrap without the diagnostic201-frame horizon. */
int ull_air_feedback_accept_frame(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot,
                             uint32_t frame_index,
                             struct ull_air_control_rx *control);

/* Controller task, after radio ownership is released. CRC, full ciphertext,
 * CCM MIC and Air record structure must pass before any state changes.
 * Returns0 for a valid packet, negative on rejection. No audio output. */
int ull_air_feedback_accept(struct ull_air_feedback *state,
                             const struct ull_air_session_plan *plan,
                             const struct ull_radio_rx_snapshot *snapshot);

/* Caller has sent these records in this frame and received the reply before
 * the observed expiry window. An ACK of TX+1 is already the next frame's
 * expected sequence; do not advance it a second time at the frame boundary. */
void ull_air_feedback_note_tx(struct ull_air_feedback *state, uint32_t frame,
                              uint8_t stream_mask, const uint8_t sequences[2]);

/* Diagnostic first-slot sender. ACK comes only from authenticated reception.
 * Expected TX is anchored to received ACK, with the measured once-per-frame
 * expiry rule between first-slot receive-only and subsequent TX intervals.
 * This is not a complete same-frame ARQ/retransmission implementation. */
void ull_air_feedback_sequences(const struct ull_air_feedback *state,
                                 uint32_t frame, const uint32_t starts[2],
                                 uint8_t tx_sequence[2], uint8_t ack_sequence[2]);
#endif
