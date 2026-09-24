#ifndef BLACKSHARK_AIR_FRAME_H
#define BLACKSHARK_AIR_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Record framing only. No RF header, hardware CRC, encryption, codec or timing.
 * These are the two-byte stream headers selected by the observed Air roles.
 * The stream bitmap is separate radio metadata, not embedded in the aggregate.
 */
#define AIR_MAX_STREAMS 4u
#define AIR_MAX_AGGREGATE 255u
#define AIR_MAX_RECORD_PAYLOAD 127u

enum air_frame_result {
    AIR_FRAME_OK = 0,
    AIR_FRAME_ARGUMENT = -1,
    AIR_FRAME_CAPACITY = -2,
    AIR_FRAME_MALFORMED = -3,
};

struct air_tx_record {
    const uint8_t *payload;
    size_t length;
    uint8_t stream_id;
    uint8_t tx_sequence;
    uint8_t ack_sequence;
    uint8_t kind;
};

struct air_rx_record {
    const uint8_t *payload; /* View into caller-owned aggregate. */
    size_t length;
    uint8_t stream_id;
    uint8_t tx_sequence;
    uint8_t ack_sequence;
    uint8_t kind;
    uint8_t flags; /* Raw second header byte; bit 2 empty, bit 3 suppressed. */
};

struct air_control_record {
    const uint8_t *payload;
    size_t length;
    uint8_t header;
    bool present;
};

/* Downlink appends the verified software checksum per record. Records must be
 * ordered by strictly increasing stream ID and share a length selector.
 * Source/output storage must not overlap. Outputs are unchanged on failure.
 * flags supplies aggregate bits 4..6 only; selector bits are derived here.
 */
int air_downlink_build(uint8_t *dst, size_t capacity,
                       const struct air_tx_record *records, size_t count,
                       const uint8_t *sizes, size_t size_count, uint8_t flags,
                       size_t *written, uint8_t *stream_mask);

/* Uplink has no per-record software checksum. This checks structure only;
 * radio integrity/encryption must be checked before trusting the payload.
 * It does not advance ACK state or deliver audio. Outputs unchanged on failure.
 */
int air_uplink_parse(const uint8_t *src, size_t length, uint8_t stream_mask,
                     const uint8_t *sizes, size_t size_count,
                     struct air_rx_record *records, size_t capacity,
                     size_t *record_count);

/* Air uplink bit0 announces a parent PDU after the record prefix. Bit1 alone
 * is a pending-control indication, not a trailer. Validates exact tail size;
 * outputs unchanged on failure. Caller must authenticate the whole packet. */
int air_uplink_parse_control(const uint8_t *src, size_t length, uint8_t air_header,
                             const uint8_t *sizes, size_t size_count,
                             struct air_rx_record *records, size_t capacity,
                             size_t *record_count,struct air_control_record *control);

#endif
