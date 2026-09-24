#ifndef BLACKSHARK_RADIO_TX_H
#define BLACKSHARK_RADIO_TX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Console-safe RAM snapshot: [selected raw CS power index, primary event count].
 * Count0 means not observed yet. No live controller access or dBm conversion. */
void ull_radio_tx_power_snapshot(uint32_t out[2]);
/* Latest repeat only, individually atomic metadata snapshot (may straddle an
 * event while active). [count,program_delay,first_duration_hus,reply_duration_hus]
 * then <=24 rows of [kind,stage,reason,signed_relative_hus,ET0first,ET0selected].
 * kind:1 start,2 pushfirst,3 pushsecond,4 callback,5 abort,6 deadline,7 collect.
 * stage:0 first,1 second,2 stopper,255 general/unknown. Selected ET is stopper
 * for stage2, second otherwise. ET UINT32_MAX means not owned.
 * No keys/payload. Read after collection for stable results. */
void ull_radio_tx_retry_snapshot(uint32_t out[160]);
/* Controller task only; suppress diagnostic-only metadata during a bounded burst. */
void ull_radio_tx_retry_trace_enable(bool enabled);

/* Experimental physical transmitter, pinned to the S3 controller in this build.
 * It uses the test-TX radio format with caller-supplied AA, CRC seed and bytes.
 * This is NOT yet the complete HyperSpeed PHY/encryption/codec implementation.
 * The integrated tone trial is experimental; headset audio is not verified.
 */
struct ull_radio_tx_request {
    uint32_t start_hs;        /* 28-bit controller clock, 312.5 us per tick */
    uint16_t start_hus;       /* 0..624 half-us within the tick */
    uint32_t access_address;
    uint32_t crc_init;        /* 24 bits */
    const uint8_t *payload;   /* copied before returning */
    size_t length;            /* 1..255 */
    uint8_t channel;          /* BLE channel index 0..39 */
    uint8_t header;           /* nibble or diagnostic full Air header0x30 */
    uint8_t phy;              /* 1 or 2 Mbit/s */
    uint16_t receive_us;      /* 0=TX; 1..1000=bounded dedicated RX, no TX. */
    uint16_t tx_receive_us;   /* Optional sync window after TX/TIFS, 0..1000. */
    uint16_t followup_delay_us; /* Dedicated RX start relative to TX, or 0. */
    uint16_t followup_window_us;
    uint8_t followup_channel;
    uint8_t prequeued_reply;  /* Reserve inactive handle EF/CS2; queue RX with TX. */
    uint8_t repeat_payload;   /* With prequeued_reply: identical stereo TX at1420us, then native RX. */
};

enum ull_radio_tx_phase {
    ULL_TX_IDLE, ULL_TX_QUEUED, ULL_TX_ACTIVE, ULL_TX_ABORTING,
    ULL_TX_ENDED, ULL_TX_CANCELLED, ULL_TX_REFUSED, ULL_TX_FAULT
};
struct ull_radio_rx_snapshot {
    uint32_t hs;
    uint16_t hus, descriptor[10];
    uint8_t copied, payload[64];
};
struct ull_radio_tx_result {
    enum ull_radio_tx_phase phase;
    uint32_t end_hs;
    uint16_t tx_count;
    uint8_t callback_reason;
    uint8_t fault_detail; /* Diagnostic guard identifier; never relaxes ownership. */
    uint8_t deadline_fired;
    uint32_t test_configured, test_programmed, test_at_callback;
    uint32_t rx_timing_before, rx_timing_active, rx_timing_callback, rx_timing_restored;
    uint32_t sequence_before, sequence_active, sequence_callback, sequence_restored;
    uint8_t supplied_prefix[8];
    uint16_t rx_callbacks, tx_callbacks, rx_freed;
    uint8_t rx_snapshots, rx_invalid;
    uint32_t deadline_hs, abort_control;
    uint16_t deadline_hus;
    uint8_t abort_checks;
    uint8_t followup_status; /* 0=none,1=queued,2=started,3=ended,4=late,5=conflict,6=aborted,7=native RX complete */
    uint32_t followup_hs, first_end_hs;
    uint16_t followup_hus, first_end_hus;
    struct ull_radio_rx_snapshot rx[3];
};

/* Controller task only. Caller must own disabled advertising handle 0xee in
 * activity slot 1 for the entire operation; slot 0 is never read or modified.
 * Returns HCI-style status, or private status ULL_TX_SCHEDULE_CONFLICT when
 * arbitration rejected the requested slot and all storage has been restored.
 * No retries and no automatic rescheduling.
 */
#define ULL_TX_SCHEDULE_CONFLICT 0x80u
#define ULL_TX_TIME_EXPIRED 0x81u /* No storage allocated, no RF queued. */
/* Controller programming delay plus two 312.5-us ticks of margin. */
uint32_t ull_radio_tx_min_lead(void);
uint8_t ull_radio_tx_submit(const struct ull_radio_tx_request *request);

struct ull_radio_pdu_request {
    uint32_t start_hs;        /* Desired INNER preamble start. */
    uint16_t start_hus;
    uint32_t access_address;
    uint32_t crc_init;
    const uint8_t *pdu;       /* Complete header + body, already encrypted. */
    size_t length;           /* 2..246 on 2M, 2..247 on 1M. */
    uint8_t channel;
    uint8_t phy;
};
/* Experimental raw-packet route through unwhitened supplied DTM bytes. Builds
 * inner preamble/AA/PDU/CRC and whitening in software, shifts the outer start by
 * its prefix airtime, then uses the same ownership/deadline/cleanup contract.
 * Mid-packet receiver acquisition, hardware timing and Air crypto are unproven.
 */
uint8_t ull_radio_pdu_submit(const struct ull_radio_pdu_request *request);
/* Cancel before start, or request bounded abort of our active event only. */
void ull_radio_tx_cancel(void);
/* Copies result; reclaims resources only after a terminal callback. Return 1
 * when reclaimed, 0 while pending, -1 for ownership loss requiring HCI reset.
 * Do not release handle 0xee or reset the controller until this returns 1.
 */
int ull_radio_tx_collect(struct ull_radio_tx_result *result);

/* First insertion after successful repeat collection; one-shot scalar snapshot.
 * [0]valid,[1..2]signed(now-requested) half-us before/after insert,[3]raw status,
 * [4]required lead half-us,[5]duration,[6]active present/ours bits,[7]head present.
 * Pre-insert slack is -(int32_t)out[1]-(int32_t)out[4]. No foreign dereference. */
void ull_radio_tx_insert_snapshot(uint32_t out[8]);
void ull_radio_tx_retry_failure(uint32_t out[10]);

#endif
