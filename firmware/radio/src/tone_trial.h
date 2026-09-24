#ifndef BLACKSHARK_TONE_TRIAL_H
#define BLACKSHARK_TONE_TRIAL_H
#include <stdint.h>
#include <stdbool.h>
struct ull_audio_source;
/* Optional caller-owned encoded USB source, selected only before first start.
 * NULL keeps the reproducible stored cue. Lifetime must include terminal
 * cleanup. Stored cues use 201 events; USB streaming has a separate limit. */
bool ull_tone_trial_set_audio_source(struct ull_audio_source *source);
/* Controller task only. Caller has negotiated E4/E0/E1 and reserved disabled
 * advertising handle EE in activity1. Start attempts a one-second tone under
 * the explicit BLE-shaped Air hypotheses in tone_packet.c. No loop/retry session.
 * Step must be serviced promptly (~1ms) for5ms audio cadence; it skips late
 * frames rather than shifting the negotiated timeline. Controller reset/slot
 * release is forbidden until terminal cleanup, except after an ownership fault
 * which requires resetting the entire ESP32 before attempting another trial.
 */
bool ull_tone_trial_continuous(bool enabled);
void ull_tone_trial_stream_state(uint32_t out[8]);
void ull_tone_trial_progress(uint32_t out[4]);
/* Cumulative radio completions and first-reply acknowledgements. Missing
 * replies do not by themselves prove that downlink audio was lost. */
void ull_tone_trial_delivery(uint32_t out[8]);
void ull_tone_trial_channel_stats(uint32_t out[37][3]);
/* One explicitly armed repeat frame per boot; state0idle/1pending/2active/
 * 3collected/4failed. Commands never write controller MMIO. */
/* Burst once per boot: max24000 eligible attempts or125s; no persistent setting.
 * Snapshot: state, reason, attempted, completed, submitfail, scheduling skips. */
/* Atomic request only; controller owns transitions. Default enabled, no NVS.
 * Status: enabled,active-owner,sessionblocked,attempted,completed,failures,
 * post-auto scheduling skips,eligible warmup frames (1000 required).
 * Counts cumulative; blocked/warmup reset only after collected session rearm.
 * Manual probe/burst APIs retain their once-per-boot limits and exclusion. */
void ull_tone_trial_retry_auto_enable(bool enabled);
void ull_tone_trial_retry_auto_status(uint32_t out[8]);
/* cooldown_active, consecutive ordinary stereo completions(cap50), cumulative
 * recoveries, remaining_ms. Resume needs250ms AND50 completions plus enabled
 * and free manual owner. Recoverable scheduling failures never clear hard block. */
void ull_tone_trial_retry_auto_recovery_status(uint32_t out[4]);
/* Once-per-boot policy-only test request; no synthetic RF error or MMIO. */
bool ull_tone_trial_retry_cooldown_probe_arm(void);
bool ull_tone_trial_retry_burst_arm(void);
void ull_tone_trial_retry_burst_status(uint32_t out[6]);
/* First burst failure only: [0]valid,[1]reason5skip/6submitfail,[2]active frame
 * for submitfail or next_frame at skip detection (may already be incremented),
 * [3]last prepared index,[4..6]desired/primary/alternate vectors packed as
 * bytes tx0,tx1,ack0,ack1 (least to most significant),[7..8]cached indices,
 * [9]bits0/1 valid,2/3 indexmatch,4/5 parentmatch (primary/alternate),
 * [10]mode0none/1primary/2alternate/3rebuild/4gap,[11]successful rebuild us else0,
 * [12]preparation succeeded,[13..15]desired/primary/alternate masks.
 * Captured cache metadata is before selection; no key or payload bytes. */
void ull_tone_trial_retry_cache_failure(uint32_t out[16]);
bool ull_tone_trial_retry_probe_arm(void);
void ull_tone_trial_retry_probe_status(uint32_t out[17]);
void ull_tone_trial_retry_attempts(uint32_t out[64]);
uint8_t ull_tone_trial_start(void);
uint8_t ull_tone_trial_step(void);
uint8_t ull_tone_trial_cancel(void);
uint8_t ull_tone_trial_status(void);
bool ull_tone_trial_active(void);
bool ull_tone_trial_rearm(void); /* Controller only; rejects active/fault states. */
#endif
