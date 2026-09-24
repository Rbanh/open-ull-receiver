#ifndef BLACKSHARK_TRIAL_PUMP_H
#define BLACKSHARK_TRIAL_PUMP_H
#include <stdbool.h>
#include <stdint.h>

/* One nonblocking VHCI sender gate for the keeper, manual bridge and timer.
 * The actual SDK call is outside critical sections, as required by IDF. */
bool ull_hci_try_send(uint8_t *data, uint16_t length);
void ull_trial_pump_init(void);
bool ull_trial_pump_start(void);
bool ull_trial_pump_continuous(bool enabled);
bool ull_trial_pump_busy(void);
bool ull_trial_pump_rearm(void); /* Controller only, after terminal collection. */
bool ull_trial_pump_claim(void);
void ull_trial_pump_finish(uint8_t status);
bool ull_trial_pump_receive(const uint8_t *data, uint16_t length);
/* Compact, bounded private diagnostics. Only the printer drains frozen data. */
bool ull_trial_trace_record(const uint8_t *data, unsigned length);
bool ull_trial_trace_next(uint8_t *data, unsigned capacity, unsigned *length,
                          int64_t *timestamp_us);
#endif
