#ifndef ULL_CONTROL_LOG_H
#define ULL_CONTROL_LOG_H
#include <stdbool.h>
#include <stdint.h>
/* Task callers only. Input must already be authenticated. Never retains input. */
void ull_control_log_air(uint8_t header,const uint8_t *payload,unsigned length);
void ull_control_log_acl(const uint8_t *payload,unsigned length);
/* Console task only: enable starts a fresh capture; disable preserves it.
 * Drain atomically disables capture before removing records. */
void ull_control_log_enable(bool enabled);
void ull_control_log_status(void);
void ull_control_log_drain(void);
#endif
