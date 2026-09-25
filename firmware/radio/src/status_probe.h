#ifndef ULL_STATUS_PROBE_H
#define ULL_STATUS_PROBE_H
#include <stdint.h>
/* Counts authenticated proprietary controls without retaining packet contents. */
void ull_status_probe_air(uint8_t header,const uint8_t *payload,unsigned length);
void ull_status_probe_acl(const uint8_t *payload,unsigned length);
/* Observe a CCM-authenticated control-only candidate without accepting it. */
void ull_status_probe_control_only(uint8_t air_header,const uint8_t *plain,unsigned length);
void ull_status_probe_snapshot(uint32_t out[15]);
#endif
