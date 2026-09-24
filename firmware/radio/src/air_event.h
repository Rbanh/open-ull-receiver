#ifndef BLACKSHARK_AIR_EVENT_H
#define BLACKSHARK_AIR_EVENT_H
#include <stdint.h>
/* Fixed, non-RF scheduler experiment; no caller-supplied event or timing. */
uint8_t ull_air_event_arm(void);
void ull_air_event_report(void);
void ull_air_event_reset(void);
#endif
