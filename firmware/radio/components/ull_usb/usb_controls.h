#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Only called in the USB owner task, except the bounded nonblocking enqueue. */
bool ull_usb_controls_init(void);
void ull_usb_controls_service(void);
bool ull_usb_controls_enqueue(uint16_t usage);
/* Optional read-only EP0 diagnostics. Called by the USB owner task; the
 * provider must use bounded, nonblocking snapshots only. */
typedef void (*ull_usb_diagnostics_cb_t)(uint16_t page, uint32_t out[15]);
void ull_usb_controls_set_diagnostics(ull_usb_diagnostics_cb_t callback);
