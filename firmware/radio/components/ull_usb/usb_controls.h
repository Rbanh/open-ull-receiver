#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Only called in the USB owner task, except the bounded nonblocking enqueue. */
bool ull_usb_controls_init(void);
void ull_usb_controls_service(void);
bool ull_usb_controls_enqueue(uint16_t usage);
