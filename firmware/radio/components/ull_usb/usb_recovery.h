#pragma once

/* Call from a task, only after the caller has excluded an active radio trial.
 * Switches the connector from TinyUSB to the ROM's hardware USB Serial/JTAG
 * download transport. No flash, NVS, partition, or eFuse writes. */
void ull_usb_enter_bootloader(void) __attribute__((noreturn));
