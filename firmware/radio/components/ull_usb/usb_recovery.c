/* Software recovery for this pinned ESP32-S3/IDF build.
 * Sources: IDF usb_console.c restart handler and Espressif arduino-esp32
 * esp32-hal-tinyusb.c usb_switch_to_cdc_jtag()/usb_persist_shutdown_handler().
 * TinyUSB descriptors must NOT be handed to the ROM via USB persistence.
 */
#include "usb_recovery.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "esp32s3/rom/usb/chip_usb_dw_wrapper.h"

/* The USB owner stops its task/interrupt and releases its PHY before returning. */
extern void ull_usb_disconnect_phy(void);

void ull_usb_enter_bootloader(void)
{
    ull_usb_disconnect_phy();
    /* Provide a real disconnect interval before changing device identity. */
    vTaskDelay(pdMS_TO_TICKS(200));
    usb_serial_jtag_ll_phy_enable_pad(false);
    usb_serial_jtag_ll_disable_intr_mask(USB_SERIAL_JTAG_LL_INTR_MASK);
    usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_LL_INTR_MASK);
    /* This official S3 HAL helper selects the internal PHY for hardware USJ. */
    usb_serial_jtag_ll_phy_enable_external(false);
    usb_serial_jtag_ll_phy_enable_pin_exchg(false);
    usb_serial_jtag_ll_phy_disable_pull_override();
    usb_serial_jtag_ll_phy_disable_vref_override();
    chip_usb_set_persist_flags(0);
    usb_serial_jtag_ll_phy_enable_pad(true);
    /* Allow the host to discard the old composite descriptor/address. */
    vTaskDelay(pdMS_TO_TICKS(250));
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
    __builtin_unreachable();
}
