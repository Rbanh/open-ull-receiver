#include "usb_controls.h"
#include "usb_recovery.h"
#include "usb_audio.h"
#include <stdatomic.h>
#include <string.h>
#include "tusb.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static QueueHandle_t pending;
static bool release_pending;
static atomic_uint key_queued,key_sent;
static int64_t release_due;

bool ull_usb_controls_init(void){
    pending=xQueueCreate(32,sizeof(uint16_t));
    return pending!=NULL;
}
bool ull_usb_controls_enqueue(uint16_t usage){
    bool ok=pending && xQueueSend(pending,&usage,0)==pdTRUE;
    if(ok)atomic_fetch_add(&key_queued,1);
    return ok;
}
void ull_usb_controls_service(void){
    if(!tud_mounted() || !tud_hid_ready())return;
    int64_t now=esp_timer_get_time();
    if(release_pending){
        if(now<release_due)return;
        uint16_t zero=0;
        if(tud_hid_report(1,&zero,sizeof zero))release_pending=false;
        return;
    }
    uint16_t usage;
    if(pending && xQueuePeek(pending,&usage,0)==pdTRUE &&
       tud_hid_report(1,&usage,sizeof usage)){
        (void)xQueueReceive(pending,&usage,0);
        atomic_fetch_add(&key_sent,1);
        release_pending=true;release_due=now+12000;
    }
}
uint16_t tud_hid_get_report_cb(uint8_t instance,uint8_t report_id,
                              hid_report_type_t report_type,uint8_t *buffer,uint16_t reqlen){
    (void)instance;(void)report_id;(void)report_type;(void)buffer;(void)reqlen;
    return 0;
}
void tud_hid_set_report_cb(uint8_t instance,uint8_t report_id,
                           hid_report_type_t report_type,uint8_t const *buffer,uint16_t bufsize){
    (void)instance;(void)report_id;(void)report_type;(void)buffer;(void)bufsize;
}

/* EP0 maintenance request; native media keys need no host software. The exact
 * request changes to ROM download mode only after its status stage completes. */
extern bool ull_usb_bootloader_allowed(void);
static bool boot_pending;
static uint8_t health[16];
static void boot_after_ack(void *unused){
    (void)unused;
    vTaskDelay(pdMS_TO_TICKS(100));
    ull_usb_enter_bootloader();
}
bool tud_vendor_control_xfer_cb(uint8_t rhport,uint8_t stage,
                                tusb_control_request_t const *request){
    if(!request)return false;
    if(request->bmRequestType==0xc0 && request->bRequest==0x5b &&
       tu_le16toh(request->wValue)==0 && tu_le16toh(request->wIndex)==0 &&
       tu_le16toh(request->wLength)==sizeof health){
        if(stage==CONTROL_STAGE_SETUP){
            memcpy(health,"HID6",4);
            uint32_t queued=atomic_load(&key_queued),sent=atomic_load(&key_sent);
            memcpy(health+4,&queued,4);memcpy(health+8,&sent,4);
            health[12]=ull_usb_audio_headset_mic_muted();
            health[13]=tud_mounted();health[14]=tud_hid_ready();
            health[15]=boot_pending;
            return tud_control_xfer(rhport,request,health,sizeof health);
        }
        return true;
    }
    if(request->bmRequestType!=0x40 || request->bRequest!=0x5a ||
       tu_le16toh(request->wValue)!=0xb007 ||
       tu_le16toh(request->wIndex)!=0xc0de ||
       tu_le16toh(request->wLength)!=0)return false;
    if(stage==CONTROL_STAGE_SETUP){
        if(boot_pending || !ull_usb_bootloader_allowed())return false;
        return tud_control_status(rhport,request);
    }
    if(stage==CONTROL_STAGE_ACK && !boot_pending){
        boot_pending=true;
        if(xTaskCreate(boot_after_ack,"usb_boot",3072,NULL,5,NULL)!=pdPASS)boot_pending=false;
    }
    return true;
}
