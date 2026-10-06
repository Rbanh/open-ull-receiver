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
static bool read_diagnostics(uint16_t page,uint32_t out[16]);
uint16_t tud_hid_get_report_cb(uint8_t instance,uint8_t report_id,
                              hid_report_type_t report_type,uint8_t *buffer,uint16_t reqlen){
    if(instance!=0 || report_type!=HID_REPORT_TYPE_FEATURE || !buffer ||
       reqlen<64 || report_id<0x10 || report_id>0x1f)return 0;
    uint32_t snapshot[16];
    if(!read_diagnostics((uint16_t)(report_id-0x10),snapshot))return 0;
    memcpy(buffer,snapshot,sizeof snapshot);
    return sizeof snapshot;
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
static uint32_t diagnostics[16];
static _Atomic(ull_usb_diagnostics_cb_t) diagnostics_cb;
void ull_usb_controls_set_diagnostics(ull_usb_diagnostics_cb_t callback){
    atomic_store_explicit(&diagnostics_cb,callback,memory_order_release);
}
static bool read_diagnostics(uint16_t page,uint32_t out[16]){
    ull_usb_diagnostics_cb_t callback=atomic_load_explicit(
        &diagnostics_cb,memory_order_acquire);
    if(page!=15 && (page>14 || !callback))return false;
    memset(out,0,64);
    uint8_t *tag=(uint8_t *)out;
    tag[0]='R';tag[1]='F';tag[2]=(uint8_t)('0'+page/10u);tag[3]=(uint8_t)('0'+page%10u);
    if(page==15)ull_usb_audio_diagnostics(out+1);
    else callback(page,out+1);
    return true;
}
static void boot_after_ack(void *unused){
    (void)unused;
    vTaskDelay(pdMS_TO_TICKS(100));
    ull_usb_enter_bootloader();
}
bool tud_vendor_control_xfer_cb(uint8_t rhport,uint8_t stage,
                                tusb_control_request_t const *request){
    if(!request)return false;
    if(request->bmRequestType==0xc0 && request->bRequest==0x5c &&
       tu_le16toh(request->wValue)<=15 && tu_le16toh(request->wIndex)==0 &&
       tu_le16toh(request->wLength)==sizeof diagnostics){
        if(stage==CONTROL_STAGE_SETUP){
            uint16_t page=tu_le16toh(request->wValue);
            if(!read_diagnostics(page,diagnostics))return false;
            return tud_control_xfer(rhport,request,diagnostics,sizeof diagnostics);
        }
        return true;
    }
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
