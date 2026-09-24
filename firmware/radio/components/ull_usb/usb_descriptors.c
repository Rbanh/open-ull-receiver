// Native USB interface for the research receiver. No Razer VID/PID impersonation.
#include <string.h>
#include <stdio.h>
#include "tusb.h"
#include "esp_mac.h"
#include "usb_descriptors.h"
const tusb_desc_device_t ull_usb_device_descriptor = {
    .bLength=sizeof(tusb_desc_device_t), .bDescriptorType=TUSB_DESC_DEVICE,
    .bcdUSB=0x0200, .bDeviceClass=TUSB_CLASS_MISC, .bDeviceSubClass=MISC_SUBCLASS_COMMON,
    .bDeviceProtocol=MISC_PROTOCOL_IAD, .bMaxPacketSize0=64,
    .idVendor=0xcafe, .idProduct=0x4011, .bcdDevice=0x0102,
    .iManufacturer=1, .iProduct=2, .iSerialNumber=3, .bNumConfigurations=1
};
static const uint8_t hid_report[] = {
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(1))
};
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance){
    (void)instance;return hid_report;
}
#define TOTAL_LEN (TUD_CONFIG_DESC_LEN + ULL_AUDIO_DESCRIPTOR_LEN + TUD_HID_DESC_LEN)
#define MASTER_CONTROLS ((AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS))
const uint8_t ull_usb_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1,ITF_TOTAL,0,TOTAL_LEN,0,250),
    TUD_AUDIO_DESC_IAD(ITF_AUDIO_CONTROL,3,2),
    TUD_AUDIO_DESC_STD_AC(ITF_AUDIO_CONTROL,0,2),
    TUD_AUDIO_DESC_CS_AC(0x0200,AUDIO_FUNC_HEADSET,ULL_AUDIO_AC_BODY_LEN,0),
    // Playback follows the independent radio clock; capture placeholder follows USB SOF.
    TUD_AUDIO_DESC_CLK_SRC(UAC_CLOCK,1,5,0,0),
    TUD_AUDIO_DESC_CLK_SRC(UAC_CAPTURE_CLOCK,5,5,0,0),
    TUD_AUDIO_DESC_INPUT_TERM(UAC_PLAYBACK_INPUT,AUDIO_TERM_TYPE_USB_STREAMING,0,UAC_CLOCK,2,3,0,0,4),
    TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(UAC_PLAYBACK_FEATURE,UAC_PLAYBACK_INPUT,MASTER_CONTROLS,0,0,4),
    TUD_AUDIO_DESC_OUTPUT_TERM(UAC_PLAYBACK_OUTPUT,AUDIO_TERM_TYPE_OUT_HEADPHONES,0,UAC_PLAYBACK_FEATURE,UAC_CLOCK,0,4),
    TUD_AUDIO_DESC_INPUT_TERM(UAC_CAPTURE_INPUT,AUDIO_TERM_TYPE_IN_GENERIC_MIC,0,UAC_CAPTURE_CLOCK,1,0,0,0,5),
    TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(UAC_CAPTURE_FEATURE,UAC_CAPTURE_INPUT,MASTER_CONTROLS,0,5),
    TUD_AUDIO_DESC_OUTPUT_TERM(UAC_CAPTURE_OUTPUT,AUDIO_TERM_TYPE_USB_STREAMING,0,UAC_CAPTURE_FEATURE,UAC_CAPTURE_CLOCK,0,5),
    TUD_AUDIO_DESC_STD_AS_INT(ITF_PLAYBACK,0,0,4),
    TUD_AUDIO_DESC_STD_AS_INT(ITF_PLAYBACK,1,2,4),
    TUD_AUDIO_DESC_CS_AS_INT(UAC_PLAYBACK_INPUT,0,AUDIO_FORMAT_TYPE_I,AUDIO_DATA_FORMAT_TYPE_I_PCM,2,3,0),
    TUD_AUDIO_DESC_TYPE_I_FORMAT(2,16),
    TUD_AUDIO_DESC_STD_AS_ISO_EP(0x01,TUSB_XFER_ISOCHRONOUS|TUSB_ISO_EP_ATT_ASYNCHRONOUS,196,1),
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,0,0,0),
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(0x84,3,1),
    TUD_AUDIO_DESC_STD_AS_INT(ITF_CAPTURE,0,0,5),
    TUD_AUDIO_DESC_STD_AS_INT(ITF_CAPTURE,1,1,5),
    TUD_AUDIO_DESC_CS_AS_INT(UAC_CAPTURE_OUTPUT,0,AUDIO_FORMAT_TYPE_I,AUDIO_DATA_FORMAT_TYPE_I_PCM,1,0,0),
    TUD_AUDIO_DESC_TYPE_I_FORMAT(2,16),
    TUD_AUDIO_DESC_STD_AS_ISO_EP(0x81,TUSB_XFER_ISOCHRONOUS|TUSB_ISO_EP_ATT_SYNCHRONOUS,98,1),
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,0,0,0),
    TUD_HID_DESCRIPTOR(ITF_HID,6,HID_ITF_PROTOCOL_NONE,sizeof(hid_report),0x82,16,10)
};
_Static_assert(sizeof(ull_usb_config_descriptor)==TOTAL_LEN,"USB descriptor length mismatch");
uint8_t const *tud_descriptor_device_cb(void) { return (const uint8_t*)&ull_usb_device_descriptor; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) { return index==0 ? ull_usb_config_descriptor : NULL; }
uint16_t const *tud_descriptor_string_cb(uint8_t index,uint16_t langid) {
    (void)langid; static uint16_t out[64]; static char serial[13];
    const char *strings[]={NULL,"Open Receiver Research","HyperSpeed Research S3",serial,"Stereo Playback","Mono Capture","Media Controls"};
    if(index>=sizeof(strings)/sizeof(strings[0]))return NULL;
    if(!index) {out[0]=(TUSB_DESC_STRING<<8)|4;out[1]=0x0409;return out;}
    if(index==3 && !serial[0]) {uint8_t mac[6];esp_efuse_mac_get_default(mac);snprintf(serial,sizeof serial,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);}
    size_t len=strlen(strings[index]);if(len>63)len=63;
    for(size_t i=0;i<len;i++)out[i+1]=(uint8_t)strings[index][i];
    out[0]=(uint16_t)((TUSB_DESC_STRING<<8)|(2+2*len));return out;
}
