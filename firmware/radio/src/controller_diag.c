// Controller-task diagnostics and guarded raw control probes FCF2..FCF4.
// ABI/table layouts verified in the pinned IDF 5.5 S3 controller binary.
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "llcp_pool.h"
#include "raw_llcp.h"
#include "air_event.h"
#include "radio_program.h"
#include "radio_activity.h"
#include "tone_trial.h"
#include "trial_pump.h"
#include "pcm_stream.h"

typedef struct { uint16_t opcode; uint8_t flags, length; const void *params, *reply; } command_desc_t;
typedef int (*handler_t)(uint16_t, const void *, uint16_t, uint16_t);
typedef struct { uint16_t opcode, padding; handler_t handler; } command_handler_t;
typedef struct { void *table; uint16_t count; } registry_t;
_Static_assert(sizeof(command_desc_t)==12,"Unexpected descriptor ABI");
_Static_assert(sizeof(command_handler_t)==8,"Unexpected handler ABI");
extern registry_t esp_vendor_cmd, esp_handler;
extern void __real_r_hci_register_vendor_desc_tab_hack(void);
extern void __real_r_register_esp_vendor_cmd_handler_hack(void);
extern void r_llm_cmd_cmp_send(uint16_t opcode, uint8_t status);
extern void *r_emi_get_mem_addr_by_offset(uint16_t offset);
extern uint8_t *p_ble_util_buf_env;
extern void ull_controller_diag_record(const uint8_t *data, unsigned size);
static DRAM_ATTR command_desc_t descriptions[23];
static DRAM_ATTR command_handler_t handlers[23];
static bool description_ready;

static int diagnostic(uint16_t msg, const void *params, uint16_t dest, uint16_t opcode) {
    (void)msg; (void)params; (void)dest;
    // Runs under the controller's existing HCI command dispatcher, so future
    // bounded operations need not call controller internals from app_main.
    uint8_t result[12]={'U','L','L','D',1,0,0,0,0,0,0,0};
    if(opcode==0xfcff){
        /* No waiting here: all controller work remains serialized. Partial
         * failure is deliberately terminal; the keeper restarts the ESP. */
        bool ok=ull_pcm_stream_paused_for_reconnect() && !ull_trial_pump_busy() && ull_tone_trial_rearm() &&
                ull_trial_pump_rearm() && ull_pcm_stream_rearm();
        r_llm_cmd_cmp_send(opcode,ok?0:0x0c);return 0;
    }
    if(opcode==0xfcfb){
        struct ull_audio_source *source=ull_pcm_stream_source();
        if(!source || !ull_tone_trial_set_audio_source(source)){r_llm_cmd_cmp_send(opcode,0x0c);return 0;}
        uint8_t status=ull_tone_trial_start();
        if(!status && !ull_trial_pump_start()){ull_tone_trial_cancel();status=0x03;}
        r_llm_cmd_cmp_send(opcode,status);return 0;
    }
    if(opcode==0xfcfc){r_llm_cmd_cmp_send(opcode,ull_tone_trial_status());return 0;}
    if(opcode==0xfcfd){
        uint8_t status=ull_tone_trial_cancel();
        if(!ull_tone_trial_active())ull_trial_pump_finish(status);
        r_llm_cmd_cmp_send(opcode,status);return 0;
    }
    if(opcode==0xfcfe){
        uint8_t status=0x0c;
        if(ull_trial_pump_claim()){
            status=ull_tone_trial_step();
            if(!ull_tone_trial_active())ull_trial_pump_finish(status);
        }
        r_llm_cmd_cmp_send(opcode,status);return 0;
    }
    if(opcode==0xfcfa){r_llm_cmd_cmp_send(opcode,ull_radio_activity_snapshot());return 0;}
    if(opcode==0xfcf8){r_llm_cmd_cmp_send(opcode,ull_radio_program_arm());return 0;}
    if(opcode==0xfcf9){ull_radio_program_report();r_llm_cmd_cmp_send(opcode,0);return 0;}
    if(opcode==0xfcf6){uint8_t status=ull_air_event_arm();ull_air_event_report();r_llm_cmd_cmp_send(opcode,status);return 0;}
    if(opcode==0xfcf7){ull_air_event_report();r_llm_cmd_cmp_send(opcode,0);return 0;}
    if(opcode==0xfcf5){r_llm_cmd_cmp_send(opcode,ull_radio_timing_snapshot());return 0;}
    if(opcode>=0xfcf2 && opcode<=0xfcf4){
        r_llm_cmd_cmp_send(opcode,ull_raw_send(opcode==0xfcf2?0x12:opcode==0xfcf3?0xe4:0xe0));
        return 0;
    }
    if(opcode==0xfcf1){
        ull_pool_dry_run(result);
        ull_controller_diag_record(result,sizeof(result));
        r_llm_cmd_cmp_send(opcode,result[6]?0x0c:0);
        return 0;
    }
    uint8_t *env=p_ble_util_buf_env;
    result[7]=ull_raw_free_hook_ready();
    uintptr_t node=0;
    if (env) memcpy(&node,env,sizeof(node));
    while (node && result[5]<20) {
        uintptr_t first=(uintptr_t)env+32;
        if (node<first || node>=first+20*8 || (node-first)%8) {result[6]=1;break;}
        result[5]++;
        memcpy(&node,(void *)node,sizeof(node));
    }
    if (node) result[6]=1;
    if (env) {
        uintptr_t first=(uintptr_t)r_emi_get_mem_addr_by_offset(0x1c00);
        uintptr_t second=(uintptr_t)r_emi_get_mem_addr_by_offset(0x1c1b);
        uintptr_t last=(uintptr_t)r_emi_get_mem_addr_by_offset(0x1e01);
        uint16_t stride=(uint16_t)(second-first), span=(uint16_t)(last-first);
        memcpy(result+8,&stride,2);memcpy(result+10,&span,2);
    } else result[6]=2;
    ull_controller_diag_record(result,sizeof(result));
    r_llm_cmd_cmp_send(opcode,0);
    return 0;
}

void __wrap_r_hci_register_vendor_desc_tab_hack(void) {
    __real_r_hci_register_vendor_desc_tab_hack();
    description_ready=false;
    if (esp_vendor_cmd.count!=7 || !esp_vendor_cmd.table) return;
    memcpy(descriptions,esp_vendor_cmd.table,7*sizeof(descriptions[0]));
    // Existing FD0C entry has no parameters and a one-byte status reply.
    command_desc_t *source=&descriptions[4];
    if (source->opcode!=0xfd0c || source->flags!=9 || source->length!=0 || source->params) return;
    for(unsigned i=7;i<23;i++){descriptions[i]=*source;descriptions[i].opcode=0xfcf0+i-7;}
    esp_vendor_cmd.table=descriptions;esp_vendor_cmd.count=23;
    description_ready=true;
}

void __wrap_r_register_esp_vendor_cmd_handler_hack(void) {
    __real_r_register_esp_vendor_cmd_handler_hack();
    if (!description_ready || esp_handler.count!=7 || !esp_handler.table) return;
    memcpy(handlers,esp_handler.table,7*sizeof(handlers[0]));
    for (unsigned i=0;i<7;i++) if (handlers[i].opcode!=descriptions[i].opcode) return;
    for(unsigned i=7;i<23;i++)handlers[i]=(command_handler_t){.opcode=0xfcf0+i-7,.handler=diagnostic};
    esp_handler.table=handlers;esp_handler.count=23;
}
