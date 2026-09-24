// Read-only activity ownership snapshot for this pinned six-activity S3 build.
#include "radio_activity.h"
#include <stdint.h>
#include <string.h>
#include "sdkconfig.h"
_Static_assert(CONFIG_BT_CTRL_BLE_MAX_ACT == 6, "Revalidate controller activity bounds");
extern uint8_t *p_llm_env, *lld_adv_env[], *lld_con_env[];
extern void **r_osi_funcs_p;
extern void *r_emi_get_mem_addr_by_offset(uint16_t);
extern void ull_controller_diag_record(const uint8_t *, unsigned);
static int valid(const void *p, unsigned n)
{
    uintptr_t address=(uintptr_t)p;
    return address>=0x3fc80000 && address<=0x3fcf0000-n;
}
uint8_t ull_radio_activity_snapshot(void)
{
    uint8_t result[38]={'U','L','L','C',1,0,6,255};
    static const uint8_t offsets[]={0,2,4,6,8,10,22,24,26,28,30,40,42};
    uint8_t fields[8+3*sizeof(offsets)]={'U','L','L','V',1,1,255,sizeof(offsets)};
    ((void (*)(void))r_osi_funcs_p[5])();
    uint8_t *activities=NULL;
    if (valid(p_llm_env,12)) memcpy(&activities,p_llm_env+8,4);
    if (!valid(activities,68*6)) result[5]=1;
    else {
        if (valid(lld_con_env[0],143)) result[7]=lld_con_env[0][142];
        if(result[7]<6){
            /* Only documented control/timing/descriptor fields, ending at42.
             * Do not read AA/CRC, encryption keys, IVs, or packet buffers. */
            volatile const uint16_t *cs=r_emi_get_mem_addr_by_offset(
                (uint16_t)(0x400+90*result[7]));
            fields[5]=0;fields[6]=result[7];
            for(unsigned i=0;i<sizeof(offsets);i++){
                uint16_t value=cs[offsets[i]/2];
                fields[8+3*i]=offsets[i];fields[9+3*i]=(uint8_t)value;
                fields[10+3*i]=(uint8_t)(value>>8);
            }
        }
        for (unsigned i=0;i<6;i++) {
            uint8_t *activity=activities+68*i,*parameters=NULL;
            memcpy(&parameters,activity,4);
            unsigned offset=8+5*i;
            result[offset]=activity[64];
            result[offset+1]=parameters!=NULL;
            // Only inactive advertising state1 has the proven handle-at-zero ABI.
            result[offset+2]=activity[64]==1 && valid(parameters,1) ? parameters[0] : 255;
            result[offset+3]=lld_adv_env[i]!=NULL;
            result[offset+4]=lld_con_env[i]!=NULL;
        }
    }
    ((void (*)(void))r_osi_funcs_p[6])();
    ull_controller_diag_record(result,sizeof(result));
    ull_controller_diag_record(fields,sizeof(fields));
    return result[5] ? 0x0c : 0;
}
