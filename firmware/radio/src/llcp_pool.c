#include <stdint.h>
#include <string.h>
#include "llcp_pool.h"
extern uint8_t *p_ble_util_buf_env;
extern void *r_ble_util_buf_llcp_tx_alloc(void);
extern void r_ble_util_buf_llcp_tx_free(uint16_t offset);
extern void *r_emi_get_mem_addr_by_offset(uint16_t offset);

static int descriptor_index(const void *node){
    uintptr_t p=(uintptr_t)node,base=(uintptr_t)p_ble_util_buf_env+32;
    if(!p_ble_util_buf_env || p<base || p>=base+160 || (p-base)%8)return -1;
    return (p-base)/8;
}
int ull_pool_free_count(void){
    if(!p_ble_util_buf_env)return -1;
    void *node;memcpy(&node,p_ble_util_buf_env,4);
    uint32_t seen=0;int count=0;
    while(node){
        int index=descriptor_index(node);
        if(index<0 || (seen&(1u<<index)))return -1;
        seen|=1u<<index;count++;
        memcpy(&node,node,4);
    }
    return count;
}
bool ull_pool_reserve(ull_pool_reservation_t *r,unsigned count){
    if(!r || count<1 || count>7)return false;
    memset(r,0,sizeof(*r));
    void *owned[20]={0};bool valid=true;
    for(unsigned n=0;n<20;n++){
        void *d=r_ble_util_buf_llcp_tx_alloc();if(!d)break;
        int index=descriptor_index(d);uint16_t offset;
        if(index<0 || owned[index]){valid=false;break;}
        owned[index]=d;memcpy(&offset,(uint8_t *)d+4,2);
        if(offset!=0x1c00+27*index){valid=false;break;}
    }
    int start=-1;
    if(valid)for(unsigned i=0;i+count<=20;i++){
        bool found=true;
        for(unsigned j=0;j<count;j++)if(!owned[i+j]){found=false;break;}
        if(found){start=i;break;}
    }
    if(start>=0){
        r->memory=r_emi_get_mem_addr_by_offset(0x1c00+27*start);
        for(unsigned i=0;i<count;i++){
            uint16_t offset=0x1c00+27*(start+i);
            if((uintptr_t)r_emi_get_mem_addr_by_offset(offset)!=(uintptr_t)r->memory+27*i){start=-1;break;}
        }
    }
    for(unsigned i=0;i<20;i++)if(owned[i]){
        if(start>=0 && i>=(unsigned)start && i<(unsigned)start+count){
            unsigned j=i-start;r->descriptors[j]=owned[i];r->offsets[j]=0x1c00+27*i;
        }else r_ble_util_buf_llcp_tx_free(0x1c00+27*i);
    }
    if(start<0){memset(r,0,sizeof(*r));return false;}
    r->count=count;return true;
}
void ull_pool_release(ull_pool_reservation_t *r,unsigned first){
    for(unsigned i=first;i<r->count;i++)if(r->descriptors[i]){
        r_ble_util_buf_llcp_tx_free(r->offsets[i]);r->descriptors[i]=NULL;
    }
    if(first==0)memset(r,0,sizeof(*r));
}
void ull_pool_dry_run(uint8_t result[12]){
    memcpy(result,"ULLB\1",5);memset(result+5,0,7);
    int before=ull_pool_free_count();result[5]=before<0?255:before;
    ull_pool_reservation_t r;
    if(before<7 || !ull_pool_reserve(&r,7)){result[6]=1;return;}
    result[7]=ull_pool_free_count();
    // Exclusively owned 189-byte span: 182-byte payload plus room for MIC.
    memset(r.memory,0xa5,189);
    for(unsigned i=0;i<182;i++)r.memory[i]=(uint8_t)(i^0x5a);
    bool pattern=true;
    for(unsigned i=0;i<189;i++){
        uint8_t expected=i<182?(uint8_t)(i^0x5a):0xa5;
        if(r.memory[i]!=expected){pattern=false;break;}
    }
    ull_pool_release(&r,0);
    int after=ull_pool_free_count();result[8]=after<0?255:after;
    result[9]=189;result[10]=pattern;
    if(!pattern || after!=before)result[6]=2;
}
