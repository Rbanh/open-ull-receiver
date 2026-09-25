#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../firmware/radio/src/status_probe.h"
int main(void){
    uint32_t before[15]={0},after[15]={0};
    ull_status_probe_snapshot(before);
    assert(before[14]==2 && before[3]==0);
    const uint8_t wheel[]={6,0,1,1,10,1,0,2,0,2};
    const uint8_t unknown[]={3,0,1,1,0x2a,0x55,0x66};
    const uint8_t setup[]={2,0,1,1,0x0e,0x01};
    const uint8_t malformed[]={3,0,1,2,0x2a,0x55,0x66};
    const uint8_t acl[]={0x33,0x44};
    ull_status_probe_air(2,wheel,sizeof wheel);
    ull_status_probe_air(2,unknown,sizeof unknown);
    ull_status_probe_air(2,setup,sizeof setup);
    ull_status_probe_air(2,malformed,sizeof malformed);
    ull_status_probe_air(3,unknown,sizeof unknown);
    ull_status_probe_acl(acl,sizeof acl);
    const uint8_t only[]={0,2,9,5,0,1,1,0x2a,0x55,0x66,0x77,0x88};
    const uint8_t only_setup[]={0,2,6,2,0,1,1,0x0e,1};
    ull_status_probe_control_only(1,only,sizeof only);
    ull_status_probe_control_only(1,only_setup,sizeof only_setup);
    ull_status_probe_control_only(1,only,sizeof only-1);
    ull_status_probe_control_only(9,only,sizeof only);
    ull_status_probe_snapshot(after);
    assert(after[0]==2 && after[1]==1 && after[2]==1 && after[3]==2);
    assert(after[4]==1 && after[5]==1);
    assert(after[6]==((2u<<24)|(2u<<8)|0x33u));
    assert(after[7]==2 && after[8]==1 && after[9]==1);
    assert(after[10]==((5u<<8)|0x2au));
    puts("status_probe: metadata only, setup hidden and malformed frames rejected");
}
