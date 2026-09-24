#include <assert.h>
#include <stdio.h>
#include "../firmware/radio/src/parent_ack.c"
static const uint8_t down[10]={6,0,1,1,10,1,0,2,0,2};
static const uint8_t up[10]={6,0,1,1,10,1,0,1,0,2};
int main(void){
 struct ull_parent_ack s;ull_parent_ack_reset(&s);
 assert(!ull_parent_ack_header(&s));
 assert(!ull_parent_ack_receive(&s,1,NULL,0,0,0));
 /* An unacknowledged speculative poll cannot bootstrap. */
 assert(!ull_parent_ack_receive(&s,10,down,10,1,0));
 assert(!ull_parent_ack_receive(&s,10,down,10,0,1));
 assert(!s.active);
 for(unsigned n=0;n<10;n++)assert(!ull_parent_ack_receive(&s,10,down,n,0,0));
 uint8_t bad[10];memcpy(bad,down,10);bad[2]=2;
 assert(!ull_parent_ack_receive(&s,10,bad,10,0,0));
 memcpy(bad,down,10);bad[4]=0xe2;
 assert(!ull_parent_ack_receive(&s,10,bad,10,0,0));
 memcpy(bad,down,10);bad[9]=255;
 assert(!ull_parent_ack_receive(&s,10,bad,10,0,0));
 assert(!ull_parent_ack_receive(&s,0x2a,down,10,0,0));
 assert(ull_parent_ack_receive(&s,10,down,10,0,0));
 assert(s.active && s.accepted==1 && ull_parent_ack_header(&s)==1);
 for(unsigned i=0;i<200;i++)assert(ull_parent_ack_receive(&s,i&1?26:10,down,10,1,0));
 assert(s.accepted==1 && s.duplicates==200 && s.acked==0);
 assert(ull_parent_ack_receive(&s,10,up,10,1,0));
 assert(s.accepted==1 && s.rx_nesn==0 && s.acked==0);
 assert(!ull_parent_ack_receive(&s,6,up,10,0,0));
 assert(!ull_parent_ack_receive(&s,6,up,10,1,1));
 assert(ull_parent_ack_receive(&s,6,up,10,1,0));
 assert(s.accepted==2 && s.acked==1 && ull_parent_ack_header(&s)==13);
 /* Old actual TX metadata may never advance the changed software sequence. */
 assert(!ull_parent_ack_receive(&s,6,up,10,1,0));
 assert(ull_parent_ack_receive(&s,22,up,10,13,0));
 assert(s.accepted==2 && s.acked==1);
 /* Empty PDUs participate in SN/NESN, without creating wheel events. */
 assert(ull_parent_ack_receive(&s,9,NULL,0,13,0));
 assert(s.empty==1 && s.acked==2 && ull_parent_ack_header(&s)==1);
 assert(ull_parent_ack_receive(&s,25,NULL,0,1,0));
 assert(s.empty==1 && s.accepted==2);
 /* Duplicate SN payload/kind is ignored, never consumed again. */
 assert(ull_parent_ack_receive(&s,10,down,10,1,0));
 uint8_t e1[9]={0xe1};assert(!ull_parent_ack_receive(&s,3,e1,9,1,0));
 ull_parent_ack_reset(&s);assert(!s.active && !s.accepted && !s.rejected);
 assert(ull_parent_ack_receive(&s,6,up,10,0,0));
 assert(s.tx_sn==1 && s.rx_nesn==1 && ull_parent_ack_header(&s)==13);
 /* Duplicate SN, changed payload: independently acknowledges local TX. */
 ull_parent_ack_reset(&s);assert(ull_parent_ack_receive(&s,10,down,10,0,0));
 assert(ull_parent_ack_receive(&s,14,up,10,1,0));
 assert(s.accepted==1 && s.duplicates==1 && s.rx_nesn==0 && s.tx_sn==1 && s.acked==1);
 /* Stale prior actual-send metadata cannot acknowledge a second time. */
 assert(!ull_parent_ack_receive(&s,14,up,10,1,0));assert(s.acked==1);
 /* Duplicate SN with changed length/kind still processes new NESN. */
 assert(ull_parent_ack_receive(&s,9,NULL,0,9,0));
 assert(s.acked==2 && s.tx_sn==0 && s.rx_nesn==0 && s.empty==0 && s.accepted==1);
 /* Unknown fresh LLCP is flow-controlled, but its TX ACK is independent. */
 assert(!ull_parent_ack_receive(&s,7,e1,9,1,0));
 assert(s.acked==3 && s.tx_sn==1 && s.rx_nesn==0 && s.accepted==1);
 /* Reserved LLID and invalid headers cannot acknowledge or consume. */
 assert(!ull_parent_ack_receive(&s,0,NULL,0,9,0));
 assert(!ull_parent_ack_receive(&s,0x23,e1,9,9,0));assert(s.acked==3);
 uint8_t scalar[6]={10,1,0,3,0,100};assert(ull_parent_ack_wheel(scalar,6));
 scalar[5]=101;assert(!ull_parent_ack_wheel(scalar,6));
 scalar[5]=2;scalar[1]=2;assert(!ull_parent_ack_wheel(scalar,6));
 /* Live probe response: first poll header1, headset wheel header6. */
 ull_parent_ack_reset(&s);
 assert(ull_parent_ack_receive(&s,6,up,10,1,0));
 assert(s.active && s.accepted==1 && ull_parent_ack_header(&s)==13);
 assert(ull_parent_ack_receive(&s,22,up,10,13,0));
 assert(s.accepted==1 && s.duplicates==1);
 /* New exact 0D media control advances the parent sequence once. */
 const uint8_t media[6]={2,0,1,1,13,1};
 assert(ull_parent_ack_receive(&s,10,media,6,13,0));
 assert(s.accepted==2);
 assert(ull_parent_ack_receive(&s,26,media,6,ull_parent_ack_header(&s),0));
 assert(s.accepted==2 && s.duplicates==2);
 ull_parent_ack_reset(&s);
 assert(ull_parent_ack_receive(&s,22,media,6,1,0));
 assert(s.active && s.accepted==1 && ull_parent_ack_header(&s)==13);
 assert(ull_parent_ack_receive(&s,22,media,6,13,0));
 assert(s.accepted==1 && s.duplicates==1);
 ull_parent_ack_reset(&s);
 const uint8_t mute[7]={3,0,1,1,5,2,0};
 assert(ull_parent_ack_receive(&s,6,mute,7,1,0));
 assert(s.active && s.accepted==1 && ull_parent_ack_header(&s)==13);
 assert(ull_parent_ack_receive(&s,22,mute,7,13,0));
 assert(s.accepted==1 && s.duplicates==1);
 ull_parent_ack_reset(&s);
 assert(ull_parent_ack_receive(&s,5,NULL,0,1,0));
 assert(s.active && s.empty==1 && s.accepted==0);
 puts("parent_ack: wheel/media/mic/empty poll bootstrap and dedup passed");
}
