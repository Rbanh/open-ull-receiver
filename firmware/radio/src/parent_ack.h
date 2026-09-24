#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Exact known scalar shape only; never key/setup material. */
static inline bool ull_parent_ack_wheel(const uint8_t *p,unsigned n){
    return p && n==6 && p[0]==10 && p[1]==1 && p[2]==0 && p[4]==0 &&
        (((p[3]==1 || p[3]==2) && p[5]==2) || (p[3]==3 && p[5]<=100));
}
static inline bool ull_parent_ack_media(const uint8_t *p,unsigned n){
    return p && n==2 && p[0]==13;
}
static inline bool ull_parent_ack_mic(const uint8_t *p,unsigned n){
    return p && n==3 && (p[0]==5 || p[0]==6) && p[1]==2 && p[2]==0;
}
struct ull_parent_ack {
    bool active;
    uint8_t tx_sn,rx_nesn;
    unsigned accepted,duplicates,empty,acked,rejected,stale;
};
void ull_parent_ack_reset(struct ull_parent_ack *s);
uint8_t ull_parent_ack_header(const struct ull_parent_ack *s);
/* Pure controller-owned experiment. sent_header/length describe actual TX,
 * not a freshly prepared frame. Header zero means no parent was sent. */
bool ull_parent_ack_receive(struct ull_parent_ack *s,uint8_t header,
    const uint8_t *payload,unsigned length,uint8_t sent_header,unsigned sent_length);
