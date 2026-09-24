#include "phy_packet.h"
#include <string.h>

#define CLOCK_MASK UINT32_C(0x0fffffff)

int ull_phy_outer_whitening(uint8_t *bytes, size_t length, uint8_t channel)
{
    if (!bytes || length > ULL_PHY_PACKET_MAX || channel > 39) return -1;
    uint8_t lfsr = channel | 0x40u;
    for (size_t i = 0; i < length + 2; i++) {
        for (unsigned bit = 0; bit < 8; bit++) {
            if (lfsr & 1u) {
                if (i >= 2) bytes[i-2] ^= 1u << bit;
                lfsr ^= 0x88u;
            }
            lfsr >>= 1;
        }
    }
    return 0;
}

static uint8_t reverse8(uint8_t x)
{
    x = (uint8_t)((x >> 4) | (x << 4));
    x = (uint8_t)(((x & 0xccu) >> 2) | ((x & 0x33u) << 2));
    return (uint8_t)(((x & 0xaau) >> 1) | ((x & 0x55u) << 1));
}

int ull_phy_packet_build(uint8_t *out, size_t capacity, size_t *written,
                         uint32_t aa, uint32_t crc_init, uint8_t channel,
                         uint8_t phy, const uint8_t *pdu, size_t pdu_length)
{
    if (!out || !written || !pdu || phy < 1 || phy > 2 || channel > 39 ||
        crc_init > 0xffffffu || pdu_length < 2 ||
        pdu_length > ULL_PHY_PACKET_MAX - 4u - 3u - phy)
        return -1;
    size_t total = phy + 4u + pdu_length + 3u;
    if (capacity < total) return -1;
    /* Build privately to allow an overlapping input/output and atomic failure. */
    uint8_t packet[ULL_PHY_PACKET_MAX];
    size_t start = phy + 4u;
    memset(packet, (aa & 1u) ? 0x55 : 0xaa, phy);
    for (unsigned i = 0; i < 4; i++) packet[phy + i] = (uint8_t)(aa >> (8 * i));
    memcpy(packet + start, pdu, pdu_length);

    /* Core Vol 6 Part B 3.1.1: PDU bits LSB-first into x^24+...+1,
     * then CRC register positions 23 through 0 are transmitted in that order. */
    uint32_t crc = crc_init;
    for (size_t i = 0; i < pdu_length; i++) {
        for (unsigned bit = 0; bit < 8; bit++) {
            unsigned feedback = ((crc >> 23) ^ (pdu[i] >> bit)) & 1u;
            crc = (crc << 1) & 0xffffffu;
            if (feedback) crc ^= 0x65bu;
        }
    }
    for (unsigned i = 0; i < 3; i++)
        packet[start + pdu_length + i] = reverse8((uint8_t)(crc >> (16 - 8 * i)));

    /* Core Vol 6 Part B 3.2: whiten PDU AND CRC, not preamble or AA. */
    uint8_t lfsr = channel | 0x40u;
    for (size_t i = start; i < total; i++) {
        for (unsigned bit = 0; bit < 8; bit++) {
            if (lfsr & 1u) {
                packet[i] ^= 1u << bit;
                lfsr ^= 0x88u;
            }
            lfsr >>= 1;
        }
    }
    memcpy(out, packet, total);
    *written = total;
    return 0;
}

int ull_phy_dtm_start(uint32_t inner_hs, uint16_t inner_hus, uint8_t phy,
                      uint32_t *outer_hs, uint16_t *outer_hus)
{
    if (!outer_hs || !outer_hus || inner_hs > CLOCK_MASK || inner_hus > 624 ||
        phy < 1 || phy > 2) return -1;
    /* 1M: (1+4+2)*8 = 56 us. 2M: (2+4+2)*4 = 32 us. */
    int hus = (int)inner_hus - (phy == 1 ? 112 : 64);
    uint32_t hs = inner_hs;
    if (hus < 0) { hus += 625; hs = (hs - 1) & CLOCK_MASK; }
    *outer_hs = hs;
    *outer_hus = (uint16_t)hus;
    return 0;
}
