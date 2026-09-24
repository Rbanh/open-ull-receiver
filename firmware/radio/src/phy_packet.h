#ifndef BLACKSHARK_PHY_PACKET_H
#define BLACKSHARK_PHY_PACKET_H

#include <stddef.h>
#include <stdint.h>
/* XOR the outer data-packet whitening sequence after its two-byte header.
 * Applying twice restores input. Used to preserve a complete inner packet on RF. */
int ull_phy_outer_whitening(uint8_t *bytes, size_t length, uint8_t channel);

/* Complete BLE-shaped uncoded packet, packed in transmission order (LSB first
 * within each byte). Encryption, proprietary headers, AA validity and channel
 * selection belong to the caller. CRC covers the already encrypted PDU.
 * The 255-byte limit permits embedding the result in supplied DTM TX bytes.
 */
#define ULL_PHY_PACKET_MAX 255u
int ull_phy_packet_build(uint8_t *out, size_t capacity, size_t *written,
                         uint32_t access_address, uint32_t crc_init,
                         uint8_t channel, uint8_t phy,
                         const uint8_t *pdu, size_t pdu_length);

/* Experimental DTM wrapper: compensate for the outer preamble, AA and header
 * preceding the embedded packet. Does not compensate an unmeasured hardware
 * startup delay and is not proof that the receiver acquires the inner packet.
 * Both packet and outer DTM use the same 1M or 2M PHY.
 */
int ull_phy_dtm_start(uint32_t inner_hs, uint16_t inner_hus, uint8_t phy,
                      uint32_t *outer_hs, uint16_t *outer_hus);

#endif
