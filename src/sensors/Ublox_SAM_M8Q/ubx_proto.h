#ifndef UBX_PROTO_H
#define UBX_PROTO_H

#include "stdint.h"

/**
 * I only ever plan on using the NAV-PVT packet. However,
 * it is possible to have packets with a higher payload.
 * If support for those are desired this needs to be changed.
 */

#define MAX_EXPECTED_PAYLOAD    92

typedef struct {
    uint8_t syncChar_1;     // 0xB5
    uint8_t syncChar_2;     // 0x62
    uint8_t pktClass;
    uint8_t pktId;
    uint16_t payloadLen;    // Just message Payload length, no other part of packet.
    uint8_t payload[MAX_EXPECTED_PAYLOAD];
    uint8_t ck_a;           // Bytes calculated in checksum include class to
    uint8_t ck_b;           // end of checksum.
} Ubx_Packet_s;

int ubx_proto_send_pkt(Ubx_Packet_s* txPkt);

#endif // UBX_PROTO_H