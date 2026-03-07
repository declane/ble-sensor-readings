#ifndef UBX_PROTO_H
#define UBX_PROTO_H

#include "stdint.h"

/**
 * I only ever plan on using the NAV-PVT packet. However,
 * it is possible to have packets with a higher payload.
 * If support for those are desired this needs to be changed.
 */

#define MAX_EXPECTED_PAYLOAD    92


#define UBX_SYNC1_CHAR          0xB5
#define UBX_SYNC2_CHAR          0x62

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

typedef enum {
    Parsing_Idle,
    Parsing_Active,
    Parsing_Complete,
    Parsing_Timeout,
    Parsing_Checksum_Error,
    Parsing_Length_Limit
} ParsingStatus_e;

typedef void (*UbxRxPacketAvailableCallback_t)(Ubx_Packet_s*);

void ubx_set_rx_pkt_cb(UbxRxPacketAvailableCallback_t rxCb);

int ubx_proto_send_pkt(Ubx_Packet_s* txPkt);

ParsingStatus_e ubx_get_parsing_status(void);

Ubx_Packet_s* ubx_get_rx_pkt(void);

void ubx_bytes_recieved(const uint8_t* rxData, uint16_t rxLen);

#endif // UBX_PROTO_H