#include "ubx_proto.h"
#include <string.h>

static uint8_t txBuffer[MAX_EXPECTED_PAYLOAD + 8];
static uint8_t rxBuffer[MAX_EXPECTED_PAYLOAD + 8];
static uint8_t rxIndex = 0;

static Ubx_Packet_s rxPkt;

static void ubx_bytes_recieved(uint8_t* rxData, uint16_t rxLen);
static int ubx_verify_checksum(Ubx_Packet_s* pkt);
static int ubx_get_checksum(Ubx_Packet_s* pkt);
static int flush_arr(uint8_t* arrPtr, uint16_t arrLen);

int ubx_proto_send_pkt(Ubx_Packet_s* txPkt)
{
    uint8_t ck_a, ck_b;
    
    if(txPkt->payloadLen > MAX_EXPECTED_PAYLOAD)
    {
        // not enough space allocated for packet.
        return 1;
    }

    memcpy(txBuffer, (uint8_t*)(&txPkt), (txPkt->payloadLen + 6) );

    ubx_get_checksum(txPkt, &ck_a, &ck_b);

    txBuffer[(txPkt->payloadLen + 6)] = ck_a;
    txBuffer[(txPkt->payloadLen + 7)] = ck_b;

    // write with driver.
    return 0;
}

static int ubx_verify_checksum(Ubx_Packet_s* pkt)
{
    uint8_t ck_a, ck_b;
    ubx_get_checksum(pkt, &ck_a, &ck_b);    

    if( (ck_a == pkt->ck_a) && (ck_b == pkt->ck_b) )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void ubx_bytes_recieved(uint8_t* rxData, uint16_t rxLen)
{
    if( (rxLen + rxIndex) > (MAX_EXPECTED_PAYLOAD + 8) )
    {
        // not enough space in our recieve buffer... what to do?
        flush_arr(rxBuffer, (rxIndex+1) );
        return;
    }

    if(rxLen > 1)
    {
        memcpy((rxBuffer+rxIndex), rxData, rxLen );
    }
    else
    {
        rxBuffer[rxIndex] = *rxData;
    }

    // we can check for packet preamble
    if(rxIndex > 2)
    {
        // preamble
        if( (rxBuffer[0] == 0xB5) && (rxBuffer[1] == 0x62) )
        {
            // can check length
            if(rxIndex > 6)
            {
                
            }
        }
        else
        {
            // do we search for preamble?
        }
    }
}

static int ubx_get_checksum(Ubx_Packet_s* pkt, uint8_t* ck_a, uint8_t* ck_b)
{
    uint8_t ck_a_buff = 0;
    uint8_t ck_b_buff = 0;
    uint8_t* bufferPtr = &pkt->pktClass;
    uint8_t* bufferEnd = (&pkt->payload + pkt->payloadLen);
    while(bufferPtr != bufferEnd)
    {
        ck_a_buff = ck_a_buff + *bufferPtr;
        ck_b_buff = ck_b_buff + ck_a_buff;
        bufferPtr++;
    }

    *ck_a = ck_a_buff;
    *ck_b = ck_b_buff; 

    return 0;
}

static int flush_arr(uint8_t* arrPtr, uint16_t arrLen)
{
    while(arrPtr != (arrPtr + arrLen) )
    {
        *arrPtr = 0;
        arrPtr++;
    }
}