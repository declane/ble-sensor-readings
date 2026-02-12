#include "ubx_proto.h"
#include <string.h>

static ParsingStatus_e parsingStatus = Parsing_Idle;
static uint8_t txBuffer[MAX_EXPECTED_PAYLOAD + 8];
static uint8_t rxBuffer[MAX_EXPECTED_PAYLOAD + 8];
static uint8_t rxIndex = 0;

static Ubx_Packet_s rxPkt;

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

void ubx_bytes_recieved(uint8_t* rxData, uint16_t rxLen)
{
    uint16_t tempLen, i;
    if( (rxLen + rxIndex) > (MAX_EXPECTED_PAYLOAD + 8) )
    {
        // not enough space in our recieve buffer... what to do?
        flush_arr(rxBuffer, rxIndex);
        return;
    }

    if(rxLen > 1)
    {
        memcpy((rxBuffer+rxIndex), rxData, rxLen );
    }
    else if(rxLen == 1)
    {
        rxBuffer[rxIndex] = *rxData;
    }
    else
    {
        return;
    }
    rxIndex += rxLen;

    // we can check for packet preamble
    if(rxIndex > 2)
    {
        // search for preamble
        for(i = 0; i < (rxIndex-1); i++)
        {
            if(rxBuffer[i] == UBX_SYNC1_CHAR)
            {
                if(rxBuffer[i+1] == UBX_SYNC2_CHAR)
                {
                    break;
                }
            }
        }

        if(!i)
        {
            // Ideal case where preamble is at start.
            parsingStatus = Parsing_Active;
        }
        else if(i < (rxIndex-1) )
        {
            // found preamble not at start...
            // how do we shift everything down?
            memset(rxBuffer, 0, i);
            memcpy(rxBuffer, (rxBuffer+i), rxIndex-i);
            rxIndex -= i;
            parsingStatus = Parsing_Active;
        }
        else
        {
            flush_arr(rxBuffer, rxIndex);
            rxIndex = 0;
            parsingStatus = Parsing_Idle;
            return;
        }

        // we can get payload length
        if(rxIndex > 6)
        {
            tempLen = (uint16_t)rxBuffer[4] | ((uint16_t)rxBuffer[5] << 8); 
            if(tempLen > MAX_EXPECTED_PAYLOAD)
            {
                /**
                 * Might not be necessary. In the past if there is in error in 
                 * parsing or the packet recieved, the length might become some very
                 * large, unexpected number that does not match the packet size. 
                 * Trying to avoid that here.
                 */
                parsingStatus = Parsing_Length_Limit;
                return;
            }

            if(rxIndex >= tempLen + 8)
            {
                // whole packet came in. Yay
                memcpy((uint8_t*)&rxPkt, rxBuffer, rxIndex);
                rxPkt.ck_a = rxBuffer[rxIndex-2];
                rxPkt.ck_b = rxBuffer[rxIndex-1];

                if(ubx_verify_checksum(&rxPkt))
                {
                    parsingStatus = Parsing_Complete;
                }
                else
                {
                    parsingStatus = Parsing_Checksum_Error;
                }
            }
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
    memset(arrPtr, 0, arrLen);

}