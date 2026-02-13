#include "ubx_proto.h"
#include <string.h>
#include <stdio.h>

static ParsingStatus_e parsingStatus = Parsing_Idle;
static uint8_t txBuffer[MAX_EXPECTED_PAYLOAD + 8];
static uint8_t rxBuffer[MAX_EXPECTED_PAYLOAD*2 + 8*2];
static uint16_t rxIndex = 0;

static Ubx_Packet_s rxPkt;

static int ubx_verify_checksum(Ubx_Packet_s* pkt);
static int ubx_get_checksum(Ubx_Packet_s* pkt, uint8_t* ck_a, uint8_t* ck_b);
static int flush_arr(uint8_t* arrPtr, uint16_t arrLen);
static uint16_t get_length_from_arr(uint8_t* arrPtr);

int ubx_proto_send_pkt(Ubx_Packet_s* txPkt)
{
    uint8_t ck_a, ck_b;
    
    if(txPkt->payloadLen > MAX_EXPECTED_PAYLOAD)
    {
        // not enough space allocated for packet.
        return 1;
    }

    memcpy(txBuffer, (uint8_t*)(txPkt), (txPkt->payloadLen + 6) );

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
    
    printf("[UBX CHKSUM] Packet     | ck_a = %d - ck_b = %d\n", pkt->ck_a, pkt->ck_b);
    printf("[UBX CHKSUM] Calculated | ck_a = %d - ck_b = %d\n", ck_a, ck_b);   

    if( (ck_a == pkt->ck_a) && (ck_b == pkt->ck_b) )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

ParsingStatus_e ubx_get_parsing_status(void)
{
    return parsingStatus;
}

Ubx_Packet_s* ubx_get_rx_pkt(void)
{
    return &rxPkt;
}

void ubx_bytes_recieved(const uint8_t* rxData, uint16_t rxLen)
{
    uint16_t tempLen, i;

    if( (rxLen + rxIndex) > (MAX_EXPECTED_PAYLOAD*2 + 8*2) )
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
        printf("[UBX] Enough bytes for preamble\n");
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
            printf("[UBX] No Preamble found. Returning\n");
            flush_arr(rxBuffer, rxIndex);
            rxIndex = 0;
            parsingStatus = Parsing_Idle;
            return;
        }

        printf("[UBX] Preamble found at index: %d\n", i);

        // we can get payload length
        if(rxIndex > 6)
        {
            printf("[UBX] Enough bytes for length\n");
            tempLen = (uint16_t)rxBuffer[4] | ((uint16_t)rxBuffer[5] << 8); 
            printf("[UBX] Found Length to be: %d\n", tempLen);
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

            printf("[UBX] Rx Buffer Index is at %d. Needs to be %d for full message\n", rxIndex, (tempLen+8));
            if(rxIndex >= tempLen + 8)
            {
                printf("[UBX] Entire packet detected. Verifying checksum...\n");
                // whole packet came in. Yay
                memcpy((uint8_t*)&rxPkt, rxBuffer, rxIndex);
                rxPkt.ck_a = rxBuffer[rxIndex-2];
                rxPkt.ck_b = rxBuffer[rxIndex-1];

                if(ubx_verify_checksum(&rxPkt))
                {
                    parsingStatus = Parsing_Complete;
                    flush_arr(rxBuffer, rxIndex);
                    rxIndex = 0;
                }
                else
                {
                    parsingStatus = Parsing_Checksum_Error;
                    flush_arr(rxBuffer, rxIndex);
                    rxIndex = 0;
                }
            }// enogh bytes for whole packet
        } // enough bytes for length
    } // enough bytes for preamble
    else
    {
        // a single byte has arrived... can't say if it is a packet
        // or not but possible.
        parsingStatus = Parsing_Active;
    }
}

static int ubx_get_checksum(Ubx_Packet_s* pkt, uint8_t* ck_a, uint8_t* ck_b)
{
    uint8_t ck_a_buff = 0;
    uint8_t ck_b_buff = 0;
    uint8_t* bufferPtr = &pkt->pktClass;
    uint8_t* bufferEnd = (pkt->payload + pkt->payloadLen);
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