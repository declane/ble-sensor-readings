#include <stdio.h>
#include <stdint.h>
#include "../ubx_proto.h"

#define UBX_LOCAL_UNIT_TESTING

// UBX-NAV-PVT example packet (92-byte payload)
// Date: 2026-02-13 12:00:00 UTC
// Location: Randolph Hall, Blacksburg, VA
// Stationary, 3D fix

const unsigned char ubx_nav_pvt[] = {

    // UBX header
    0xB5, 0x62,

    // Class, ID
    0x01, 0x07,

    // Length (92 bytes)
    0x5C, 0x00,

    // ---------------- Payload ----------------

    // iTOW
    0x00, 0x00, 0x00, 0x00,

    // Year = 2026
    0xEA, 0x07,

    // Month, Day, Hour, Min, Sec
    0x02, 0x0D, 0x0C, 0x00, 0x00,

    // Valid
    0x07,

    // tAcc
    0xA0, 0x86, 0x01, 0x00,

    // nano
    0x00, 0x00, 0x00, 0x00,

    // fixType, flags, flags2, numSV
    0x03, 0x01, 0x00, 0x0C,

    // Longitude (-80.4244 deg)
    0xE0, 0x35, 0x10, 0xD0,

    // Latitude (37.2296 deg)
    0x40, 0xC9, 0x30, 0x16,

    // Height (640 m)
    0x00, 0xC4, 0x09, 0x00,

    // hMSL
    0x00, 0xC4, 0x09, 0x00,

    // hAcc
    0xE8, 0x03, 0x00, 0x00,

    // vAcc
    0xE8, 0x03, 0x00, 0x00,

    // velN
    0x00, 0x00, 0x00, 0x00,

    // velE
    0x00, 0x00, 0x00, 0x00,

    // velD
    0x00, 0x00, 0x00, 0x00,

    // gSpeed
    0x00, 0x00, 0x00, 0x00,

    // headMot
    0x00, 0x00, 0x00, 0x00,

    // sAcc
    0x64, 0x00, 0x00, 0x00,

    // headAcc
    0xA0, 0x86, 0x01, 0x00,

    // pDOP
    0x96, 0x00,

    // flags3
    0x00,

    // reserved1[6]
    0x00, 0x00, 0x00, 0x00, 0x00,

    // headVeh
    0x00, 0x00, 0x00, 0x00,

    // magDec
    0x00, 0x00,

    // magAcc
    0x00, 0x00,

    // ---------------- Checksum ----------------
    0x83, 0x9F
};

const unsigned char ubx_nav_pvt_junk_in_front[] = {

    // Bytes to be ignored
    0x01, 0x05, 0x06, 0x99, 0xFF,
    
    // UBX header
    0xB5, 0x62,

    // Class, ID
    0x01, 0x07,

    // Length (92 bytes)
    0x5C, 0x00,

    // ---------------- Payload ----------------

    // iTOW
    0x00, 0x00, 0x00, 0x00,

    // Year = 2026
    0xEA, 0x07,

    // Month, Day, Hour, Min, Sec
    0x02, 0x0D, 0x0C, 0x00, 0x00,

    // Valid
    0x07,

    // tAcc
    0xA0, 0x86, 0x01, 0x00,

    // nano
    0x00, 0x00, 0x00, 0x00,

    // fixType, flags, flags2, numSV
    0x03, 0x01, 0x00, 0x0C,

    // Longitude (-80.4244 deg)
    0xE0, 0x35, 0x10, 0xD0,

    // Latitude (37.2296 deg)
    0x40, 0xC9, 0x30, 0x16,

    // Height (640 m)
    0x00, 0xC4, 0x09, 0x00,

    // hMSL
    0x00, 0xC4, 0x09, 0x00,

    // hAcc
    0xE8, 0x03, 0x00, 0x00,

    // vAcc
    0xE8, 0x03, 0x00, 0x00,

    // velN
    0x00, 0x00, 0x00, 0x00,

    // velE
    0x00, 0x00, 0x00, 0x00,

    // velD
    0x00, 0x00, 0x00, 0x00,

    // gSpeed
    0x00, 0x00, 0x00, 0x00,

    // headMot
    0x00, 0x00, 0x00, 0x00,

    // sAcc
    0x64, 0x00, 0x00, 0x00,

    // headAcc
    0xA0, 0x86, 0x01, 0x00,

    // pDOP
    0x96, 0x00,

    // flags3
    0x00,

    // reserved1[6]
    0x00, 0x00, 0x00, 0x00, 0x00,

    // headVeh
    0x00, 0x00, 0x00, 0x00,

    // magDec
    0x00, 0x00,

    // magAcc
    0x00, 0x00,

    // ---------------- Checksum ----------------
    0x83, 0x9F
};

//#define FULL_MESSAGE_TEST
//#define HALF_MESSAGE_TEST
//#define BYTE_BY_BYTE_MESSAGE_TEST


const uint16_t ubx_nav_pvt_len = sizeof(ubx_nav_pvt);
const uint16_t junk_in_front_len = sizeof(ubx_nav_pvt_junk_in_front);

static void print_parsing_result(ParsingStatus_e result);

static int assert_data_correct(Ubx_Packet_s* pkt);

int main(void)
{
    ParsingStatus_e parsingStatus;

#ifdef FULL_MESSAGE_TEST
    /** ***************************************************
     *  Test 1 - Parse entire message
     * ****************************************************
     */
    printf(" ******************* Full Message Test Start *******************\n");
    ubx_bytes_recieved(ubx_nav_pvt, ubx_nav_pvt_len);
    parsingStatus = ubx_get_parsing_status();

    if(parsingStatus == Parsing_Complete)
    {
        if(assert_data_correct(ubx_get_rx_pkt()))
        {
            printf("Full Parse Success!\n\n");
        }
        else
        {
            printf("FULL PARSE TEST FAILED! Error in parsed packet!\n");
        }       
        
    }
    else
    {
        printf("FULL PARSE TEST FAILED! See parsing status:\n");
        print_parsing_result(parsingStatus);
    }
    printf(" ******************* Full Message Test End *******************\n\n\n");
#endif

#ifdef HALF_MESSAGE_TEST
    /** ***************************************************
     *  Test 2 - Parse message in two chunks
     * ****************************************************
     */
    printf(" ******************* Half Message Test Start *******************\n");
    ubx_bytes_recieved(ubx_nav_pvt, ubx_nav_pvt_len/2);
    parsingStatus = ubx_get_parsing_status();

    if(parsingStatus == Parsing_Active)
    {
        printf("First half looks good...\n");
        ubx_bytes_recieved( (ubx_nav_pvt + ubx_nav_pvt_len/2), ubx_nav_pvt_len/2);
        parsingStatus = ubx_get_parsing_status();
        if(parsingStatus == Parsing_Complete)
        {
            if(assert_data_correct(ubx_get_rx_pkt()))
            {
                printf("Half Parse Success!\n\n");
            }
            else
            {
                printf("HALF PARSE TEST FAILED! Error in parsed packet!\n");
            } 
        }
        else
        {
            printf("HALF PARSE TEST FAILED ON SECOND HALF! See parsing status:\n");
            print_parsing_result(parsingStatus);
        }
    }
    else
    {
        printf("HALF PARSE TEST FAILED ON FIRST HALF! See parsing status:\n");
        print_parsing_result(parsingStatus);
    }
    printf(" ******************* Half Message Test End *******************\n\n\n");
#endif

#ifdef BYTE_BY_BYTE_MESSAGE_TEST
    /** ***************************************************
     *  Test 3 - Write a single byte at a time
     *           consider commenting out print statements
     *           in UBX module here, could flood terminal.
     * ****************************************************
     */
    printf(" ******************* Byte by Byte Message Test Start *******************\n");
    int i, errorFlag = 0;
    for(i = 0; i < ubx_nav_pvt_len; i++)
    {
        ubx_bytes_recieved( (ubx_nav_pvt + i), 1);
        parsingStatus = ubx_get_parsing_status();

        if( (parsingStatus != Parsing_Active) && (i < (ubx_nav_pvt_len-1)) )
        {
            errorFlag = 1;
            break;
        }
        else if( (parsingStatus != Parsing_Complete) && (i == (ubx_nav_pvt_len-1)) )
        {
            errorFlag = 1;
            break;
        }
    }

    if(errorFlag)
    {
        printf("BYTE BY BYTE TEST FAILED DUE TO BAD STATUS AT BYTE %d\n! See parsing status:\n", i);
        print_parsing_result(parsingStatus);
    }
    else
    {
        if(assert_data_correct(ubx_get_rx_pkt()))
        {
            printf("Byte by Byte Parse Success!\n\n");
        }
        else
        {
            printf("BYTE BY BYTE TEST FAILED! Error in parsed packet!\n");
        }
    }

    printf(" ******************* Byte by Byte Message Test End *******************\n");
#endif

    return 0;
}

static void print_parsing_result(ParsingStatus_e result)
{
    switch(result)
    {
        case Parsing_Idle:
            printf("\tParsing Idle\n\n");
            break;
        case Parsing_Active:
            printf("\tParsing Active\n\n");
            break;
        case Parsing_Complete:
            printf("\tParsing Complete\n\n");
            break;
        case Parsing_Timeout:
            printf("\tParsing Timeout\n\n");
            break;
        case Parsing_Checksum_Error:
            printf("\tChecksum Error\n\n");
            break;
        case Parsing_Length_Limit:
            printf("\tLength in packet Exceeded Limit\n\n");
            break;
        default:
            printf("\tUnknown Parsing Status.\n\n");

    }
}

/**
 * @retval: 0 - failure
 * @retval: 1 - success
 */
static int assert_data_correct(Ubx_Packet_s* pkt)
{
    if( pkt->syncChar_1 != UBX_SYNC1_CHAR )
    {
        printf("Parsed packet sync 1 char incorrect\n");
        return 0;
    }
    if( pkt->syncChar_2 != UBX_SYNC2_CHAR )
    {
        printf("Parsed packet sync 2 char incorrect\n");
        return 0;
    }
    if( pkt->pktClass != 0x01 )
    {
        printf("Parsed packet class incorrect\n");
        return 0;
    }
    if( pkt->pktId != 0x07 )
    {
        printf("Parsed packet id incorrect\n");
        return 0;
    }  
    if( pkt->payloadLen != 92 )
    {
        printf("Parsed packet payload incorrect\n");
        return 0;
    }
    else
    {
        return 1;
    }
}