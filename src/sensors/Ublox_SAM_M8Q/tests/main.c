#include <stdio.h>
#include <stdint.h>
#include "../ubx_proto.h"

const uint8_t ubx_nav_pvt[] = {
    0xB5, 0x62,             // Sync chars
    0x01, 0x07,             // Class, ID (NAV-PVT)
    0x5C, 0x00,             // Length (92 bytes)

    // Payload (92 bytes)
    0x00, 0x00, 0x00, 0x00, // iTOW
    0xE8, 0x07,             // year = 2024
    0x01,                   // month
    0x01,                   // day
    0x00,                   // hour
    0x00,                   // min
    0x00,                   // sec
    0x07,                   // valid
    0x00, 0x00, 0x00, 0x00, // tAcc
    0x00, 0x00, 0x00, 0x00, // nano
    0x03,                   // fixType = 3D
    0x00,                   // flags
    0x00,                   // flags2
    0x00,                   // numSV
    0x00, 0x00, 0x00, 0x00, // lon
    0x00, 0x00, 0x00, 0x00, // lat
    0x00, 0x00, 0x00, 0x00, // height
    0x00, 0x00, 0x00, 0x00, // hMSL
    0x00, 0x00, 0x00, 0x00, // hAcc
    0x00, 0x00, 0x00, 0x00, // vAcc
    0x00, 0x00, 0x00, 0x00, // velN
    0x00, 0x00, 0x00, 0x00, // velE
    0x00, 0x00, 0x00, 0x00, // velD
    0x00, 0x00, 0x00, 0x00, // gSpeed
    0x00, 0x00, 0x00, 0x00, // headMot
    0x00, 0x00, 0x00, 0x00, // sAcc
    0x00, 0x00, 0x00, 0x00, // headAcc
    0x00, 0x00,             // pDOP
    0x00, 0x00,             // flags3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Checksum
    0x7B, 0xA2
};

const uint16_t ubx_nav_pvt_len = sizeof(ubx_nav_pvt);

static void print_parsing_result(ParsingStatus_e result);

int main(void)
{
    ParsingStatus_e parsingStatus;
    /** ***************************************************
     *  Test 1 - Parse enter message
     * ****************************************************
     */
    printf("Passing full message to ubx parser...\n");
    ubx_bytes_recieved(ubx_nav_pvt, ubx_nav_pvt_len);
    parsingStatus = ubx_get_parsing_status();

    if(parsingStatus == Parsing_Complete)
    {
        printf("Full Parse Success!\n\n");
    }
    else
    {
        printf("FULL PARSE TEST FAILED! See parsing status:\n");
        print_parsing_result(parsingStatus);
    }
    


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
