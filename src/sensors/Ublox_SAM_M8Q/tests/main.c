#include <stdio.h>
#include <stdint.h>
#include "../ubx_proto.h"

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
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // headVeh
    0x00, 0x00, 0x00, 0x00,

    // magDec
    0x00, 0x00,

    // magAcc
    0x00, 0x00,

    // ---------------- Checksum ----------------
    0x83, 0x22
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
