#include <stdarg.h>

//#define DEBUG
#ifndef DEBUG
#include "ArduinoMotorShieldR3.h"
#else // Define dummy class if in debug/development mode
#include "src/ArduinoMotorShieldR3.h"
#endif

#include "src/crc.h"
#include "src/packet.h"
#include "src/custom_packets.h"
#include "src/generic.hpp"

ArduinoMotorShieldR3 motor;
#define MOTORSPEED_MAX 400

void setup()
{
    Serial.print("Initializing...");

    Serial.begin(115200);
    motor.init();

    Serial.println("Done");
}

uint8_t readBuffer[128];
size_t readSize;
void HandleSerialInput()
{
    if(!Serial.available())
        return;

    readSize = Serial.readBytes(readBuffer, sizeof(readBuffer));

    spprintf("Raw: size=%zu, hex=%s\n", readSize, hexstr(readBuffer, readSize));

    if(readSize < sizeof(packet_header_t))
    {
        spprintf("Header size failed: %zu\n", readSize);
        return;
    }

    const packet_header_t* hdr = (const packet_header_t*)readBuffer;
    uint16_t chksum = mkcrc16((const uint8_t * const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    spprintf("Header:   chksum_header=%hX, chksum_data=%hX, type=%hhu, size=%hu (chksum=%hX, hex=%s)\n", hdr->chksum_header, hdr->chksum_data, hdr->type, hdr->size, chksum, hexstr(readBuffer, sizeof(*hdr)));

    if(packet_verifyheader(hdr) == 0)
    {
        spprintf("Header checksum failed: %hu / %hu\n", hdr->chksum_header, chksum);
        return;
    }

    if(readSize < sizeof(*hdr) + hdr->size)
    {
        spprintf("Content size failed: %zu\n", readSize);
        return;
    }

    chksum = mkcrc16((const uint8_t * const)hdr + sizeof(*hdr), hdr->size);
    spprintf("Content:  chksum_data=%hX, size=%zu (chksum=%hX, hex=%s)\n", hdr->chksum_data, hdr->size, chksum, hexstr((const uint8_t * const)readBuffer + sizeof(*hdr), hdr->size));
    if(packet_verifydata(hdr) == 0)
    {
        spprintf("Content checksum failed: %hu / %hu\n", hdr->chksum_data, chksum);
        return;
    }

    switch(hdr->type)
    {
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)readBuffer;
            float left, right;
            memcpy(&left, &pkt->left, sizeof(pkt->left));
            memcpy(&right, &pkt->right, sizeof(pkt->right));

            spprintf("CPT_MOTORRUN: left=%.2f, right=%.2f\n", left, right);

            motor.setM1Speed(MOTORSPEED_MAX * left);
            motor.setM2Speed(MOTORSPEED_MAX * right);
            break;
        }
        case CPT_MOTORSTOP:
        {
            spprintf("CPT_MOTORSTOP\n");
            motor.setBrakes();
            break;
        }
        default:
        {
            spprintf("Unknown packet type: %hhu\n", hdr->type);
            break;
        }
    }
}

void loop()
{
    HandleSerialInput();
}
