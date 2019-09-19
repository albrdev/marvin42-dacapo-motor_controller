#include <stdarg.h>

//#define DEBUG
#ifndef DEBUG
#include <ArduinoMotorShieldR3.h>
#else // Define dummy class if in debug/development mode
#include "ArduinoMotorShieldR3.h"
#endif

#include "src/crc.h"
#include "src/packet.h"
#include "src/custom_packets.h"
#include "src/generic.hpp"

ArduinoMotorShieldR3 motor;
#define MOTORSPEED_MAX 400

#define D4 4
#define D7 7

void SetStatus(const bool status)
{
    digitalWrite(D4, status ? HIGH : LOW);
    digitalWrite(D7, !status ? HIGH : LOW);
}

void setup()
{
    delay(2500);
    Serial.print("Initializing...");

    Serial.begin(115200);
    motor.init();

    pinMode(D4, OUTPUT);
    pinMode(D7, OUTPUT);
    SetStatus(true);

    Serial.println("Done");
}

uint8_t readBuffer[128];
size_t readSize;
void HandleSerialInput()
{
    if(!Serial.available())
        return;

    readSize = Serial.readBytes(readBuffer, sizeof(readBuffer));
    Serial.print("Raw: size="); Serial.print(readSize); Serial.print(", hex="); Serial.println(hexstr(readBuffer, readSize));

    if(readSize < sizeof(packet_header_t))
    {
        Serial.print("Header size failed: ");
        Serial.print(readSize); Serial.print(" / "); Serial.println(sizeof(packet_header_t));
        SetStatus(false);
        return;
    }

    const packet_header_t* hdr = (const packet_header_t*)readBuffer;
    uint16_t chksum = mkcrc16((const uint8_t * const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    Serial.print("Header: chksum_header="); Serial.print(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); Serial.print(", chksum_data="); Serial.print(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    Serial.print(", type="); Serial.print(hdr->type); Serial.print(", size="); Serial.print(hdr->size);
    Serial.print(" (chksum="); Serial.print(hexstr(&chksum, sizeof(chksum))); Serial.print(", hex="); Serial.print(hexstr(readBuffer, sizeof(*hdr)));
    Serial.println(")");

    if(packet_verifyheader(hdr) == 0)
    {
        Serial.print("Header checksum failed: ");
        Serial.print(hdr->chksum_header); Serial.print(", "); Serial.println(chksum);
        SetStatus(false);
        return;
    }

    if(readSize < sizeof(*hdr) + hdr->size)
    {
        Serial.print("Content size failed: ");
        Serial.print(readSize); Serial.print(" / "); Serial.println(sizeof(*hdr) + hdr->size);
        SetStatus(false);
        return;
    }

    chksum = mkcrc16((const uint8_t * const)hdr + sizeof(*hdr), hdr->size);
    Serial.print("Content: chksum_header="); Serial.print(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); Serial.print(", size="); Serial.print(hdr->size);
    Serial.print(" (chksum="); Serial.print(hexstr(&chksum, sizeof(chksum))); Serial.print(", hex="); Serial.print(hexstr((const uint8_t * const)readBuffer + sizeof(*hdr), hdr->size));
    Serial.println(")");
    if(packet_verifydata(hdr) == 0)
    {
        Serial.print("Content checksum failed: ");
        Serial.print(hdr->chksum_data); Serial.print(" / "); Serial.println(chksum);
        SetStatus(false);
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

            Serial.print("CPT_MOTORRUN: left="); Serial.print(left); Serial.print(", right="); Serial.println(right);

            motor.setM1Speed(MOTORSPEED_MAX * left);
            motor.setM2Speed(MOTORSPEED_MAX * right);
            break;
        }
        case CPT_MOTORSTOP:
        {
            Serial.println("CPT_MOTORSTOP");
            motor.setBrakes();
            break;
        }
        default:
        {
            Serial.print("Unknown packet type: "); Serial.println(hdr->type);
            break;
        }
    }

    Serial.println("");
    SetStatus(true);
}

void loop()
{
    HandleSerialInput();
}
