#include <stdarg.h>
#include <SoftwareSerial.h>
#include <ArduinoMotorShieldR3.h>
#include "src/crc.h"
#include "src/packet.h"
#include "src/custom_packets.h"

//#define M42_DEBUG // Must define before including 'generic.hpp'
#include "src/generic.hpp"

SoftwareSerial serial(5, 6);
ArduinoMotorShieldR3 motor;
#define MOTORSPEED_MAX 400

#define D4 4
#define D7 7

void SetStatus(const bool status)
{
    digitalWrite(D4, status ? HIGH : LOW);
    digitalWrite(D7, !status ? HIGH : LOW);
}

void setup(void)
{
    delay(2500);
    Serial.print("Initializing...");

    Serial.begin(9600);
    serial.begin(115200);
    motor.init();

    pinMode(D4, OUTPUT);
    pinMode(D7, OUTPUT);
    SetStatus(true);

    Serial.println("Done");
}

uint8_t readBuffer[512];
size_t readSize;

void HandleSerialInput(void)
{
    if(!serial.available())
        return;

    readSize = serial.readBytes(readBuffer, sizeof(readBuffer));
    PrintDebug("Raw: size="); PrintDebug(readSize); PrintDebug(", hex="); PrintDebugLine(hexstr(readBuffer, readSize));
    const uint8_t* currentOffset = readBuffer;

    PrintDebugLine("BUFFER BEGIN");
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    while(currentOffset < readBufferEnd)
    {
        if(currentOffset + sizeof(packet_header_t) > readBufferEnd)
        {
            size_t a = (size_t)((currentOffset + sizeof(packet_header_t)) - readBuffer);
            size_t b = (size_t)(readBufferEnd - readBuffer);
            PrintDebug("Header size failed: ");
            PrintDebug(a); PrintDebug(" / "); PrintDebugLine(b);
            SetStatus(false);
            return;
        }

        size_t incrementSize;
        if(!HandlePacket(currentOffset, incrementSize))
        {
            PrintDebugLine("Skipping current buffer data");
            PrintDebugLine("");
            return;
        }

        PrintDebugLine("");
        currentOffset += incrementSize;
    }

    PrintDebugLine("BUFFER END");
    PrintDebugLine("");
}

bool HandlePacket(const uint8_t* const offset, size_t& packetSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    uint16_t chksum = mkcrc16((const uint8_t * const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    PrintDebug("Header: chksum_header="); PrintDebug(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug(", chksum_data="); PrintDebug(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    PrintDebug(", type="); PrintDebug(hdr->type); PrintDebug(", size="); PrintDebug(hdr->size);
    PrintDebug(" (chksum="); PrintDebug(hexstr(&chksum, sizeof(chksum))); PrintDebug(", hex="); PrintDebug(hexstr(offset, sizeof(*hdr)));
    PrintDebugLine(")");

    if(packet_verifyheader(hdr) == 0)
    {
        PrintDebug("Header checksum failed: ");
        PrintDebug(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug(", "); PrintDebugLine(hexstr(&chksum, sizeof(chksum)));
        SetStatus(false);
        return false;
    }

    if(readSize < sizeof(*hdr) + hdr->size)
    {
        PrintDebug("Content size failed: ");
        PrintDebug(readSize); PrintDebug(" / "); PrintDebugLine(sizeof(*hdr) + hdr->size);
        SetStatus(false);
        return false;
    }

    chksum = mkcrc16((const uint8_t * const)hdr + sizeof(*hdr), hdr->size);
    PrintDebug("Content: chksum_header="); PrintDebug(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); PrintDebug(", size="); PrintDebug(hdr->size);
    PrintDebug(" (chksum="); PrintDebug(hexstr(&chksum, sizeof(chksum))); PrintDebug(", hex="); PrintDebug(hexstr((const uint8_t * const)readBuffer + sizeof(*hdr), hdr->size));
    PrintDebugLine(")");
    if(packet_verifydata(hdr) == 0)
    {
        PrintDebug("Content checksum failed: ");
        PrintDebug(hdr->chksum_data); PrintDebug(" / "); PrintDebugLine(chksum);
        SetStatus(false);
        return false;
    }

    int size = sizeof(*hdr) + hdr->size;
    switch(hdr->type)
    {
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)offset;
            float left;
            float right;
            memcpy(&left, &pkt->left, sizeof(pkt->left));
            memcpy(&right, &pkt->right, sizeof(pkt->right));

            PrintDebug("CPT_MOTORRUN: left="); PrintDebug(left); PrintDebug(", right="); PrintDebugLine(right);

            motor.setM1Speed(MOTORSPEED_MAX * left);
            motor.setM2Speed(MOTORSPEED_MAX * right);

            size = sizeof(*pkt);
            break;
        }
        case CPT_MOTORJSDATA:
        {
            const packet_motorjsdata_t* pkt = (const packet_motorjsdata_t*)offset;
            float balance;
            int8_t direction;
            memcpy(&balance, &pkt->balance, sizeof(pkt->balance));
            memcpy(&direction, &pkt->direction, sizeof(pkt->direction));

            PrintDebug("CPT_MOTORJSDATA: balance="); PrintDebug(balance); PrintDebug(", direction="); PrintDebugLine(direction);

            if(direction == 0)
            {
                //motor.setBrakes();
                motor.setM1Speed(0);
                motor.setM2Speed(0);
                break;
            }

            float x = denormalize11(balance, 0, MOTORSPEED_MAX);
            float m1speed = (MOTORSPEED_MAX - x) * direction;
            float m2speed = x * direction;
            PrintDebug("Left motor speed: "); PrintDebugLine(m1speed);
            PrintDebug("Right motor speed: "); PrintDebugLine(m2speed);
            motor.setM1Speed(m1speed);
            motor.setM2Speed(m2speed);

            size = sizeof(*pkt);
            break;
        }
        case CPT_MOTORSTOP:
        {
            PrintDebugLine("CPT_MOTORSTOP");
            //motor.setBrakes();
            motor.setM1Speed(0);
            motor.setM2Speed(0);

            break;
        }
        default:
        {
            PrintDebug("Unknown packet type: "); PrintDebugLine(hdr->type);
            break;
            //return -1;
        }
    }

    packetSize = size;
    SetStatus(true);
    return true;
}

void loop(void)
{
    HandleSerialInput();
}
