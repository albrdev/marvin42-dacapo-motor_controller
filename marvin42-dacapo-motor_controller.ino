#include <stdarg.h>
#include <ArduinoMotorShieldR3.h>
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

void setup(void)
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

uint8_t readBuffer[512];
size_t readSize;

void HandleSerialInput(void)
{
    if(!Serial.available())
        return;

    readSize = Serial.readBytes(readBuffer, sizeof(readBuffer));
    Serial.print("Raw: size="); Serial.print(readSize); Serial.print(", hex="); Serial.println(hexstr(readBuffer, readSize));
    const uint8_t* currentOffset = readBuffer;

    Serial.println("BUFFER BEGIN");
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    while(currentOffset < readBufferEnd)
    {
        if(currentOffset + sizeof(packet_header_t) > readBufferEnd)
        {
            size_t a = (size_t)((currentOffset + sizeof(packet_header_t)) - readBuffer);
            size_t b = (size_t)(readBufferEnd - readBuffer);
            Serial.print("Header size failed: ");
            Serial.print(a); Serial.print(" / "); Serial.println(b);
            SetStatus(false);
            return;
        }

        size_t incrementSize;
        if(!HandlePacket(currentOffset, incrementSize))
        {
            Serial.println("Skipping current buffer data");
            Serial.println("");
            return;
        }

        Serial.println("");
        currentOffset += incrementSize;
    }

    Serial.println("BUFFER END");
    Serial.println("");
}

bool HandlePacket(const uint8_t* const offset, size_t& packetSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    uint16_t chksum = mkcrc16((const uint8_t * const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    Serial.print("Header: chksum_header="); Serial.print(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); Serial.print(", chksum_data="); Serial.print(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    Serial.print(", type="); Serial.print(hdr->type); Serial.print(", size="); Serial.print(hdr->size);
    Serial.print(" (chksum="); Serial.print(hexstr(&chksum, sizeof(chksum))); Serial.print(", hex="); Serial.print(hexstr(offset, sizeof(*hdr)));
    Serial.println(")");

    if(packet_verifyheader(hdr) == 0)
    {
        Serial.print("Header checksum failed: ");
        Serial.print(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); Serial.print(", "); Serial.println(hexstr(&chksum, sizeof(chksum)));
        SetStatus(false);
        return false;
    }

    if(readSize < sizeof(*hdr) + hdr->size)
    {
        Serial.print("Content size failed: ");
        Serial.print(readSize); Serial.print(" / "); Serial.println(sizeof(*hdr) + hdr->size);
        SetStatus(false);
        return false;
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

            Serial.print("CPT_MOTORRUN: left="); Serial.print(left); Serial.print(", right="); Serial.println(right);

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

            Serial.print("CPT_MOTORJSDATA: balance="); Serial.print(balance); Serial.print(", direction="); Serial.println(direction);

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
            Serial.print("Left motor speed: "); Serial.println(m1speed);
            Serial.print("Right motor speed: "); Serial.println(m2speed);
            motor.setM1Speed(m1speed);
            motor.setM2Speed(m2speed);

            size = sizeof(*pkt);
            break;
        }
        case CPT_MOTORSTOP:
        {
            Serial.println("CPT_MOTORSTOP");
            //motor.setBrakes();
            motor.setM1Speed(0);
            motor.setM2Speed(0);

            break;
        }
        default:
        {
            Serial.print("Unknown packet type: "); Serial.println(hdr->type);
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
