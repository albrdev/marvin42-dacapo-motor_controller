#include <stdint.h>
#include <stdarg.h>
#include "DCMotorAssembly.hpp"
#include "crc.h"
#include "packet.h"
#include "custom_packets.h"
#include "generic.hpp"

DCMotorAssembly motors;

#define D4 4
#define D7 7

void SetStatus(const bool status)
{
    digitalWrite(D4, status ? HIGH : LOW);
    digitalWrite(D7, !status ? HIGH : LOW);
}

uint8_t readBuffer[512];
size_t readOffset = 0U;
#define INVALID_SIZE SIZE_MAX

void OnPacketReceived(const packet_header_t* const hdr)
{
    switch(hdr->type)
    {
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)hdr;
            float left;
            float right;
            memcpy(&left, &pkt->left, sizeof(pkt->left));
            memcpy(&right, &pkt->right, sizeof(pkt->right));

            PrintDebug("CPT_MOTORRUN: left="); PrintDebug(left); PrintDebug(", right="); PrintDebugLine(right);

            motors.SetLeftSpeed(left);
            motors.SetRightSpeed(right);

            break;
        }
        case CPT_MOTORRUN2:
        {
            const packet_motorrun2_t* pkt = (const packet_motorrun2_t*)hdr;
            float direction[2];
            float power;
            memcpy(&direction[0], &pkt->direction.x, sizeof(pkt->direction.x));
            memcpy(&direction[1], &pkt->direction.y, sizeof(pkt->direction.y));
            memcpy(&power, &pkt->power, sizeof(pkt->power));

            PrintDebug("CPT_MOTORRUN2: direction="); PrintDebug("(x="); PrintDebug(direction[0]); PrintDebug(", y="); PrintDebug(direction[1]); PrintDebug(")");
            PrintDebug(", power="); PrintDebugLine(power);

            motors.Run(direction[0], direction[1], power);

            break;
        }
        case CPT_MOTORSTOP:
        {
            PrintDebugLine("CPT_MOTORSTOP");
            motors.Halt();

            break;
        }
        default:
        {
            PrintDebug("Unknown packet type: "); PrintDebugLine(hdr->type);
            break;
        }
    }
}

bool handle_packet(const uint8_t* const offset, const uint8_t* const end, size_t* const incrementSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    *incrementSize = INVALID_SIZE;

    #ifdef M42_DEBUG
    uint16_t chksum = mkcrc16((const uint8_t* const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    #endif
    PrintDebug("Header: chksum_header="); PrintDebug(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug(", chksum_data="); PrintDebug(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    PrintDebug(", type="); PrintDebug(hdr->type); PrintDebug(", size="); PrintDebug(hdr->size);
    PrintDebug(" (chksum="); PrintDebug(hexstr(&chksum, sizeof(chksum))); PrintDebug(", hex="); PrintDebug(hexstr(offset, sizeof(*hdr)));
    PrintDebugLine(")");

    if(packet_verifyheader(hdr) == 0)
    {
        PrintDebug("Header checksum failed: ");
        PrintDebug(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug(", "); PrintDebugLine(hexstr(&chksum, sizeof(chksum)));
        return false;
    }

    if(offset + (sizeof(*hdr) + hdr->size) > end)
    {
        *incrementSize = 0U;
        return false;
    }

    #ifdef M42_DEBUG
    chksum = mkcrc16((const uint8_t* const)hdr + sizeof(*hdr), hdr->size);
    #endif
    PrintDebug("Content: chksum_data="); PrintDebug(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); PrintDebug(", size="); PrintDebug(hdr->size);
    PrintDebug(" (chksum="); PrintDebug(hexstr(&chksum, sizeof(chksum))); PrintDebug(", hex="); PrintDebug(hexstr((const uint8_t* const)offset + sizeof(*hdr), hdr->size));
    PrintDebugLine(")");

    if(packet_verifydata(hdr) == 0)
    {
        PrintDebug("Content checksum failed: ");
        PrintDebug(hdr->chksum_data); PrintDebug(" / "); PrintDebugLine(chksum);
        return false;
    }

    *incrementSize = sizeof(*hdr) + hdr->size;
    OnPacketReceived(hdr);
    return true;
}

void handle_data(void)
{
    if(Serial.available() <= 0)
        return;

    if(readOffset >= sizeof(readBuffer))
    {
        // Should/could only happen if packet received claims it has an abnormally large data size, i.e. buffer overflow attack or possibly a very uncautious programmer either sending too large data or has reduced the receive buffer size lower than the packet that is currently receiving
        PrintDebug("Buffer overflow: offset="); PrintDebug(readOffset); PrintDebug(", max="); PrintDebug(sizeof(readBuffer));
        readOffset = 0U; // Ignore this rubbish packet
    }

    size_t readSize = Serial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
    readSize += readOffset;
    readOffset = 0U;

    PrintDebug("Raw: size="); PrintDebug(readSize); PrintDebug(", hex="); PrintDebugLine(hexstr(readBuffer, readSize));

    PrintDebugLine("BUFFER BEGIN");
    size_t indexOffset = 0U;
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    size_t incrementSize;
    while(indexOffset + sizeof(packet_header_t) < readSize)
    {
        if(!handle_packet(readBuffer + indexOffset, readBufferEnd, &incrementSize))
        {
            break;
        }

        PrintDebugLine("");
        indexOffset += incrementSize;
    }

    if(incrementSize == INVALID_SIZE)
    {
        PrintDebugLine("BUFFER ERROR");
        PrintDebugLine("");
        //PacketFailCount++;
        SetStatus(false);
        return;
    }
    else
    {
        //PacketSuccessCount++;
        SetStatus(true);
    }


    readOffset = readSize - indexOffset;
    for(size_t i = 0U; i < readOffset; i++)
    {
        readBuffer[i] = readBuffer[indexOffset + i];
    }

    PrintDebugLine("BUFFER END");
    PrintDebugLine("");
}

void setup(void)
{
    delay(2500);
    Serial.begin(9600, SERIAL_8N1);
    Serial.print("Initializing...");

    motors.Begin();

    pinMode(D4, OUTPUT);
    pinMode(D7, OUTPUT);
    SetStatus(true);

    Serial.println("Done");
}

void loop(void)
{
    handle_data();
}
