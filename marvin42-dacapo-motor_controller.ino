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

#define CommandSerial Serial2

void SetStatus(const bool status)
{
    digitalWrite(D4, status ? HIGH : LOW);
    digitalWrite(D7, !status ? HIGH : LOW);
}

struct
{
    vector2data_t direction;
    float power;
} inputdata = { { 0.0f, 0.0f }, 0.0f };

uint8_t readBuffer[512];
size_t readOffset = 0U;
#define INVALID_SIZE SIZE_MAX

size_t packetSuccessCount = 0U;
size_t packetFailCount = 0U;

void OnPacketReceived(const packet_header_t* const hdr)
{
    switch(hdr->type)
    {
        case CPT_MOTORBALANCE:
        {
            const packet_motorbalance_t* pkt = (const packet_motorbalance_t*)hdr;
            float left;
            float right;
            memcpy(&left, &pkt->left, sizeof(pkt->left));
            memcpy(&right, &pkt->right, sizeof(pkt->right));

            PrintDebug("CPT_MOTORBALANCE: left="); PrintDebug(left); PrintDebug(", right="); PrintDebugLine(right);

            motors.SetLeftSpeed(left);
            motors.SetRightSpeed(right);

            break;
        }
        case CPT_DIRECTION:
        {
            const packet_direction_t* pkt = (const packet_direction_t*)hdr;
            memcpy(&inputdata.direction, &pkt->direction, sizeof(pkt->direction));

            PrintDebug("CPT_DIRECTION: direction="); PrintDebug("(x="); PrintDebug(inputdata.direction.x); PrintDebug(", y="); PrintDebug(inputdata.direction.y); PrintDebug(")");
            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORPOWER:
        {
            const packet_motorpower_t* pkt = (const packet_motorpower_t*)hdr;
            memcpy(&inputdata.power, &pkt->power, sizeof(pkt->power));

            PrintDebug("CPT_MOTORPOWER: power="); PrintDebugLine(inputdata.power);
            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)hdr;
            memcpy(&inputdata.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.power, &pkt->power, sizeof(pkt->power));

            PrintDebug("CPT_MOTORRUN: direction="); PrintDebug("(x="); PrintDebug(inputdata.direction.x); PrintDebug(", y="); PrintDebug(inputdata.direction.y); PrintDebug(")");
            PrintDebug(", power="); PrintDebugLine(inputdata.power);

            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORSTOP:
        {
            inputdata = { { 0.0f, 0.0f }, 0.0f };
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
    if(CommandSerial.available() <= 0)
        return;

    if(readOffset >= sizeof(readBuffer))
    {
        // Should/could only happen if packet received claims it has an abnormally large data size. This could happen if:
        //   * Intentional buffer overflow attack / client uncautiously sending too large data
        //   * The buffer on the server side is smaller than the packet that is currently receiving
        PrintDebug("Buffer overflow: offset="); PrintDebug(readOffset); PrintDebug(", max="); PrintDebug(sizeof(readBuffer));
        readOffset = 0U; // Ignore this rubbish
    }

    size_t readSize = CommandSerial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
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
        packetFailCount++;
        SetStatus(false);
        return;
    }
    else
    {
        packetSuccessCount++;
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

    CommandSerial.begin(9600, SERIAL_8N1);

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
