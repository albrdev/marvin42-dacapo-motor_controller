#include <stdint.h>
#include <stdarg.h>
#include "DCMotorAssembly.hpp"
#include "Button.hpp"
#include "crc.h"
#include "packet.h"
#include "custom_packets.h"
#include "generic.hpp"

DCMotorAssembly motors;
Button debugButton(13);

#define D4 4
#define D7 7

#define CommandSerial Serial2

void SetStatus(const bool status)
{
    //digitalWrite(D4, status ? HIGH : LOW);
    //digitalWrite(D7, !status ? HIGH : LOW);
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

bool debug = false;
#define PrintDebugBla(msg) if(debug) { PrintDebug(msg); }
#define PrintDebugBlaLine(msg) if(debug) { PrintDebugLine(msg); }

void toggleDebug(const bool state)
{
    if(!state)
        return;

    if(debug)
        PrintDebugBlaLine("DEBUG: OFF");

    debug = !debug;

    if(debug)
        PrintDebugBlaLine("DEBUG: ON");

    delay(350);
}

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

            PrintDebugBla("CPT_MOTORBALANCE: left="); PrintDebugBla(left); PrintDebugBla(", right="); PrintDebugBla(right);
            PrintDebugBlaLine("");

            motors.SetLeftSpeed(left);
            motors.SetRightSpeed(right);

            break;
        }
        case CPT_DIRECTION:
        {
            const packet_direction_t* pkt = (const packet_direction_t*)hdr;
            memcpy(&inputdata.direction, &pkt->direction, sizeof(pkt->direction));

            PrintDebugBla("CPT_DIRECTION: direction="); PrintDebugBla("(x="); PrintDebugBla(inputdata.direction.x); PrintDebugBla(", y="); PrintDebugBla(inputdata.direction.y); PrintDebugBla(")");
            PrintDebugBlaLine(""); 

            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORPOWER:
        {
            const packet_motorpower_t* pkt = (const packet_motorpower_t*)hdr;
            memcpy(&inputdata.power, &pkt->power, sizeof(pkt->power));

            PrintDebugBla("CPT_MOTORPOWER: power="); PrintDebugBlaLine(inputdata.power);
            PrintDebugBlaLine("");

            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)hdr;
            memcpy(&inputdata.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.power, &pkt->power, sizeof(pkt->power));

            PrintDebugBla("CPT_MOTORRUN: direction="); PrintDebugBla("(x="); PrintDebugBla(inputdata.direction.x); PrintDebugBla(", y="); PrintDebugBla(inputdata.direction.y); PrintDebugBla(")");
            PrintDebugBla(", power="); PrintDebugBla(inputdata.power);
            PrintDebugBlaLine("");

            motors.Run(inputdata.direction.x, inputdata.direction.y, inputdata.power);

            break;
        }
        case CPT_MOTORSTOP:
        {
            inputdata = { { 0.0f, 0.0f }, 0.0f };
            PrintDebugBla("CPT_MOTORSTOP");
            PrintDebugBlaLine("");

            motors.Halt();

            break;
        }
        default:
        {
            PrintDebugBla("Unknown packet type: "); PrintDebugBla(hdr->type);
            PrintDebugBlaLine("");

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
    PrintDebugBla("Header: chksum_header="); PrintDebugBla(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebugBla(", chksum_data="); PrintDebugBla(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    PrintDebugBla(", type="); PrintDebugBla(hdr->type); PrintDebugBla(", size="); PrintDebugBla(hdr->size);
    PrintDebugBla(" (chksum="); PrintDebugBla(hexstr(&chksum, sizeof(chksum))); PrintDebugBla(", hex="); PrintDebugBla(hexstr(offset, sizeof(*hdr))); PrintDebugBla(")");
    PrintDebugBlaLine("");

    if(packet_verifyheader(hdr) == 0)
    {
        PrintDebugBla("Header checksum failed: ");
        PrintDebugBla(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebugBla(", "); PrintDebugBla(hexstr(&chksum, sizeof(chksum)));
        PrintDebugBlaLine("");
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
    PrintDebugBla("Content: chksum_data="); PrintDebugBla(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); PrintDebugBla(", size="); PrintDebugBla(hdr->size);
    PrintDebugBla(" (chksum="); PrintDebugBla(hexstr(&chksum, sizeof(chksum))); PrintDebugBla(", hex="); PrintDebugBla(hexstr((const uint8_t* const)offset + sizeof(*hdr), hdr->size)); PrintDebugBla(")");
    PrintDebugBlaLine("");

    if(packet_verifydata(hdr) == 0)
    {
        PrintDebugBla("Content checksum failed: ");
        PrintDebugBla(hdr->chksum_data); PrintDebugBla(" / "); PrintDebugBla(chksum);
        PrintDebugBlaLine("");
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
        PrintDebugBla("Buffer overflow: offset="); PrintDebugBla(readOffset); PrintDebugBla(", max="); PrintDebugBla(sizeof(readBuffer));
        PrintDebugBlaLine("");
        readOffset = 0U; // Ignore this rubbish
    }

    size_t readSize = CommandSerial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
    readSize += readOffset;
    readOffset = 0U;

    PrintDebugBla("Raw: size="); PrintDebugBla(readSize); PrintDebugBla(", hex="); PrintDebugBla(hexstr(readBuffer, readSize));
    PrintDebugBlaLine("");

    PrintDebugBla("BUFFER BEGIN");
    PrintDebugBlaLine("");
    size_t indexOffset = 0U;
    size_t incrementSize = 0U;
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    while(indexOffset + sizeof(packet_header_t) <= readSize)
    {
        if(!handle_packet(readBuffer + indexOffset, readBufferEnd, &incrementSize))
        {
            break;
        }

        PrintDebugBlaLine("");
        indexOffset += incrementSize;
    }

    if(incrementSize == INVALID_SIZE)
    {
        PrintDebugBla("BUFFER ERROR");
        PrintDebugBlaLine(""); PrintDebugBlaLine("");
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

    PrintDebugBla("BUFFER END");
    PrintDebugBlaLine(""); PrintDebugBlaLine("");
}

void setup(void)
{
    delay(2500);
    Serial.begin(9600, SERIAL_8N1);
    Serial.print("Initializing...");

    debugButton.SetOnStateChangedEvent(toggleDebug);

    CommandSerial.begin(115200, SERIAL_8N1);

    motors.Begin();

    //pinMode(D4, OUTPUT);
    //pinMode(D7, OUTPUT);
    SetStatus(true);

    Serial.print("Done");
    Serial.println("");
}

void loop(void)
{
    debugButton.Poll();

    handle_data();
}
