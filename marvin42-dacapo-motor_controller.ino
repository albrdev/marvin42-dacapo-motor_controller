#include <stdint.h>
#include <stdarg.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DCMotorAssembly.hpp"
#include "HC_SR04.hpp"
#include "Button.hpp"
#include "crc.h"
#include "packet.h"
#include "custom_packets.h"
#include "generic.hpp"

DCMotorAssembly motors;

#define BRAKE_THRESHOLD 15.0f   // cm
HC_SR04 sonicSensor(22, 23);
float distance = 0.0f;

Button debugButton(13);

#define D4 4
#define D7 7

#define CommandSerial   Serial1
#define LOOP_DELAY      10UL

void SetStatus(const bool status)
{
    //digitalWrite(D4, status ? HIGH : LOW);
    //digitalWrite(D7, !status ? HIGH : LOW);
}

struct
{
    struct
    {
        vector2data_t direction;
        float power;
    } movement;

    struct
    {
        int8_t direction;
        float power;
    } rotation;

    Quaternion joystickRotation;
} inputdata = { { { 0.0f, 0.0f }, 1.0f}, { 0, 0.0f }, Quaternion() };

uint8_t readBuffer[64];
size_t readOffset = 0U;
#define INVALID_SIZE SIZE_MAX

size_t packetSuccessCount = 0U;
size_t packetFailCount = 0U;

#define KA_INTERVAL 1000UL
unsigned long int nextAutoHalt = 0UL;

bool debug = false;
#define PrintDebug2(msg)      do { if(debug) { PrintDebug(msg); } } while(false)
#define PrintDebugLine2(msg)  do { if(debug) { PrintDebugLine(msg); } } while(false)

void toggleDebug(const bool state)
{
    if(!state)
        return;

    debug = !debug;
    Serial.println(debug ? "DEBUG: ON" : "DEBUG: OFF");
    delay(350);
}

void calcDirection(const vector2data_t& direction, Quaternion& remoteRotation, Quaternion& localRotation)
{
    Quaternion q = remoteRotation.getProduct(localRotation.getConjugate()); // q1 * inverse(q2) == q1 / q2
    VectorFloat tmp(direction.x, direction.y, 0.0f); // Rotate by direction
    tmp.rotate(&q);
    tmp.normalize();

    vector2data_t result = { tmp.x, tmp.y };
    //VectorFloat localDirection = q.getProduct(direction).getNormalized();
}

bool obstacleNear(void)
{
    return distance >= 0.0f && distance <= BRAKE_THRESHOLD;
}

bool movingForwards(void)
{
    return inputdata.movement.direction.y > 0.0f && inputdata.movement.power > 0.0f;
}

void Halt(void)
{
    inputdata.movement.direction = { 0.0f, 0.0f };
    inputdata.rotation.direction = 0;
    motors.Halt();
}

void Run(void)
{
    if(obstacleNear() && movingForwards())
    {
        PrintDebug2("Ignored: Obstruction");
        PrintDebugLine2();

        Halt();
        return;
    }

    motors.Run(inputdata.movement.direction.x, inputdata.movement.direction.y, inputdata.movement.power);
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

            PrintDebug2("CPT_MOTORBALANCE: ");
            PrintDebug2("left="); PrintDebug2(left); PrintDebug2(", right="); PrintDebug2(right);
            PrintDebugLine2();

            motors.SetLeftSpeed(left);
            motors.SetRightSpeed(right);
            break;
        }
        case CPT_DIRECTION:
        {
            const packet_direction_t* pkt = (const packet_direction_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));

            PrintDebug2("CPT_DIRECTION: ");
            PrintDebug2("direction="); PrintDebug2("(x="); PrintDebug2(inputdata.movement.direction.x); PrintDebug2(", y="); PrintDebug2(inputdata.movement.direction.y); PrintDebug2(")");
            PrintDebugLine2();

            Run();
            break;
        }
        case CPT_DIRQUAT:
        {
            const packet_dirquat_t* pkt = (const packet_dirquat_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));
            inputdata.joystickRotation = Quaternion(pkt->rotation.w, pkt->rotation.x, pkt->rotation.y, pkt->rotation.z);

            PrintDebug2("CPT_DIRQUAT: ");
            PrintDebug2("direction="); PrintDebug2("(x="); PrintDebug2(inputdata.movement.direction.x); PrintDebug2(", y="); PrintDebug2(inputdata.movement.direction.y); PrintDebug2("), ");
            PrintDebug2("rotation="); PrintDebug2("(w="); PrintDebug2(inputdata.joystickRotation.w); PrintDebug2(", x="); PrintDebug2(inputdata.joystickRotation.x); PrintDebug2(", y="); PrintDebug2(inputdata.joystickRotation.y); PrintDebug2(", z="); PrintDebug2(inputdata.joystickRotation.z); PrintDebug2(")");
            PrintDebugLine2();

            Run();
            break;
        }
        case CPT_MOTORPOWER:
        {
            const packet_motorpower_t* pkt = (const packet_motorpower_t*)hdr;
            memcpy(&inputdata.movement.power, &pkt->power, sizeof(pkt->power));

            PrintDebug2("CPT_MOTORPOWER: ");
            PrintDebug2("power="); PrintDebug2(inputdata.movement.power);
            PrintDebugLine2();

            Run();
            break;
        }
        case CPT_MOTORROTATION:
        {
            const packet_motorrotation_t* pkt = (const packet_motorrotation_t*)hdr;
            memcpy(&inputdata.rotation.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.rotation.power, &pkt->power, sizeof(pkt->power));

            PrintDebug2("CPT_MOTORROTATION: ");
            PrintDebug2("direction="); PrintDebug2(inputdata.rotation.direction); PrintDebug2(", power="); PrintDebug2(inputdata.rotation.power);
            PrintDebugLine2();

            motors.Rotate(-inputdata.rotation.direction, inputdata.rotation.power);
            break;
        }
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.movement.power, &pkt->power, sizeof(pkt->power));

            PrintDebug2("CPT_MOTORRUN: ");
            PrintDebug2("direction="); PrintDebug2("(x="); PrintDebug2(inputdata.movement.direction.x); PrintDebug2(", y="); PrintDebug2(inputdata.movement.direction.y); PrintDebug2(")");
            PrintDebug2(", power="); PrintDebug2(inputdata.movement.power);
            PrintDebugLine2();

            Run();
            break;
        }
        case CPT_MOTORSTOP:
        {
            PrintDebug2("CPT_MOTORSTOP");
            PrintDebugLine2();

            Halt();
            break;
        }
        case PT_SYN:
        {
            PrintDebug2("PT_SYN");
            PrintDebugLine2();

            break;
        }
        default:
        {
            PrintDebug2("Unknown packet type: "); PrintDebug2(hdr->type);
            PrintDebugLine2();

            break;
        }
    }

    nextAutoHalt = millis() + KA_INTERVAL;
}

bool handle_packet(const uint8_t* const offset, const uint8_t* const end, size_t* const incrementSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    *incrementSize = INVALID_SIZE;

    #ifdef M42_DEBUG
    uint16_t chksum = mkcrc16((const uint8_t* const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    #endif
    PrintDebug2("Header: chksum_header="); PrintDebug2(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug2(", chksum_data="); PrintDebug2(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    PrintDebug2(", type="); PrintDebug2(hdr->type); PrintDebug2(", size="); PrintDebug2(hdr->size);
    PrintDebug2(" (chksum="); PrintDebug2(hexstr(&chksum, sizeof(chksum))); PrintDebug2(", hex="); PrintDebug2(hexstr(offset, sizeof(*hdr))); PrintDebug2(")");
    PrintDebugLine2();

    if(packet_verifyheader(hdr) == 0)
    {
        PrintDebug2("Header checksum failed: ");
        PrintDebug2(hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); PrintDebug2(", "); PrintDebug2(hexstr(&chksum, sizeof(chksum)));
        PrintDebugLine2();
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
    PrintDebug2("Content: chksum_data="); PrintDebug2(hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); PrintDebug2(", size="); PrintDebug2(hdr->size);
    PrintDebug2(" (chksum="); PrintDebug2(hexstr(&chksum, sizeof(chksum))); PrintDebug2(", hex="); PrintDebug2(hexstr((const uint8_t* const)offset + sizeof(*hdr), hdr->size)); PrintDebug2(")");
    PrintDebugLine2();

    if(packet_verifydata(hdr) == 0)
    {
        PrintDebug2("Content checksum failed: ");
        PrintDebug2(hdr->chksum_data); PrintDebug2(" / "); PrintDebug2(chksum);
        PrintDebugLine2();
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
        PrintDebug2("Buffer overflow: offset="); PrintDebug2(readOffset); PrintDebug2(", max="); PrintDebug2(sizeof(readBuffer));
        PrintDebugLine2();
        readOffset = 0U; // Ignore this rubbish
    }

    digitalWrite(15, HIGH);
    size_t readSize = CommandSerial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
    digitalWrite(15, LOW);
    readSize += readOffset;
    readOffset = 0U;

    PrintDebug2("Raw: size="); PrintDebug2(readSize); PrintDebug2(", hex="); PrintDebug2(hexstr(readBuffer, readSize));
    PrintDebugLine2();

    PrintDebug2("BUFFER BEGIN");
    PrintDebugLine2();
    size_t indexOffset = 0U;
    size_t incrementSize = 0U;
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    while(indexOffset + sizeof(packet_header_t) <= readSize)
    {
        if(!handle_packet(readBuffer + indexOffset, readBufferEnd, &incrementSize))
        {
            break;
        }

        PrintDebugLine2();
        indexOffset += incrementSize;
    }

    if(incrementSize == INVALID_SIZE)
    {
        PrintDebug2("BUFFER ERROR");
        PrintDebugLine2(); PrintDebugLine2();
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

    PrintDebug2("BUFFER END");
    PrintDebugLine2(); PrintDebugLine2();
}

void setup(void)
{
    delay(2500);
    Serial.begin(9600, SERIAL_8N1);
    Serial.println("Initializing...");

    debugButton.SetOnStateChangedEvent(toggleDebug);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);

    CommandSerial.begin(115200, SERIAL_8N1);
    CommandSerial.setTimeout(50UL);

    motors.Begin();

    //pinMode(D4, OUTPUT);
    //pinMode(D7, OUTPUT);
    SetStatus(true);

    Serial.println("Done");
    Serial.println();
    Serial.flush();
}

void loop(void)
{
    debugButton.Poll();

    distance = sonicSensor.GetDistance();
    if(obstacleNear() && movingForwards())
    {
        PrintDebug2("Halt: Obstruction");
        PrintDebugLine2();
        Halt();
    }

    if((long)(millis() - nextAutoHalt) >= 0L)
    {
        Halt();
        nextAutoHalt = millis() + KA_INTERVAL;

        PrintDebug2("Halt: Timeout");
        PrintDebugLine2();
    }

    handle_data();
    delay(LOOP_DELAY);
}
