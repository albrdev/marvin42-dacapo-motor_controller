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
#include "debug.hpp"

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
    } spin;

    Quaternion rotation;
    Quaternion joystickRotation;
} inputdata = { { { 0.0f, 0.0f }, 1.0f}, { 0, 0.0f }, Quaternion(), Quaternion() };

uint8_t readBuffer[64];
size_t readOffset = 0U;
#define INVALID_SIZE SIZE_MAX

size_t packetSuccessCount = 0U;
size_t packetFailCount = 0U;

#define KA_INTERVAL 1000UL
unsigned long int nextAutoHalt = 0UL;

bool debug = false;
#define DebugPrint2(msg)        do { if(debug) { DebugPrint(msg); } } while(false)
#define DebugPrintLine2(msg)    do { if(debug) { DebugPrintLine(msg); } } while(false)

void toggleDebug(const bool state)
{
    if(!state)
        return;

    debug = !debug;
    DebugPrintLine(debug ? "DEBUG: ON" : "DEBUG: OFF");
    delay(350);
}

vector2data_t calcDirection(const vector2data_t& direction, Quaternion& remoteRotation, Quaternion& localRotation)
{
    VectorFloat tmpDir(direction.x, direction.y, 0.0f);
    Quaternion q = remoteRotation.getProduct(localRotation.getConjugate()); // q1 * inverse(q2) == q1 / q2

    VectorFloat globalDir = tmpDir.getRotated(&remoteRotation).getNormalized();
    VectorFloat localDir = tmpDir.getRotated(&q).getNormalized();

    return { localDir.x, localDir.y };
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
    inputdata.spin.direction = 0;
    motors.Halt();
}

void Run(void)
{
    if(obstacleNear() && movingForwards())
    {
        DebugPrintLineN(DM_GENERIC, "Ignored: Obstruction");

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

            DebugPrintN(DM_NWDATA, "CPT_MOTORBALANCE: ");
            DebugPrintN(DM_NWDATA, "left="); DebugPrintN(DM_NWDATA, left); DebugPrintN(DM_NWDATA, ", right="); DebugPrintN(DM_NWDATA, right);
            DebugPrintLineN(DM_NWDATA);

            motors.SetLeftSpeed(left);
            motors.SetRightSpeed(right);
            break;
        }
        case CPT_DIRECTION:
        {
            const packet_direction_t* pkt = (const packet_direction_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));

            DebugPrintN(DM_NWDATA, "CPT_DIRECTION: ");
            DebugPrintN(DM_NWDATA, "direction="); DebugPrintN(DM_NWDATA, "(x="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.y); DebugPrintN(DM_NWDATA, ")");
            DebugPrintLineN(DM_NWDATA);

            Run();
            break;
        }
        case CPT_DIRQUAT:
        {
            const packet_dirquat_t* pkt = (const packet_dirquat_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));
            inputdata.joystickRotation = Quaternion(pkt->rotation.w, pkt->rotation.x, pkt->rotation.y, pkt->rotation.z);

            DebugPrintN(DM_NWDATA, "CPT_DIRQUAT: ");
            DebugPrintN(DM_NWDATA, "direction="); DebugPrintN(DM_NWDATA, "(x="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.y); DebugPrintN(DM_NWDATA, "), ");
            DebugPrintN(DM_NWDATA, "rotation="); DebugPrintN(DM_NWDATA, "(w="); DebugPrintN(DM_NWDATA, inputdata.joystickRotation.w); DebugPrintN(DM_NWDATA, ", x="); DebugPrintN(DM_NWDATA, inputdata.joystickRotation.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.joystickRotation.y); DebugPrintN(DM_NWDATA, ", z="); DebugPrintN(DM_NWDATA, inputdata.joystickRotation.z); DebugPrintN(DM_NWDATA, ")");
            DebugPrintLineN(DM_NWDATA);

            Run();
            break;
        }
        case CPT_MOTORPOWER:
        {
            const packet_motorpower_t* pkt = (const packet_motorpower_t*)hdr;
            memcpy(&inputdata.movement.power, &pkt->power, sizeof(pkt->power));

            DebugPrintN(DM_NWDATA, "CPT_MOTORPOWER: ");
            DebugPrintN(DM_NWDATA, "power="); DebugPrintN(DM_NWDATA, inputdata.movement.power);
            DebugPrintLineN(DM_NWDATA);

            Run();
            break;
        }
        case CPT_MOTORSPIN:
        {
            const packet_motorspin_t* pkt = (const packet_motorspin_t*)hdr;
            memcpy(&inputdata.spin.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.spin.power, &pkt->power, sizeof(pkt->power));

            DebugPrintN(DM_NWDATA, "CPT_MOTORSPIN: ");
            DebugPrintN(DM_NWDATA, "direction="); DebugPrintN(DM_NWDATA, inputdata.spin.direction); DebugPrintN(DM_NWDATA, ", power="); DebugPrintN(DM_NWDATA, inputdata.spin.power);
            DebugPrintLineN(DM_NWDATA);

            motors.Rotate(-inputdata.spin.direction, inputdata.spin.power);
            break;
        }
        case CPT_MOTORRUN:
        {
            const packet_motorrun_t* pkt = (const packet_motorrun_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));
            memcpy(&inputdata.movement.power, &pkt->power, sizeof(pkt->power));

            DebugPrintN(DM_NWDATA, "CPT_MOTORRUN: ");
            DebugPrintN(DM_NWDATA, "direction="); DebugPrintN(DM_NWDATA, "(x="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.y); DebugPrintN(DM_NWDATA, ")");
            DebugPrintN(DM_NWDATA, ", power="); DebugPrintN(DM_NWDATA, inputdata.movement.power);
            DebugPrintLineN(DM_NWDATA);

            Run();
            break;
        }
        case CPT_MOTORSTOP:
        {
            DebugPrintN(DM_NWDATA, "CPT_MOTORSTOP");
            DebugPrintLineN(DM_NWDATA);

            Halt();
            break;
        }
        case PT_SYN:
        {
            DebugPrintN(DM_NWDATA, "PT_SYN");
            DebugPrintLineN(DM_NWDATA);

            break;
        }
        default:
        {
            DebugPrintN(DM_NWDATA, "Unknown packet type: "); DebugPrintN(DM_NWDATA, hdr->type);
            DebugPrintLineN(DM_NWDATA);

            break;
        }
    }

    nextAutoHalt = millis() + KA_INTERVAL;
}

bool parsePacket(const uint8_t* const offset, const uint8_t* const end, size_t* const incrementSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    *incrementSize = INVALID_SIZE;

    uint16_t chksum = mkcrc16((const uint8_t* const)hdr + sizeof(hdr->chksum_header), sizeof(*hdr) - sizeof(hdr->chksum_header));
    DebugPrintN(DM_NWPKT, "Header: chksum_header="); DebugPrintN(DM_NWPKT, hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); DebugPrintN(DM_NWPKT, ", chksum_data="); DebugPrintN(DM_NWPKT, hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data)));
    DebugPrintN(DM_NWPKT, ", type="); DebugPrintN(DM_NWPKT, hdr->type); DebugPrintN(DM_NWPKT, ", size="); DebugPrintN(DM_NWPKT, hdr->size);
    DebugPrintN(DM_NWPKT, " (chksum="); DebugPrintN(DM_NWPKT, hexstr(&chksum, sizeof(chksum))); DebugPrintN(DM_NWPKT, ", hex="); DebugPrintN(DM_NWPKT, hexstr(offset, sizeof(*hdr))); DebugPrintN(DM_NWPKT, ")");
    DebugPrintLineN(DM_NWPKT);

    if(packet_verifyheader(hdr) == 0)
    {
        DebugPrintN(DM_NWPKT, "Header checksum failed: ");
        DebugPrintN(DM_NWPKT, hexstr(&hdr->chksum_header, sizeof(hdr->chksum_header))); DebugPrintN(DM_NWPKT, ", "); DebugPrintN(DM_NWPKT, hexstr(&chksum, sizeof(chksum)));
        DebugPrintLineN(DM_NWPKT);
        return false;
    }

    if(offset + (sizeof(*hdr) + hdr->size) > end)
    {
        *incrementSize = 0U;
        return false;
    }

    chksum = mkcrc16((const uint8_t* const)hdr + sizeof(*hdr), hdr->size);
    DebugPrintN(DM_NWPKT, "Content: chksum_data="); DebugPrintN(DM_NWPKT, hexstr(&hdr->chksum_data, sizeof(hdr->chksum_data))); DebugPrintN(DM_NWPKT, ", size="); DebugPrintN(DM_NWPKT, hdr->size);
    DebugPrintN(DM_NWPKT, " (chksum="); DebugPrintN(DM_NWPKT, hexstr(&chksum, sizeof(chksum))); DebugPrintN(DM_NWPKT, ", hex="); DebugPrintN(DM_NWPKT, hexstr((const uint8_t* const)offset + sizeof(*hdr), hdr->size)); DebugPrintN(DM_NWPKT, ")");
    DebugPrintLineN(DM_NWPKT);

    if(packet_verifydata(hdr) == 0)
    {
        DebugPrintN(DM_NWPKT, "Content checksum failed: ");
        DebugPrintN(DM_NWPKT, hdr->chksum_data); DebugPrintN(DM_NWPKT, " / "); DebugPrintN(DM_NWPKT, chksum);
        DebugPrintLineN(DM_NWPKT);
        return false;
    }

    *incrementSize = sizeof(*hdr) + hdr->size;
    OnPacketReceived(hdr);
    return true;
}

void recvData(void)
{
    if(CommandSerial.available() <= 0)
        return;

    if(readOffset >= sizeof(readBuffer))
    {
        // Should/could only happen if packet received claims it has an abnormally large data size. This could happen if:
        //   * Intentional buffer overflow attack / client uncautiously sending too large data
        //   * The buffer on the server side is smaller than the packet that is currently receiving
        DebugPrintN(DM_NWRECV, "Buffer overflow: offset="); DebugPrintN(DM_NWRECV, readOffset); DebugPrintN(DM_NWRECV, ", max="); DebugPrintN(DM_NWRECV, sizeof(readBuffer));
        DebugPrintLineN(DM_NWRECV);
        readOffset = 0U; // Ignore this rubbish
    }

    digitalWrite(15, HIGH);
    size_t readSize = CommandSerial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
    digitalWrite(15, LOW);
    readSize += readOffset;
    readOffset = 0U;

    DebugPrintN(DM_NWRECV, "Raw: size="); DebugPrintN(DM_NWRECV, readSize); DebugPrintN(DM_NWRECV, ", hex="); DebugPrintN(DM_NWRECV, hexstr(readBuffer, readSize));
    DebugPrintLineN(DM_NWRECV);

    DebugPrintLineN(DM_NWRECV, "[PACKET BEGIN]");
    size_t indexOffset = 0U;
    size_t incrementSize = 0U;
    const uint8_t* const readBufferEnd = &readBuffer[readSize];
    while(indexOffset + sizeof(packet_header_t) <= readSize)
    {
        if(!parsePacket(readBuffer + indexOffset, readBufferEnd, &incrementSize))
        {
            break;
        }

        indexOffset += incrementSize;
    }

    if(incrementSize == INVALID_SIZE)
    {
        DebugPrintLineN(DM_NWRECV, "[PACKET ERROR]");
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

    DebugPrintLineN(DM_NWRECV, "[PACKET END]");
}

#define MPU6060_INT_PIN 2
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setupMPU6050(void)
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    DebugPrintLineN(DM_SETUP, "MPU6050...");
    mpu.initialize();
    pinMode(MPU6060_INT_PIN, INPUT);

    if(!mpu.testConnection())
    {
        DebugPrintLineN(DM_SETUP, "Failed");
        while(true);
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    if(devStatus != 0)
    {
        DebugPrintN(DM_SETUP, "Fail: DMP, "); DebugPrintLineN(DM_SETUP, devStatus);
        while(true);
    }

    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(MPU6060_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void readDMP(void)
{
    if(!dmpReady || mpu.dmpGetCurrentFIFOPacket(fifoBuffer) == 0)
    {
        return;
    }

    mpu.dmpGetQuaternion(&inputdata.rotation, fifoBuffer);
}

void setup(void)
{
    delay(2500);
    Serial.begin(9600, SERIAL_8N1);
    DebugPrintLineN(DM_SETUP, "Initializing...");

    debugButton.SetOnStateChangedEvent(toggleDebug);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);

    CommandSerial.begin(115200, SERIAL_8N1);
    CommandSerial.setTimeout(50UL);

    motors.Begin();

    //pinMode(D4, OUTPUT);
    //pinMode(D7, OUTPUT);
    SetStatus(true);

    DebugPrintLineN(DM_SETUP, "Done");
    DebugPrintLineN(DM_SETUP);
    Serial.flush();
}

void loop(void)
{
    debugButton.Poll();

    distance = sonicSensor.GetDistance();
    if(obstacleNear() && movingForwards())
    {
        DebugPrintLineN(DM_GENERIC, "Halt: Obstruction");
        Halt();
    }

    if((long)(millis() - nextAutoHalt) >= 0L)
    {
        Halt();
        nextAutoHalt = millis() + KA_INTERVAL;

        DebugPrintLineN(DM_GENERIC, "Halt: Timeout");
    }

    readDMP();

    recvData();
    delay(LOOP_DELAY);
}
