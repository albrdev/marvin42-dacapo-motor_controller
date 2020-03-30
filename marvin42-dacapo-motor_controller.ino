#include <stdint.h>
#include <stdarg.h>
#include <Wire.h>
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

#define DataSerial              Serial1

#define PROXIMITYHALT_THRESHOLD 5.0f    // cm
#define KEEPALIVE_INTERVAL      1000UL  // ms

#define PIN_FAILLED             44
#define PIN_SUCCESSLED          46
#define PIN_RECVLED             48

#define LOOP_DELAY              10UL

static const Quaternion QUATERNION_INVALID(0.0f, 0.0f, 0.0f, 0.0f);

DCMotorAssembly motors;

HC_SR04 sonicSensor(22, 23);
float distance = -1.0f;

Button debugButton(13);

unsigned long int nextAutoHalt = 0UL;

void setStatusLED(const bool status)
{
    digitalWrite(PIN_SUCCESSLED, status ? HIGH : LOW);
    digitalWrite(PIN_FAILLED, !status ? HIGH : LOW);
}

void setupFail(void)
{
    setStatusLED(false);
    while(true);
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
    Quaternion remoteRotation;
} inputdata = { { { 0.0f, 0.0f }, 1.0f}, { 0, 0.0f }, QUATERNION_INVALID, QUATERNION_INVALID };

uint8_t readBuffer[128];
size_t readOffset = 0U;
#define INVALID_SIZE SIZE_MAX

size_t packetSuccessCount = 0U;
size_t packetFailCount = 0U;

bool quaternionEquals(const Quaternion& a, const Quaternion& b)
{
    return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z;
}

vector2data_t calcDirection(const vector2data_t& inputDirection, Quaternion& remoteRotation, Quaternion& localRotation)
{
    VectorFloat tmpDir(inputDirection.x, inputDirection.y, 0.0f);
    Quaternion q = remoteRotation.getProduct(localRotation.getConjugate()); // q1 * inverse(q2) == q1 / q2

    VectorFloat globalDir = tmpDir.getRotated(&remoteRotation).getNormalized();
    VectorFloat localDir = tmpDir.getRotated(&q).getNormalized();

    DebugPrintLineN(DM_GENERIC, "[DIRECTION]");
    DebugPrintN(DM_GENERIC, "input=(x="); DebugPrintN(DM_GENERIC, tmpDir.x); DebugPrintN(DM_GENERIC, ", y="); DebugPrintN(DM_GENERIC, tmpDir.y); DebugPrintN(DM_GENERIC, "), ");
    DebugPrintN(DM_GENERIC, "global=(x="); DebugPrintN(DM_GENERIC, globalDir.x); DebugPrintN(DM_GENERIC, ", y="); DebugPrintN(DM_GENERIC, globalDir.y); DebugPrintN(DM_GENERIC, "), ");
    DebugPrintN(DM_GENERIC, "local=(x="); DebugPrintN(DM_GENERIC, localDir.x); DebugPrintN(DM_GENERIC, ", y="); DebugPrintN(DM_GENERIC, localDir.y); DebugPrintN(DM_GENERIC, "), ");
    DebugPrintLineN(DM_GENERIC);

    return { localDir.x, localDir.y };
}

bool obstacleNear(void)
{
    return distance >= 0.0f && distance <= PROXIMITYHALT_THRESHOLD;
}

bool movingForwards(void)
{
    return inputdata.movement.direction.y > 0.0f && inputdata.movement.power > 0.0f;
}

void halt(void)
{
    inputdata.movement.direction = { 0.0f, 0.0f };
    inputdata.spin.direction = 0;
    motors.Halt();
}

void run(void)
{
    if(obstacleNear() && movingForwards())
    {
        DebugPrintLineN(DM_GENERIC, "Ignored: Obstruction");

        halt();
        return;
    }

    motors.Run(inputdata.movement.direction.x, inputdata.movement.direction.y, inputdata.movement.power);
}

void onPacketReceived(const packet_header_t* const hdr)
{
    DebugPrintLineN(DM_NWDATA, "[DM_NWDATA]");
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

            inputdata.rotation = QUATERNION_INVALID;
            run();
            break;
        }
        case CPT_DIRQUAT:
        {
            const packet_dirquat_t* pkt = (const packet_dirquat_t*)hdr;
            memcpy(&inputdata.movement.direction, &pkt->direction, sizeof(pkt->direction));
            inputdata.remoteRotation = Quaternion(pkt->rotation.w, pkt->rotation.x, pkt->rotation.y, pkt->rotation.z);

            DebugPrintN(DM_NWDATA, "CPT_DIRQUAT: ");
            DebugPrintN(DM_NWDATA, "direction="); DebugPrintN(DM_NWDATA, "(x="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.movement.direction.y); DebugPrintN(DM_NWDATA, "), ");
            DebugPrintN(DM_NWDATA, "rotation="); DebugPrintN(DM_NWDATA, "(w="); DebugPrintN(DM_NWDATA, inputdata.remoteRotation.w); DebugPrintN(DM_NWDATA, ", x="); DebugPrintN(DM_NWDATA, inputdata.remoteRotation.x); DebugPrintN(DM_NWDATA, ", y="); DebugPrintN(DM_NWDATA, inputdata.remoteRotation.y); DebugPrintN(DM_NWDATA, ", z="); DebugPrintN(DM_NWDATA, inputdata.remoteRotation.z); DebugPrintN(DM_NWDATA, ")");
            DebugPrintLineN(DM_NWDATA);

            if((inputdata.movement.direction.x != 0.0f || inputdata.movement.direction.y != 0.0f) && !quaternionEquals(inputdata.remoteRotation, QUATERNION_INVALID))
            {
                //readDMP();
                if(!quaternionEquals(inputdata.rotation, QUATERNION_INVALID))
                {
                    break;
                }

                calcDirection(inputdata.movement.direction, inputdata.remoteRotation, inputdata.rotation);
            }

            run();
            break;
        }
        case CPT_MOTORPOWER:
        {
            const packet_motorpower_t* pkt = (const packet_motorpower_t*)hdr;
            memcpy(&inputdata.movement.power, &pkt->power, sizeof(pkt->power));

            DebugPrintN(DM_NWDATA, "CPT_MOTORPOWER: ");
            DebugPrintN(DM_NWDATA, "power="); DebugPrintN(DM_NWDATA, inputdata.movement.power);
            DebugPrintLineN(DM_NWDATA);

            run();
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

            run();
            break;
        }
        case CPT_MOTORSTOP:
        {
            DebugPrintN(DM_NWDATA, "CPT_MOTORSTOP");
            DebugPrintLineN(DM_NWDATA);

            halt();
            break;
        }
        case PT_SYN:
        {
            DebugPrintN(DM_NWEXTRA, "PT_SYN");
            DebugPrintLineN(DM_NWEXTRA);

            break;
        }
        default:
        {
            DebugPrintN(DM_NWDATA, "Unknown packet type: "); DebugPrintN(DM_NWDATA, hdr->type);
            DebugPrintLineN(DM_NWDATA);

            break;
        }
    }

    nextAutoHalt = millis() + KEEPALIVE_INTERVAL;
}

bool parsePacket(const uint8_t* const offset, const uint8_t* const end, size_t* const incrementSize)
{
    const packet_header_t* hdr = (const packet_header_t*)offset;
    *incrementSize = INVALID_SIZE;

    DebugPrintLineN(DM_NWPKT, "[DM_NWPKT]");
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
    onPacketReceived(hdr);
    return true;
}

void recvData(void)
{
    if(DataSerial.available() <= 0)
        return;

    if(readOffset >= sizeof(readBuffer))
    {
        // Should/could only happen if packet received claims it has an abnormally large data size. This could happen if:
        //   * Intentional buffer overflow attack / client uncautiously sending too large data
        //   * The buffer on the server side is smaller than the packet that is currently receiving
        DebugPrintN(DM_NWRECV, "Buffer overflow: offset="); DebugPrintN(DM_NWRECV, readOffset); DebugPrintN(DM_NWRECV, ", max="); DebugPrintN(DM_NWRECV, sizeof(readBuffer));
        DebugPrintLineN(DM_NWRECV);
        readOffset = 0U; // Ignore data
    }

    digitalWrite(PIN_RECVLED, HIGH);
    size_t readSize = DataSerial.readBytes(readBuffer + readOffset, sizeof(readBuffer) - readOffset);
    digitalWrite(PIN_RECVLED, LOW);
    readSize += readOffset;
    readOffset = 0U;

    DebugPrintLineN(DM_NWRECV, "[DM_NWRECV]");
    DebugPrintN(DM_NWRECV, "Raw: size="); DebugPrintN(DM_NWRECV, readSize); DebugPrintN(DM_NWRECV, ", hex="); DebugPrintN(DM_NWRECV, hexstr(readBuffer, readSize));
    DebugPrintLineN(DM_NWRECV);

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
        packetFailCount++;
        setStatusLED(false);
        return;
    }
    else
    {
        packetSuccessCount++;
        setStatusLED(true);
    }


    readOffset = readSize - indexOffset;
    for(size_t i = 0U; i < readOffset; i++)
    {
        readBuffer[i] = readBuffer[indexOffset + i];
    }

    #if (DEBUG_MODE & ((1 << DM_NWRECV) | (1 << DM_NWPKT) | (1 << DM_NWDATA))) != 0
    DebugPrintLine();
    #endif
}

#define MPU6060_INT_PIN 2
MPU6050 mpu;
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
        setupFail();
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    if(devStatus != 0)
    {
        DebugPrintN(DM_SETUP, "Fail: DMP, "); DebugPrintLineN(DM_SETUP, devStatus);
        setupFail();
    }

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(MPU6060_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

bool readDMP(void)
{
    return (mpu.getIntStatus() & 0b00001) != 0 && mpu.dmpGetCurrentFIFOPacket(fifoBuffer) != 0 && mpu.dmpGetQuaternion(&inputdata.rotation, fifoBuffer) == 0;
}

void proximityHalt(void)
{
    distance = sonicSensor.GetDistance();
    if(obstacleNear() && movingForwards())
    {
        DebugPrintLineN(DM_GENERIC, "Halt: Obstruction");
        halt();
    }
}

void timedHalt(void)
{
    if((long)(millis() - nextAutoHalt) >= 0L)
    {
        halt();
        nextAutoHalt = millis() + KEEPALIVE_INTERVAL;

        DebugPrintLineN(DM_GENERIC, "Halt: Timeout");
    }
}

void setup(void)
{
    delay(2500);
    Serial.begin(9600, SERIAL_8N1);
    DebugPrintLineN(DM_SETUP, "Initializing...");

    pinMode(PIN_SUCCESSLED, OUTPUT);
    digitalWrite(PIN_SUCCESSLED, LOW);
    pinMode(PIN_FAILLED, OUTPUT);
    digitalWrite(PIN_FAILLED, LOW);

    pinMode(PIN_RECVLED, OUTPUT);
    digitalWrite(PIN_RECVLED, LOW);

    DataSerial.begin(115200, SERIAL_8N1);
    DataSerial.setTimeout(50UL);

    motors.Begin();

    setupMPU6050();

    setStatusLED(true);

    DebugPrintLineN(DM_SETUP, "Done");
    DebugPrintLineN(DM_SETUP);
    DebugFlushN(DM_SETUP);
}

void loop(void)
{
    proximityHalt();
    timedHalt();

    readDMP();

    recvData();

    delay(LOOP_DELAY);
}
