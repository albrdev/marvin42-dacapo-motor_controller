#include "DCMotorAssembly.hpp"
#include <math.h>
#include "maths.h"

const float DCMotorAssembly::k_DirectionMap[k_VectorLength][k_VectorLength][k_DeviceCount] =
{
    // Down row
    {
        { -1, 0, 0, -1 },   // 0, Down-Left
        { -1, -1, -1, -1 }, // 1, Down-Middle
        { 0, -1, -1, 0 }    // 2, Down-Right
    },

    // Middle row
    {
        { -1, 1, 1, -1 },   // 3, Middle-Left
        { 0, 0, 0, 0 },     // 4, Center (Stopped)
        { 1, -1, -1, 1 }    // 5, Middle-Right
    },

    // Up row
    {
        { 0, 1, 1, 0 },     // 6, Up-Left
        { 1, 1, 1, 1 },     // 7, Up-Middle
        { 1, 0, 0, 1 }      // 8, Up-Right
    }
};

Adafruit_MotorShield DCMotorAssembly::s_Shield(0x61);
bool DCMotorAssembly::s_ShieldInitialized = false;

bool DCMotorAssembly::IsRunning(void) const
{
    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        if(IsRunning())
        {
            return true;
        }
    }

    return false;
}

bool DCMotorAssembly::IsRunning(const size_t port) const
{
    return m_Devices[port].power != 0.0f;
}

void DCMotorAssembly::Run(const float directionX, const float directionY, float power)
{
    if(power == 0.0f || (directionX == 0.0f && directionY == 0.0f))
    {
        Halt();
        return;
    }

    const float maxIndex = DCMotorAssembly::k_VectorLength - 1U;
    size_t x = (size_t)denormalize11(clamp11((float)roundf(directionX)), 0.0f, maxIndex);
    size_t y = (size_t)denormalize11(clamp11((float)roundf(directionY)), 0.0f, maxIndex);

    power = clamp01(power);
    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        SetSpeed(i, DCMotorAssembly::k_DirectionMap[y][x][i] * power);
    }
}

void DCMotorAssembly::SetSpeed(const float value)
{
    if(value == 0.0f)
    {
        Halt();
        return;
    }

    uint8_t speed = denormalize01(fabs(value), 0, 255);
    uint8_t direction = value < 0.0f ? BACKWARD : FORWARD;

    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        m_Devices[i].device->setSpeed(speed);
        m_Devices[i].device->run(direction);
        m_Devices[i].power = value;
    }
}

void DCMotorAssembly::SetSpeed(const size_t port, const float value)
{
    if(value == 0.0f)
    {
        Halt(port);
        return;
    }

    m_Devices[port].device->setSpeed(denormalize01(fabsf(value), 0, 255));
    m_Devices[port].device->run(value < 0.0f ? BACKWARD : FORWARD);
    m_Devices[port].power = value;
}

void DCMotorAssembly::SetLeftSpeed(const float value)
{
    SetSpeed(0, value);
    SetSpeed(2, value);
}

void DCMotorAssembly::SetRightSpeed(const float value)
{
    SetSpeed(1, value);
    SetSpeed(3, value);
}

void DCMotorAssembly::SetFrontSpeed(const float value)
{
    SetSpeed(0, value);
    SetSpeed(1, value);
}

void DCMotorAssembly::SetBackSpeed(const float value)
{
    SetSpeed(2, value);
    SetSpeed(3, value);
}

void DCMotorAssembly::SetXSpeed(const float left, const float right)
{
    SetLeftSpeed(left);
    SetRightSpeed(right);
}

void DCMotorAssembly::SetYSpeed(const float up, const float down)
{
    SetFrontSpeed(up);
    SetBackSpeed(down);
}

void DCMotorAssembly::Rotate(const int direction, const float power)
{
    if(direction == 0 || power == 0.0f)
    {
        Halt();
        return;
    }

    float tmp = clamp11(direction) * clamp01(power);
    SetXSpeed(tmp, -tmp);
}

void DCMotorAssembly::Halt(void)
{
    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        Halt(i);
    };
}

void DCMotorAssembly::Halt(const size_t port)
{
    if(!IsRunning(port))
    {
        return;
    }

    m_Devices[port].device->setSpeed(0);
    //m_Devices[port].device->run(RELEASE);
    m_Devices[port].power = 0.0f;
}

void DCMotorAssembly::Begin(void)
{
    if(!s_ShieldInitialized)
    {
        s_Shield.begin();
        s_ShieldInitialized = true;
    }

    for(uint8_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        m_Devices[i].device = s_Shield.getMotor(i + 1U);
    }
}

DCMotorAssembly::DCMotorAssembly(void) { }

DCMotorAssembly::~DCMotorAssembly(void)
{
    //Halt();
}
