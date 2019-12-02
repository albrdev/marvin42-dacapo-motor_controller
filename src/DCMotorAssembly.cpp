#include "DCMotorAssembly.hpp"

Adafruit_MotorShield DCMotorAssembly::s_Shield;
bool DCMotorAssembly::s_ShieldInitialized = false;

void DCMotorAssembly::Run(const float directionX, const float directionY, float power)
{
    size_t x = (size_t)denormalize11(clamp11(directionX), 0, DCMotorAssembly::k_VectorLength - 1U);
    size_t y = (size_t)denormalize11(clamp11(directionY), 0, DCMotorAssembly::k_VectorLength - 1U);

    power = clamp01(power);
    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        SetSpeed(i, DCMotorAssembly::k_DirectionMap[x][y][i] * power);
    }
}

void DCMotorAssembly::SetSpeed(const float value)
{
    uint8_t speed = denormalize01(fabs(value), 0, 255);
    uint8_t direction = sgn(value) > 0 ? FORWARD : BACKWARD;
    if(value == 0.0f)
    {
        Halt();
    }

    for(size_t i = 0U; i < DCMotorAssembly::k_DeviceCount; i++)
    {
        m_Devices[i]->setSpeed(speed);
        m_Devices[i]->run(direction);
    }
}

void DCMotorAssembly::SetSpeed(const size_t port, const float value)
{
    if(value == 0.0f)
    {
        Halt(port);
    }

    m_Devices[port]->setSpeed(denormalize01(fabsf(value), 0, 255));
    m_Devices[port]->run(sgn(value) > 0 ? FORWARD : BACKWARD);
    m_Devices[port]->run(RELEASE);
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

void DCMotorAssembly::Rotate(float value)
{
    if(value == 0.0f)
    {
        Halt();
        return;
    }

    value = clamp11(value);
    SetXSpeed(value, -value);
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
    m_Devices[port]->setSpeed(0);
    //m_Devices[port]->run(RELEASE);
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
        m_Devices[i] = s_Shield.getMotor(i + 1U);
    }
}

DCMotorAssembly::DCMotorAssembly(void)
{
}

DCMotorAssembly::~DCMotorAssembly(void)
{
}
