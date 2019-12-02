#ifndef __DCMOTORASSEMBLY_HPP__
#define __DCMOTORASSEMBLY_HPP__

#include <stdint.h>
#include <Adafruit_MotorShield.h>
#include "maths.h"

class DCMotorAssembly
{
private:
    static Adafruit_MotorShield s_Shield;
    static bool s_ShieldInitialized;

    const static size_t k_DeviceCount = 4U;
    Adafruit_DCMotor* m_Devices[k_DeviceCount];

    const static size_t k_VectorLength = 3U;
    const static size_t k_DirectionCount = k_VectorLength * k_VectorLength;
    static const float k_DirectionMap[k_VectorLength][k_VectorLength][k_DeviceCount];

public:
    void Run(const float directionX, const float directionY, float power = 1.0f);

    void SetSpeed(const float value);
    void SetSpeed(const size_t port, const float value);

    void SetLeftSpeed(const float value);
    void SetRightSpeed(const float value);
    void SetFrontSpeed(const float value);
    void SetBackSpeed(const float value);

    void SetXSpeed(const float left, const float right);
    void SetYSpeed(const float up, const float down);

    void Rotate(float value);

    void Halt(void);
    void Halt(const size_t port);

    void Begin(void);

    DCMotorAssembly(void);
    virtual ~DCMotorAssembly(void);
};

#endif // __DCMOTORASSEMBLY_HPP__
