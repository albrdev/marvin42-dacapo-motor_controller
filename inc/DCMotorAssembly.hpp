#ifndef __DCMOTORASSEMBLY_HPP__
#define __DCMOTORASSEMBLY_HPP__

#include <stdint.h>
#include <Adafruit_MotorShield.h>

class DCMotorAssembly
{
private:
    static Adafruit_MotorShield s_Shield; /*!< TODO */
    static bool s_ShieldInitialized; /*!< TODO */

    static const size_t k_DeviceCount = 4U; /*!< TODO */
    struct
    {
        Adafruit_DCMotor* device;
        float power;
    } m_Devices[k_DeviceCount] =
    {
        { nullptr, 0.0f },
        { nullptr, 0.0f },
        { nullptr, 0.0f },
        { nullptr, 0.0f }
    }; /*!< TODO */

    static const size_t k_VectorLength = 3U; /*!< TODO */
    static const size_t k_DirectionCount = k_VectorLength * k_VectorLength; /*!< TODO */
    static const float k_DirectionMap[k_VectorLength][k_VectorLength][k_DeviceCount]; /*!< TODO */

public:
    /*! IsRunning().
    \fn IsRunning().
    \param .
    \return .
    */
    bool IsRunning(void) const;

    /*! IsRunning().
    \fn IsRunning().
    \param .
    \return .
    */
    bool IsRunning(const size_t port) const;

    /*! Run().
    \fn Run().
    \param .
    \return .
    */
    void Run(const float directionX, const float directionY, float power = 1.0f);

    /*! SetSpeed().
    \fn SetSpeed().
    \param .
    \return .
    */
    void SetSpeed(const float value);
    
    /*! SetSpeed().
    \fn SetSpeed().
    \param .
    \return .
    */
    void SetSpeed(const size_t port, const float value);
    
    /*! SetLeftSpeed().
    \fn SetLeftSpeed().
    \param .
    \return .
    */
    void SetLeftSpeed(const float value);
    
    /*! SetRightSpeed().
    \fn SetRightSpeed().
    \param .
    \return .
    */
    void SetRightSpeed(const float value);

    /*! SetFrontSpeed().
    \fn SetFrontSpeed().
    \param .
    \return .
    */
    void SetFrontSpeed(const float value);

    /*! SetBackSpeed().
    \fn SetBackSpeed().
    \param .
    \return .
    */
    void SetBackSpeed(const float value);

    /*! SetXSpeed().
    \fn SetXSpeed().
    \param .
    \return .
    */
    void SetXSpeed(const float left, const float right);

    /*! SetYSpeed().
    \fn SetYSpeed().
    \param .
    \return .
    */
    void SetYSpeed(const float up, const float down);

    /*! Rotate().
    \fn Rotate().
    \param .
    \return .
    */
    void Rotate(const int direction, const float power);

    /*! Halt().
    \fn Halt().
    \param .
    \return .
    */
    void Halt(void);

    /*! Halt().
    \fn Halt().
    \param .
    \return .
    */
    void Halt(const size_t port);

    /*! Begin().
    \fn Begin().
    \param .
    \return .
    */
    void Begin(void);

    /*! DCMotorAssembly().
    \fn DCMotorAssembly().
    \param .
    \return .
    */
    DCMotorAssembly(void);
    virtual ~DCMotorAssembly(void);
};

#endif // __DCMOTORASSEMBLY_HPP__
