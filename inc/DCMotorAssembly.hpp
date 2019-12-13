#ifndef __DCMOTORASSEMBLY_HPP__
#define __DCMOTORASSEMBLY_HPP__

#include <stdint.h>
#include <Adafruit_MotorShield.h>

class DCMotorAssembly
{
private:
    static Adafruit_MotorShield s_Shield; /*!< TODO */
    static bool s_ShieldInitialized; /*!< TODO */

    const static size_t k_DeviceCount = 4U; /*!< TODO */
    Adafruit_DCMotor* m_Devices[k_DeviceCount]; /*!< TODO */

    const static size_t k_VectorLength = 3U; /*!< TODO */
    const static size_t k_DirectionCount = k_VectorLength * k_VectorLength; /*!< TODO */
    static const float k_DirectionMap[k_VectorLength][k_VectorLength][k_DeviceCount]; /*!< TODO */

public:
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
