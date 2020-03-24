#ifndef __HC_SR04_HPP__
#define __HC_SR04_HPP__

#include <stdint.h> /* uint8_t */
#include "Component.hpp"

class HC_SR04 : private Component<2>
{
protected:
    static constexpr float SPEED_OF_SOUND = 343.0f * (100.0f / 1000000.0f); // cm/us

    bool AwaitPulse(const uint8_t value, const unsigned long int timeout, unsigned long int& result) const;

public:
    static constexpr float DISTANCE_MAX = 400.0f; // cm
    static constexpr unsigned long int TIMEOUT_MAX = ((DISTANCE_MAX / SPEED_OF_SOUND) * 2); // us

    float GetDistance(const unsigned long int timeout = HC_SR04::TIMEOUT_MAX) const;

    HC_SR04(const uint8_t triggerPin, const uint8_t echoPin);
};

#endif // __HC_SR04_HPP__
