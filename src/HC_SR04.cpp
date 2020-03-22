#include "HC_SR04.hpp"
#include <Arduino.h>

bool HC_SR04::AwaitPulse(const uint8_t value, unsigned long int timeout, unsigned long int &result) const
{
    unsigned long int start;

    start = micros();
    timeout += start;
    while(digitalRead(m_Pins[1]) != value)
    {
        if((long)(micros() - timeout) >= 0L)
        {
            return false;
        }
    }

    start = micros();
    while(digitalRead(m_Pins[1]) == value)
    {
        if((long)(micros() - timeout) >= 0L)
        {
            return false;
        }
    }

    result = micros() - start;
    return true;
}

float HC_SR04::GetDistance(const unsigned long int timeout) const
{
    unsigned long int duration;

    digitalWrite(m_Pins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(m_Pins[0], LOW);

    return AwaitPulse(HIGH, timeout, duration) ? (duration * SPEED_OF_SOUND) / 2.0f : -1.0f;
}

HC_SR04::HC_SR04(const uint8_t triggerPin, const uint8_t echoPin) : Component { triggerPin, echoPin }
{
    pinMode(m_Pins[0], OUTPUT);
    pinMode(m_Pins[1], INPUT);

    digitalWrite(m_Pins[0], LOW);
    //delayMicroseconds(2);
}
