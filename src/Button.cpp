#include "Button.hpp"
#include <Arduino.h>    /* pinMode(), digitalRead() */

bool Button::GetState(void) const
{
    return digitalRead(m_Pin);
}

void Button::SetOnStateChangedEvent(const OnStateChangedEventHandler value)
{
    m_OnStateChangedEvent = value;
}

void Button::Poll(void)
{
    bool state = digitalRead(m_Pin);
    if(state != m_State)
    {
        m_State = state;
        if(m_OnStateChangedEvent != nullptr)
        {
            m_OnStateChangedEvent(m_State);
        }
    }
}

Button::Button(const uint8_t pin) : m_Pin(pin)
{
    pinMode(m_Pin, INPUT_PULLUP);
    m_State = digitalRead(m_Pin);
}
