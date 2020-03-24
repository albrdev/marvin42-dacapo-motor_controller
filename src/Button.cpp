#include "Button.hpp"
#include <Arduino.h>    /* pinMode(), digitalRead() */

bool Button::GetState(void) const
{
    return digitalRead(m_Pins[0]);
}

void Button::SetOnStateChangedEvent(const OnStateChangedEventHandler value)
{
    m_OnStateChangedEvent = value;
}

void Button::Poll(void)
{
    bool state = digitalRead(m_Pins[0]);
    if(state != m_State)
    {
        m_State = state;
        if(m_OnStateChangedEvent != nullptr)
        {
            m_OnStateChangedEvent(m_State);
        }
    }
}

Button::Button(const uint8_t pin) : Component<1>(pin)
{
    pinMode(m_Pins[0], INPUT_PULLUP);
    m_State = digitalRead(m_Pins[0]);
}
