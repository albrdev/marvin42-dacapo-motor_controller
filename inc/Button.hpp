#ifndef __BUTTON_HPP__
#define __BUTTON_HPP__

#include <stdint.h> /* uint8_t */

class Button
{
public:
    typedef void(*OnStateChangedEventHandler)(const bool);

private:
    uint8_t m_Pin;
    bool m_State;

    OnStateChangedEventHandler m_OnStateChangedEvent = nullptr;

public:
    bool GetState(void) const;
    void SetOnStateChangedEvent(const OnStateChangedEventHandler value);

    virtual void Poll(void);

    Button(const uint8_t pin);
};

#endif // __BUTTON_HPP__
