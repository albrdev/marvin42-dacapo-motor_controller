#ifndef __BUTTON_HPP__
#define __BUTTON_HPP__

#include <stdint.h> /* uint8_t */
#include "Component.hpp"

class Button : private Component<1>
{
public:
    /*! OnStateChangedEventHandler().
    \fn OnStateChangedEventHandler().
    \param .
    \return .
    */
    typedef void(*OnStateChangedEventHandler)(const bool);

private:
    bool m_State;

    OnStateChangedEventHandler m_OnStateChangedEvent = nullptr;

public:
    /*! GetState().
    \fn GetState().
    \param .
    \return .
    */
    bool GetState(void) const;
    
    /*! SetOnStateChangedEvent().
    \fn SetOnStateChangeEvent().
    \param .
    \return .
    */
    void SetOnStateChangedEvent(const OnStateChangedEventHandler value);

    /*! Poll().
    \fn Poll().
    \param .
    \return .
    */
    virtual void Poll(void);

    Button(const uint8_t pin);
};

#endif // __BUTTON_HPP__
