#ifndef __COMPONENT_HPP__
#define __COMPONENT_HPP__

#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>

template <size_t N>
class Component
{
protected:
    uint8_t m_Pins[N];

    template <typename ... I>
    Component(const I& ... i) : m_Pins { i... } { }
};

template <size_t N>
class Output
{
protected:
    uint8_t m_Pins[N];

    template <typename ... I>
    Output(const I& ... i) : m_Pins{ i... }
    {
        pinMode(i..., OUTPUT);
    }
};

template <size_t N>
class Input
{
protected:
    uint8_t m_Pins[N];

    template <typename ... I>
    Input(const I& ... i) : m_Pins{ i... }
    {
        pinMode(i..., INPUT);
    }
};

#endif // __COMPONENT_HPP__
