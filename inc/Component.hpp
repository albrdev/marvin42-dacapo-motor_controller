#ifndef __COMPONENT_HPP__
#define __COMPONENT_HPP__

#include <stddef.h>
#include <stdint.h>

template <size_t N>
class Component
{
protected:
    uint8_t m_Pins[N];

    template <typename ... I>
    Component(const I& ... i) : m_Pins{ i... } { }
};

#endif // __COMPONENT_HPP__
