#include "generic.hpp"
#include <stdarg.h>     /* vsnprintf() */
#include <Arduino.h>    /* Serial, min() */

static char _format_buf[128 + 1];
int spprintf(const char* const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(_format_buf, sizeof(_format_buf), fmt, args);
    va_end(args);

    Serial.print(_format_buf);
    return len;
}

int spprintf(SoftwareSerial& serialPort, const char* const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(_format_buf, sizeof(_format_buf), fmt, args);
    va_end(args);

    serialPort.print(_format_buf);
    return len;
}

#define HEXSTRBUF_SIZE 64
char const _hexchar_map[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
static char _hexstr_buf[HEXSTRBUF_SIZE + 1];
const char* hexstr(const void* const data, const size_t size)
{
    const unsigned char* ptr = (const unsigned char*)data;
    size_t max = min(size, HEXSTRBUF_SIZE);
    size_t i;
    for(i = 0U; i < max; i++)
    {
        _hexstr_buf[i] += _hexchar_map[(ptr[i] & 0xF0) >> 4];
        _hexstr_buf[i] += _hexchar_map[(ptr[i] & 0x0F)];
    }

    _hexstr_buf[i] = '\0';
    return _hexstr_buf;
}
