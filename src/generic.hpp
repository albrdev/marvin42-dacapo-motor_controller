#ifndef __GENERIC_H__
#define __GENERIC_H__

#include <stdint.h> /* uint8_t, uint16_t */
#include <stdlib.h> /* size_t */
#include <SoftwareSerial.h>

int spprintf(const char* const fmt, ...);
int spprintf(SoftwareSerial& serialPort, const char* const fmt, ...);

#ifdef __cplusplus
extern "C"
{
#endif
    const char* hexstr(const void* const data, const size_t size);
#ifdef __cplusplus
} // extern "C"
#endif

#endif // __GENERIC_H__
