#include "crc.h"
#ifdef __AVR__
#include <util/crc16.h>
#else
// https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1ga95371c87f25b0a2497d9cba13190847f.html
static uint16_t _crc16_update(uint16_t crc, uint8_t a)
{
    uint8_t i;

    crc = (uint16_t)(crc ^ a);
    for(i = 0U; i < 8U; i++)
    {
        crc = (crc & 1) != 0 ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }

    return crc;
}
#endif

uint16_t mkcrc16(const void* const data, const size_t size)
{
    const uint8_t* const tmpData = (const uint8_t*)data;
    uint16_t result = 0U;
    for(size_t i = 0U; i < size; i++)
    {
        result = _crc16_update(result, tmpData[i]);
    }

    return result;
}
