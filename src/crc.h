#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h> /* uint8_t, uint16_t */
#include <stdlib.h> /*size_t */

#ifdef __cplusplus
extern "C"
{
#endif
    uint16_t mkcrc16(const void* const data, const size_t size);
#ifdef __cplusplus
} // extern "C"
#endif

#endif // __CRC_H__
