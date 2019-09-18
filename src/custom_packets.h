#ifndef __CUSTOM_PACKETS_H__
#define __CUSTOM_PACKETS_H__

#include <stdint.h> /* uint32_t */
#include <string.h> /* size_t, memcpy() */
#include "packet.h"

typedef struct _packet_motorrun packet_motorrun_t;

enum CustomPacketType
{
    CPT_STATUS = PT_MAX,
    CPT_MOTORRUN,
    CPT_MOTORSTOP
};

struct __attribute__((packed)) _packet_motorrun
{
    packet_header_t header;

    uint32_t left;
    uint32_t right;
};

#ifdef __cplusplus
extern "C"
{
#endif
    void packet_mkmotorrun(struct _packet_motorrun *const pkt, const float left, const float right);
#ifdef __cplusplus
} // extern "C"
#endif

#endif // __CUSTOM_PACKETS_H__
