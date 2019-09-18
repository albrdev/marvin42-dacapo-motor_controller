#ifndef __PACKET_H__
#define __PACKET_H__

#include <stddef.h> /* size_t */
#include <stdint.h> /* uint8_t, uint16_t */

typedef struct _packet_header packet_header_t;

enum PacketType
{
    PT_FALSE    = 0,
    PT_TRUE     = 1,
    PT_MAX      = 2
};

struct __attribute__((packed)) _packet_header
{
    uint16_t chksum_header;
    uint16_t chksum_data;
    uint8_t type;
    uint16_t size;
};

#ifdef __cplusplus
extern "C"
{
#endif
    int packet_verifyheader(const struct _packet_header *const pkt);
    int packet_verifydata(const void *const pkt);
    void packet_mkheader(struct _packet_header *const pkt, const size_t size, const uint8_t type);
    void packet_mkbasic(void *const pkt, const uint8_t type);
#ifdef __cplusplus
} // extern "C"
#endif

#endif // __PACKET_H__
