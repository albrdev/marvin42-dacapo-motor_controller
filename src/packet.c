#include "packet.h"
#include "crc.h"

int packet_verifyheader(const struct _packet_header *const pkt)
{
    return mkcrc16((uint8_t *)pkt + sizeof(pkt->chksum_header), sizeof(*pkt) - sizeof(pkt->chksum_header)) == pkt->chksum_header;
}

int packet_verifydata(const void *const pkt)
{
    const struct _packet_header *const hdr = (const struct _packet_header *const)pkt;
    return mkcrc16((uint8_t *)pkt + sizeof(*hdr), hdr->size) == hdr->chksum_data;
}

void packet_mkheader(struct _packet_header *const pkt, const size_t size, const uint8_t type)
{
    // Checmsum has to be calculated last
    // Checksum can't be taken into account for the checksum itself, therefore start at an offset to avoid it
    // The length of the data to be calculated over is now total size minus the checksum size to avoid going outside allowed memory

    // Packet in bits: C = checksum, T = type, D = data (size may vary)
    // CCCCCCCCCCCCCCCCTTTTTTTTDDDDDDDDDDDDDDDD
    //                 ^---------------------->| size - sizeof(checksum)

    pkt->type = type;
    pkt->size = (uint16_t)(size - sizeof(struct _packet_header));
    pkt->chksum_data = mkcrc16((uint8_t *)pkt + sizeof(*pkt), pkt->size);
    pkt->chksum_header = 0U;
    pkt->chksum_header = mkcrc16((uint8_t *)pkt + sizeof(pkt->chksum_header), sizeof(*pkt) - sizeof(pkt->chksum_header));
}

void packet_mkbasic(void *const pkt, const uint8_t type)
{
    packet_mkheader((struct _packet_header *const)pkt, sizeof(struct _packet_header), type);
}
