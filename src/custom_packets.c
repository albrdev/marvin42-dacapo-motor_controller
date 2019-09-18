#include "custom_packets.h"
#include "crc.h"

void packet_mkmotorrun(struct _packet_motorrun *const pkt, const float left, const float right)
{
    memcpy(&pkt->left, &left, sizeof(pkt->left));
    memcpy(&pkt->right, &right, sizeof(pkt->right));

    packet_mkheader(&pkt->header, sizeof(*pkt), CPT_MOTORRUN);
}
