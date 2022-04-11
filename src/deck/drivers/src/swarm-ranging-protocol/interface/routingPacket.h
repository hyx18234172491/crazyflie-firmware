#ifndef __ROUTINGPACKET_H__
#define __ROUTINGPACKET_H__

#include "mac.h"

// Routing message
typedef struct
{
    uint16_t originatorAddress;
    uint16_t destinationAddress;
    uint16_t nextAddress;
    uint16_t sequence;
    uint16_t size;
    uint8_t timeToLive;
} __attribute__((packed)) rMessageHeader_t; // 11 bytes

#endif // __ROUTINGPACKET_H__