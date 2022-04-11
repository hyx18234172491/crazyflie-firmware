#ifndef __ROUTINGALGO_H__
#define __ROUTINGALGO_H__

#include "mac.h"

#include "rangingProtocolPacket.h"

#define R_INTERVAL 500

enum ROUTING_TYPE{
    ROUTING_TO_LOCAL = 0,
    ROUTING_TO_NEXT = 1,
    ROUTING_TO_OTHER = 2,
};

void Routing();
bool GenerateR(packet_t* txRPacket, uint16_t destinationAddress, uint8_t timeToLive);
bool DispatchR(message_t* message);

#endif /* __ROUTINGALGO_H__ */