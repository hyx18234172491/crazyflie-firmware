#ifndef _CRTP_TRANS_OLSR_H_
#define _CRTP_TRANS_OLSR_H_
#include <stdint.h>

#define OLSR_UNIT_NUMBER 3

typedef struct olsrPacketHeader {
  uint8_t sender;
  uint16_t seqNumber;
} __attribute__((packed)) olsrPacketHeader_t;

typedef struct olsrUnit {
  uint8_t neighbor1;
  uint8_t neighbor2;
} __attribute__((packed)) olsrUnit_t;

typedef struct olsrPacketPayload {
  olsrUnit_t units[OLSR_UNIT_NUMBER];
} __attribute__((packed)) olsrPacketPayload_t;

#define OLSR_PACKET_SIZE (sizeof(olsrPacketHeader_t) + sizeof(olsrPacketPayload_t))

typedef struct olsrPacket {
  union {
    struct {
      olsrPacketHeader_t header;
      olsrPacketPayload_t payload;
    };
    uint8_t raw[OLSR_PACKET_SIZE];
  };
} __attribute__((packed)) OLSRPacket;

void crtpTransOlsrInit();
#endif /* _CRTP_TRANS_OLSR_H_ */