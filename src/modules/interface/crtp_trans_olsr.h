#ifndef _CRTP_TRANS_OLSR_H_
#define _CRTP_TRANS_OLSR_H_
#include <stdint.h>

typedef union olsrPacket_u
{
  uint8_t raw[30];
  struct {
      uint8_t test1;
      uint16_t test2;
      uint16_t test3;
      float test4;
  } __attribute__((packed)) ;
}__attribute__((packed)) olsrPacket;

void crtpTransOlsrInit();
#endif /* _CRTP_TRANS_OLSR_H_ */