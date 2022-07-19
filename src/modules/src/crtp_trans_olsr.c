#include <string.h>

#include "crtp_trans_olsr.h"
#include "crtp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "config.h"
#include "debug.h"

typedef enum
{
  START_TOTAL_TRANS   = 0,
  END_TOTAL_TRANS     = 1,
  TOTAL_TRANS         = 2,
  ADD_OLSR_PACKET     = 3,
  DELETE_OLSR_PACKET  = 4,
  MODIFY_OLSR_PACKET  = 5
} transOlsrChannels_t;

static TaskHandle_t crtpTxHandle = 0;

static void crtpTxOlsrTask(void* parameters) {
  CRTPPacket crtpPacket;
  OLSRPacket olsrPacket;
  uint16_t seqNumber = 1;
  while (true) {
    crtpPacket.header = CRTP_HEADER(CRTP_PORT_TRANSFER_OLSR, TOTAL_TRANS);
    olsrPacket.header.sender = 1;
    olsrPacket.header.seqNumber = seqNumber;
    olsrPacket.payload.units[0].neighbor1 = 3;
    olsrPacket.payload.units[0].neighbor2 = 4;
    olsrPacket.payload.units[1].neighbor1 = 5;
    olsrPacket.payload.units[1].neighbor2 = 6;
    olsrPacket.payload.units[2].neighbor1 = 7;
    olsrPacket.payload.units[2].neighbor2 = 8;
    memcpy(crtpPacket.data, olsrPacket.raw, sizeof(olsrPacket));
    crtpPacket.size = sizeof(olsrPacket);
    DEBUG_PRINT("CRTP_TX_TASK_SIZE: %d\n", sizeof(olsrPacket));
    DEBUG_PRINT("CRTP_TX_TASK\n");
    crtpSendPacket(&crtpPacket);
    seqNumber++;
    vTaskDelay(M2T(100));
  }
}

void crtpTransOlsrInit() {
    xTaskCreate(crtpTxOlsrTask, CRTP_TRANS_OLSR, 1 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &crtpTxHandle);   
}

