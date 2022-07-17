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
  CRTPPacket packet;
  while (true) {
    packet.header = CRTP_HEADER(CRTP_PORT_TRANSFER_OLSR, TOTAL_TRANS);
    olsrPacket payload = {
      .test1 = 1,
      .test2 = 2, 
      .test3 = 3, 
      .test4 = 4.04
    };
    memcpy(packet.data, payload.raw, sizeof(payload));
    packet.size = sizeof(payload);
    DEBUG_PRINT("CRTP_TX_TASK\n");
    crtpSendPacket(&packet);
    vTaskDelay(M2T(100));
  }
}

void crtpTransOlsrInit() {
    xTaskCreate(crtpTxOlsrTask, CRTP_TX_TEST, 1 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &crtpTxHandle);   
}

