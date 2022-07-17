#include "testcrtp.h"
#include "crtp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "config.h"
#include "debug.h"

static TaskHandle_t crtpTxHandle = 0;

static void crtpTxTask(void* parameters) {
  CRTPPacket packet;
  while (true) {
    packet.header = CRTP_HEADER(9, 1);
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

void testcrtpInit() {
    xTaskCreate(crtpTxTask, CRTP_TX_TEST, 1 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &crtpTxHandle);   
}

