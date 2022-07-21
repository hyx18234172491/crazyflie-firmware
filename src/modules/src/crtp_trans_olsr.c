#include "crtp_trans_olsr.h"
#include "crtp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "config.h"
#include "debug.h"
#include "string.h"

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

olsrPacket olsrPacketList[1000];
static int numberOfPacket = 1000;
static int packetNum = 0;
static bool genetratePacket() 
{
  for (int i = 0; i < numberOfPacket; i++)
  {
    olsrPacket p = {
      .test1 = i,
      .test2 = i,
      .test3 = i * 2,
      .test4 = i * 1.1
    };
    olsrPacketList[i] = p;
  }
}

static void crtpTxOlsrTask(void* parameters) {
  CRTPPacket packet;
  olsrPacket pk;
  genetratePacket();
  while (true) {
    if (packetNum >= 1000) 
    {
      packetNum = 0;
    }
    int offset = 0;
    while (packetNum < 1000)
    {
      pk = olsrPacketList[packetNum];
      //uint8_t payload[27];
      memcpy(packet.data + offset, pk.raw, sizeof(pk));
      offset += sizeof(pk);
      packetNum++;
      //DEBUG_PRINT("CRTP_TX_TASK:%d\n", packetNum);
      vTaskDelay(M2T(100));
      
      if (packetNum == numberOfPacket)
      {
        packet.header = CRTP_HEADER(CRTP_PORT_TRANSFER_OLSR, END_TOTAL_TRANS);
        packet.size = offset;
        DEBUG_PRINT("CRTP_TX_TASK END\n");
        crtpSendPacket(&packet);
        offset = 0;
      }
      else if (packetNum % 3 == 0)
      {
        packet.header = CRTP_HEADER(CRTP_PORT_TRANSFER_OLSR, TOTAL_TRANS);
        packet.size  = offset;
        // memcpy(packet.data, payload, sizeof(payload));
        DEBUG_PRINT("CRTP_TX_TASK:%d\n", packetNum);
        crtpSendPacket(&packet);
        offset = 0;
      }
    }
    vTaskDelay(M2T(200));
  }
}

void crtpTransOlsrInit() {
    xTaskCreate(crtpTxOlsrTask, CRTP_TRANS_OLSR, 1 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &crtpTxHandle);   
}

