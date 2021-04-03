#include "adHocApp.h"
#include "olsrAlgo.h"
#include "olsrDebug.h"
#include "uwbOlsr.h"
#include "log.h"
#define SENDER_ADDRESS 1
#define RECEIVER_ADDRESS 2
#define TOTAL_PACKET_NUM 10000
bool isSender = false;
bool isReceiver = false;
float receiveRate = 0.0;
static uint16_t sendCount = 0;
static uint16_t recvCount = 0;
void endToEndTask() {
  adHocAppInit(ADHOC_PORT_END_TO_END_DELAY);

  if (myAddress == RECEIVER_ADDRESS) {
    isReceiver = true;
  } else if (myAddress == SENDER_ADDRESS) {
    isSender = true;
  }
  if (isReceiver) {
    DEBUG_PRINT_OLSR_APP("Im Receiver!\n");
  }
  if (isSender) {
    DEBUG_PRINT_OLSR_APP("Im Sender!\n");
  }
  if (!isReceiver && !isSender) {
    DEBUG_PRINT_OLSR_APP("Im Route!\n");
  }
  int seq = 1;
  int firstSeq = 0;
  while (1) {
    if (isSender && sendCount < TOTAL_PACKET_NUM) {
      olsrSendData(myAddress,
                   ADHOC_PORT_END_TO_END_DELAY,
                   RECEIVER_ADDRESS,
                   ADHOC_PORT_END_TO_END_DELAY,
                   seq++,
                   NULL,
                   0);
      sendCount++;
    }
    if (isReceiver) {
      olsrDataMessage_t msg;
      adHocGetfromQueue(ADHOC_PORT_END_TO_END_DELAY, &msg);
      recvCount++;
      if (!firstSeq) {
        firstSeq = msg.m_dataHeader.m_seq;
      }
      receiveRate = recvCount * 100.0 / (msg.m_dataHeader.m_seq - firstSeq + 1);
      DEBUG_PRINT_OLSR_APP("Recv: %d; Seq: %d; Rate: %.2f.\n", recvCount, msg.m_dataHeader.m_seq, receiveRate);
    }
    // DEBUG_PRINT_OLSR_APP("recv pack:%d\n",recvCount);
    vTaskDelay(50);
  }
}
LOG_GROUP_START(ReceiveRate)
        LOG_ADD(LOG_FLOAT, receiveRate, &receiveRate)
        LOG_ADD(LOG_UINT16, receiveCount, &recvCount)
        LOG_ADD(LOG_UINT16, sendCount, &sendCount)
LOG_GROUP_STOP(RECEIVE_RATE)