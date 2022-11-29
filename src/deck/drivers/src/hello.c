#define DEBUG_MODULE "ROUTING"

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "routing.h"
#include "hello_struct.h"
#include "mprMessage.h"

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static TaskHandle_t uwbHelloTxTaskHandle = 0;
static TaskHandle_t uwbHelloRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;


void linkSensing(const olsrMessage_t *helloMsg);
void populateNeighborSet(const olsrMessage_t *helloMsg);
void populateTwoHopNeighborSet(const olsrMessage_t *helloMsg);
void mprCompute();
void populateMprSelectorSet(const olsrMessage_t *helloMsg);
void olsrPrintAll();

void helloRxCallback(void *parameters)
{
    DEBUG_PRINT("helloRxCallback \n");
}

void helloTxCallback(void *parameters)
{
    DEBUG_PRINT("helloTxCallback \n");
}

static uint16_t getMessageSeqNumber()
{
  return g_staticMessageSeq++; // quick access and return, no need to set lock
}

// void processHello(UWB_Packet_t *packet)
// {
//     for (int i = 0; i < m_Nei_Table.one_nei_number; i++)
//     {
//         if (message->header.srcAddress == m_Nei_Table.one_nei_address[i].address)
//         {
//             m_Nei_Table.one_nei_address[i].mtime = mtime;
//             uint8_t two_nei_number = (message->header.msgLength - 20) / sizeof(Body_Unit_t);
//             m_Nei_Table.one_nei_address[i].two_nei_number = two_nei_number;
//             for (int j = 0; j < two_nei_number; j++)
//             {
//                 m_Nei_Table.one_nei_address[i].two_nei_address[j] = message->bodyUnits[j].address;
//                 m_Nei_Table.one_nei_address[i].two_nei_distance[j] = message->bodyUnits[j].distance;
//             }
//         }
//         // else{}
//     }
//}

int generateHelloMessage(olsrMessage_t *message)
{
    int msgLen = sizeof(olsrMessage_t);
    // message->seqNumber = seqNumber++;
    message->m_messageHeader.m_messageType = HELLO_MESSAGE;
    message->m_messageHeader.m_vTime = OLSR_NEIGHB_HOLD_TIME;
    message->m_messageHeader.m_messageSize = sizeof(olsrMessageHeader_t);
    message->m_messageHeader.m_originatorAddress = myAddress;
    message->m_messageHeader.m_destinationAddress = 0;
    message->m_messageHeader.m_relayAddress = myAddress;
    message->m_messageHeader.m_timeToLive = 0xff;
    message->m_messageHeader.m_hopCount = 0;
    message->m_messageHeader.m_messageSeq = getMessageSeqNumber();

    // hello message
    olsrHelloMessage_t helloMessage;                          // 84
    helloMessage.m_helloHeader.m_hTime = OLSR_HELLO_INTERVAL; // hello's header on packet
    helloMessage.m_helloHeader.m_willingness = WILL_ALWAYS;
    helloMessage.m_helloHeader.m_linkMessageNumber = 0;

    // loop
    setIndex_t linkTupleIndex = olsrLinkSet.fullQueueEntry; // 2
    olsrTime_t now = xTaskGetTickCount();                   // 4
    while (linkTupleIndex != -1)
    {
        if (!(olsrLinkSet.setData[linkTupleIndex].data.m_localAddr == myAddress &&
              olsrLinkSet.setData[linkTupleIndex].data.m_expirationTime >= now))
        {
            linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
            continue;
        }
        uint8_t linkType, nbType = 0xff;

        if (olsrLinkSet.setData[linkTupleIndex].data.m_symTime >= now)
        {
            linkType = OLSR_SYM_LINK; // 2
        }
        else if (olsrLinkSet.setData[linkTupleIndex].data.m_asymTime >= now)
        {
            linkType = OLSR_ASYM_LINK; // 1
        }
        else
        {
            linkType = OLSR_LOST_LINK; // 3
        }
        if (olsrFindMprByAddr(&olsrMprSet, olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr)) // reg+3.4
        {
            nbType = OLSR_MPR_NEIGH; // 2
        }
        else
        {
            bool ok = false;
            setIndex_t neighborTupleIndex = olsrNeighborSet.fullQueueEntry;
            while (neighborTupleIndex != -1)
            {
                if (olsrNeighborSet.setData[neighborTupleIndex].data.m_neighborAddr ==
                    olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr) // this linkTuple Addr is in NighborSet
                {
                    if (olsrNeighborSet.setData[neighborTupleIndex].data.m_status == STATUS_SYM)
                    {
                        nbType = OLSR_SYM_NEIGH; // is a sym neighbor 1
                    }
                    else if (olsrNeighborSet.setData[neighborTupleIndex].data.m_status == STATUS_NOT_SYM)
                    {
                        nbType = OLSR_NOT_NEIGH; // is not a sym neghbor 0
                    }
                    else
                    {
                        DEBUG_PRINT_OLSR_HELLO("There is a neighbor tuple with an unknown status!\n");
                    }
                    ok = true;
                    break;
                }
                neighborTupleIndex = olsrNeighborSet.setData[neighborTupleIndex].next;
            }
            if (!ok)
            {
                linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
                continue;
            }
        }                              // TODO -1 in queue will be not dropped
        olsrLinkMessage_t linkMessage; // 6
        linkMessage.m_linkCode = (linkType & 0x03) | ((nbType << 2) & 0x0f);
        linkMessage.m_addressUsedSize = 1;
        linkMessage.m_addresses = olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr;
#ifdef USER_ROUTING
        // int16_t distTmp = getDistanceFromAddr(linkMessage.m_addresses);
        //  float lostrate=distanceToPacketLoss(distTmp);
        //  linkMessage.m_weight=1-lostrate;
        linkMessage.m_weight = distanceToWeight(getDistanceFromAddr(olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr));
        // DEBUG_PRINT_OLSR_ROUTING("to %d's distance is %d,weight is:%f\n",linkMessage.m_addresses,distTmp,linkMessage.m_weight);
#endif
        if (helloMessage.m_helloHeader.m_linkMessageNumber == LINK_MESSAGE_MAX_NUM)
            break;
        helloMessage.m_linkMessage[helloMessage.m_helloHeader.m_linkMessageNumber++] = linkMessage;
        linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
    }
    uint16_t writeSize = sizeof(olsrHelloMessageHeader_t) + helloMessage.m_helloHeader.m_linkMessageNumber *
                                                                sizeof(olsrLinkMessage_t);
    message->m_messageHeader.m_messageSize += writeSize;
    memcpy(message->m_messagePayload, &helloMessage, writeSize);

    return msgLen;
}

static void processHelloMessage(const olsrMessage_t *helloMsg)
{
    linkSensing(helloMsg);
    populateNeighborSet(helloMsg);
    populateTwoHopNeighborSet(helloMsg);
    mprCompute();
    populateMprSelectorSet(helloMsg);
    olsrPrintAll();
}

static void uwbHelloTxTask(void *parameters)
{
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.type = HELLO;
    //  txPacketCache.header.mac = ? TODO init mac header
    while (true)
    {
        int msgLen = generateHelloMessage((olsrMessage_t *)&txPacketCache.payload);
        txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
        uwbSendPacketBlock(&txPacketCache);
        vTaskDelay(M2T(100));
    }
}

static void uwbHelloRxTask(void *parameters)
{
    systemWaitStart();

    UWB_Packet_t rxPacketCache;

    while (true)
    {
        if (uwbReceivePacketBlock(HELLO, &rxPacketCache))
        {
            processHelloMessage((olsrMessage_t *)(&rxPacketCache));
        }
    }
}

void helloInit()
{
    rxQueue = xQueueCreate(HELLO_RX_QUEUE_SIZE, HELLO_RX_QUEUE_ITEM_SIZE);

    UWB_Message_Listener_t listener;
    listener.type = HELLO;
    listener.rxQueue = rxQueue;
    listener.rxCb = helloRxCallback;
    listener.txCb = helloTxCallback;
    uwbRegisterListener(&listener);

    xTaskCreate(uwbHelloTxTask, ADHOC_DECK_ROUTING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbHelloTxTaskHandle);
    xTaskCreate(uwbHelloRxTask, ADHOC_DECK_ROUTING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbHelloRxTaskHandle);
}

void linkSensing(const olsrMessage_t *helloMsg)
{
  configASSERT(helloMsg->m_messageHeader.m_vTime > 0);
  setIndex_t linkTuple = olsrFindInLinkByAddr(&olsrLinkSet, helloMsg->m_messageHeader.m_originatorAddress);
  olsrTime_t now = xTaskGetTickCount();
  bool created = false, updated = false;
  if (linkTuple == -1)
  {
    DEBUG_PRINT_OLSR_LINK("not found Addr %d in linkSet\n", helloMsg->m_messageHeader.m_originatorAddress);
    olsrLinkTuple_t newLinkTuple;
    newLinkTuple.m_localAddr = myAddress;
    newLinkTuple.m_neighborAddr = helloMsg->m_messageHeader.m_originatorAddress;
    newLinkTuple.m_symTime = now - M2T(1000);
    newLinkTuple.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
    linkTuple = olsrInsertToLinkSet(&olsrLinkSet, &newLinkTuple);
    if (linkTuple == -1)
    {
      DEBUG_PRINT_OLSR_LINK("can not malloc from link set by Function[linkSensing]\n");
      return;
    }
    created = true;
  }
  else
  {
    updated = true;
  }
  olsrLinkSet.setData[linkTuple].data.m_asymTime = now + helloMsg->m_messageHeader.m_vTime;
  olsrHelloMessage_t *helloMsgBody = (olsrHelloMessage_t *)(helloMsg->m_messagePayload);
  for (uint8_t i = 0; i < helloMsgBody->m_helloHeader.m_linkMessageNumber; i++)
  {
    uint8_t lt = helloMsgBody->m_linkMessage[i].m_linkCode & 0x03;
    uint8_t nt = (helloMsgBody->m_linkMessage[i].m_linkCode >> 2) & 0x03;

    if ((lt == OLSR_SYM_LINK && nt == OLSR_NOT_NEIGH) || (nt != OLSR_SYM_NEIGH && nt != OLSR_MPR_NEIGH && nt != OLSR_NOT_NEIGH))
    {
      continue;
    }
    if (helloMsgBody->m_linkMessage[i].m_addresses == myAddress)
    {
      if (lt == OLSR_LOST_LINK)
      {
        DEBUG_PRINT_OLSR_LINK("this addr %d is lost link\n", helloMsgBody->m_linkMessage[i].m_addresses);
        olsrLinkSet.setData[linkTuple].data.m_symTime = now - M2T(1000);
        updated = true;
      }
      else if (lt == OLSR_SYM_LINK || lt == OLSR_ASYM_LINK)
      {
        olsrLinkSet.setData[linkTuple].data.m_symTime = now + helloMsg->m_messageHeader.m_vTime;
        olsrLinkSet.setData[linkTuple].data.m_expirationTime = olsrLinkSet.setData[linkTuple].data.m_symTime + OLSR_NEIGHB_HOLD_TIME;
        updated = true;
      }
      else
      {
        DEBUG_PRINT_OLSR_LINK("BAD LINK");
      }
    }
    else
    {
      DEBUG_PRINT_OLSR_LINK("this %d is not equal to myaddress\n", helloMsgBody->m_linkMessage[i].m_addresses);
    }
  }
  olsrLinkSet.setData[linkTuple].data.m_expirationTime = olsrLinkSet.setData[linkTuple].data.m_asymTime >
                                                                 olsrLinkSet.setData[linkTuple].data.m_expirationTime
                                                             ? olsrLinkSet.setData[linkTuple].data.m_asymTime
                                                             : olsrLinkSet.setData[linkTuple].data.m_expirationTime;
  if (updated)
  {
    linkTupleUpdated(&olsrLinkSet.setData[linkTuple].data, helloMsgBody->m_helloHeader.m_willingness);
    DEBUG_PRINT_OLSR_LINK("update\n");
  }
  if (created)
  {
    linkTupleAdded(&olsrLinkSet.setData[linkTuple].data, helloMsgBody->m_helloHeader.m_willingness); // same addr?
    // del
  }
}

void populateNeighborSet(const olsrMessage_t *helloMsg)
{
  setIndex_t nbTuple = olsrFindNeighborByAddr(&olsrNeighborSet,
                                              helloMsg->m_messageHeader.m_originatorAddress);
  if (nbTuple != -1)
  {
    olsrNeighborSet.setData[nbTuple].data.m_willingness = ((olsrHelloMessageHeader_t *)helloMsg->m_messagePayload)->m_willingness;
    DEBUG_PRINT_OLSR_NEIGHBOR("populate successful\n");
  }
}

void populateTwoHopNeighborSet(const olsrMessage_t *helloMsg)
{
  olsrTime_t now = xTaskGetTickCount();
  olsrAddr_t sender = helloMsg->m_messageHeader.m_originatorAddress;
  setIndex_t linkTuple = olsrFindInLinkByAddr(&olsrLinkSet, sender);
#ifdef USER_ROUTING
  olsrWeight_t n1Weight = distanceToWeight(getDistanceFromAddr(sender));
#endif
  if (linkTuple != -1)
  {
    olsrHelloMessage_t *helloMsgBody = (olsrHelloMessage_t *)helloMsg->m_messagePayload;
    for (int i = 0; i < helloMsgBody->m_helloHeader.m_linkMessageNumber; i++)
    {
      uint8_t nbType = (helloMsgBody->m_linkMessage[i].m_linkCode >> 2) & 0x3;
      olsrAddr_t candidate = helloMsgBody->m_linkMessage[i].m_addresses;
      if (nbType == OLSR_SYM_NEIGH || nbType == OLSR_MPR_NEIGH)
      {
        if (candidate == myAddress)
        {
          continue;
        }
        setIndex_t twoHopNeighborTuple = olsrFindTwoHopNeighborTuple(&olsrTwoHopNeighborSet, sender, candidate);
        if (twoHopNeighborTuple != -1)
        {
          olsrTwoHopNeighborSet.setData[twoHopNeighborTuple].data.m_expirationTime = now +
                                                                                     helloMsg->m_messageHeader.m_vTime;
#ifdef USER_ROUTING
          // olsrTwoHopNeighborSet.setData[twoHopNeighborTuple].data.m_weight=(1-distanceToPacketLoss(getDistanceFromAddr(sender)))*helloMsgBody->m_linkMessage[i].m_weight;
          olsrTwoHopNeighborSet.setData[twoHopNeighborTuple].data.m_weight =
              n1Weight * helloMsgBody->m_linkMessage[i].m_weight;
#endif
#ifdef EFF_BROADCASTING
          olsrTwoHopNeighborSet.setData[twoHopNeighborTuple].data.m_weight =
              helloMsgBody->m_linkMessage[i].m_weight;
#endif
        }
        else
        {
          olsrTwoHopNeighborTuple_t newTuple;
          newTuple.m_neighborAddr = sender;
          newTuple.m_twoHopNeighborAddr = candidate;
          newTuple.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
#ifdef USER_ROUTING
          // newTuple.m_weight=(1-distanceToPacketLoss(getDistanceFromAddr(sender)))*helloMsgBody->m_linkMessage[i].m_weight;
          newTuple.m_weight = n1Weight * helloMsgBody->m_linkMessage[i].m_weight;
#endif
#ifdef EFF_BROADCASTING
          newTuple.m_weight = helloMsgBody->m_linkMessage[i].m_weight;
#endif
          addTwoHopNeighborTuple(&newTuple);
        }
      }
      else if (nbType == OLSR_NOT_NEIGH)
      {
        olsrEraseTwoHopNeighborTuple(&olsrTwoHopNeighborSet, sender, candidate);
      }
      else
      {
        DEBUG_PRINT_OLSR_NEIGHBOR2("bad neighbor type in func [PopulateTwoHopNeighborSet]\n");
      }
    }
  }
  else
  {
    DEBUG_PRINT_OLSR_NEIGHBOR2("can not found link tuple\n");
  }
}

void populateMprSelectorSet(const olsrMessage_t *helloMsg)
{
  olsrTime_t now = xTaskGetTickCount();
  olsrAddr_t sender = helloMsg->m_messageHeader.m_originatorAddress;
  olsrHelloMessage_t *helloBody = (olsrHelloMessage_t *)helloMsg->m_messagePayload;
  for (int i = 0; i < helloBody->m_helloHeader.m_linkMessageNumber; i++)
  {
    uint8_t nt = helloBody->m_linkMessage[i].m_linkCode >> 2;
    if (nt == OLSR_MPR_NEIGH && helloBody->m_linkMessage[i].m_addresses == myAddress)
    {
      setIndex_t candidate = olsrFindInMprSelectorSet(&olsrMprSelectorSet, sender);
      if (candidate == -1)
      {
        olsrMprSelectorTuple_t new;
        new.m_addr = sender;
        new.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
        addMprSelectorTuple(&new);
      }
      else
      {
        olsrMprSelectorSet.setData[candidate].data.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
      }
    }
  }
}

void mprCompute()
{
  olsrMprSetInit(&olsrMprSet);
  olsrNeighborSet_t N;
  olsrNeighborSetInit(&N);

  setIndex_t itForOlsrNeighborSet = olsrNeighborSet.fullQueueEntry;
  while (itForOlsrNeighborSet != -1)
  {
    if (olsrNeighborSet.setData[itForOlsrNeighborSet].data.m_status == STATUS_SYM)
    {
      olsrInsertToNeighborSet(&N, &olsrNeighborSet.setData[itForOlsrNeighborSet].data);
    }
    itForOlsrNeighborSet = olsrNeighborSet.setData[itForOlsrNeighborSet].next;
  }

  olsrTwoHopNeighborSet_t N2;
  olsrTwoHopNeighborSetInit(&N2);

  setIndex_t itForTwoHopNeighborSet = olsrTwoHopNeighborSet.fullQueueEntry;
  while (itForTwoHopNeighborSet != -1)
  {
    olsrTwoHopNeighborSetItem_t twoHopNTuple = olsrTwoHopNeighborSet.setData[itForTwoHopNeighborSet];
    // two hop neighbor can not equal to myself
    // TODO bianli fangshi check
    if (twoHopNTuple.data.m_twoHopNeighborAddr == myAddress)
    {
      itForTwoHopNeighborSet = twoHopNTuple.next;
      continue;
    }

    bool ok = false;
    setIndex_t itForN = N.fullQueueEntry;
    while (itForN != -1)
    {
      if (N.setData[itForN].data.m_neighborAddr == twoHopNTuple.data.m_neighborAddr)
      {
        // N.setData[itForN].data.m_willingness == WILL_NEVER? ok=false: ok=true;
        ok = true;
        break;
      }
      itForN = N.setData[itForN].next;
    }
    if (!ok)
    {
      itForTwoHopNeighborSet = twoHopNTuple.next;
      DEBUG_PRINT_OLSR_MPR("464 continue\n");
      continue;
    }
    // this two hop neighbor can not be one hop neighbor
    itForN = N.fullQueueEntry;
    while (itForN != -1)
    {
      if (N.setData[itForN].data.m_neighborAddr == twoHopNTuple.data.m_twoHopNeighborAddr) // A-B A-C B-C
      {
        ok = false;
        break;
      }
      itForN = N.setData[itForN].next;
    }

    if (ok)
    {
      olsrInsertToTwoHopNeighborSet(&N2, &twoHopNTuple.data);
    }

    itForTwoHopNeighborSet = twoHopNTuple.next;
  }
  uint16_t N1_len = 0;
  setIndex_t itN1 = N.fullQueueEntry;
  while (itN1 != -1)
  {
    N1_len++;
    itN1 = N.setData[itN1].next;
  }
  uint16_t N2_len = 0;
  setIndex_t itN2 = N2.fullQueueEntry;
  while (itN2 != -1)
  {
    N2_len++;
    itN2 = N2.setData[itN2].next;
  }
  if (N1_len == 0 || N2_len == 0)
  {
    return;
  }
  olsrAddr_t n2map[N2_len];
  for (int i = 0; i < N2_len; ++i)
  {
    n2map[i] = 1000;
  }
  // get true n2_len
  N2_len = 0;
  itN2 = N2.fullQueueEntry;
  while (itN2 != -1)
  {
    if (find(n2map, N2.setData[itN2].data.m_twoHopNeighborAddr, N2_len))
    {
      itN2 = N2.setData[itN2].next;
      continue;
    }
    else
    {
      n2map[N2_len++] = N2.setData[itN2].data.m_twoHopNeighborAddr;
    }
    itN2 = N2.setData[itN2].next;
  }
  // 2 d array
  contriNode contriMatrix[N2_len][N1_len];
  float lb[N2_len];
  int32_t lb_log[N2_len];
  for (int i = 0; i < N2_len; i++)
  {
    lb[i] = 1.0;
  }
  itN1 = N.fullQueueEntry;
  itN2 = N2.fullQueueEntry;
  int16_t idx_i = 0;
  int16_t idx_j = 0;
  olsrWeight_t gain = 1.5;
  float base = 0.999;
  while (itN1 != -1)
  {
    olsrWeight_t loss1 = N.setData[itN1].data.m_weight;
    for (idx_j = 0; idx_j < N2_len; idx_j++)
    {
      olsrWeight_t loss2 = 1.0;
      itN2 = N2.fullQueueEntry;
      while (itN2 != -1)
      {
        if (N2.setData[itN2].data.m_twoHopNeighborAddr == n2map[idx_j] && N2.setData[itN2].data.m_neighborAddr == N.setData[itN1].data.m_neighborAddr)
        {
          loss2 = N2.setData[itN2].data.m_weight;
          break;
        }
        itN2 = N2.setData[itN2].next;
      }
      float temp = loss1 + (1 - loss1) * loss2;
      lb[idx_j] *= temp;
      contriMatrix[idx_j][idx_i].val = log_base(base, temp);
      contriMatrix[idx_j][idx_i].addr = N.setData[itN1].data.m_neighborAddr;
      DEBUG_PRINT_OLSR_MPR("cmatrix[%d][%d]={%d,%d},", idx_j, idx_i, contriMatrix[idx_j][idx_i].val, contriMatrix[idx_j][idx_i].addr);
    }
    DEBUG_PRINT_OLSR_MPR("\n");
    idx_i++;
    itN1 = N.setData[itN1].next;
  }
  // calcu lb_log
  for (int i = 0; i < N2_len; ++i)
  {
    lb_log[i] = log_base(base, MIN(lb[i] * gain, 0.999));
  }
  // sort
  int32_t cur_sum[N2_len];
  for (int i = 0; i < N2_len; ++i)
  {
    cur_sum[i] = 0;
  }
  contriNode oriContriMatrix[N2_len][N1_len];
  olsrAddr_t map[N1_len];
  for (int i = 0; i < N2_len; i++)
  {
    for (int j = 0; j < N1_len; j++)
    {
      oriContriMatrix[i][j] = contriMatrix[i][j];
      map[j] = contriMatrix[i][j].addr;
    }
  }
  for (int i = 0; i < N2_len; ++i)
  {
    sort(&contriMatrix[i][0], 0, N1_len - 1);
  }
  DEBUG_PRINT_OLSR_MPR("After sorted\n");
  for (int i = 0; i < N2_len; ++i)
  {
    for (int j = 0; j < N1_len; ++j)
    {
      DEBUG_PRINT_OLSR_MPR("cmatrix[%d][%d]={%d,%d},", i, j, contriMatrix[i][j].val, contriMatrix[i][j].addr);
    }
    DEBUG_PRINT_OLSR_MPR("\n");
  }

  // represent the selected mpr
  olsrAddr_t sn1[N1_len];
  int sn1_len = 0;
  for (int i = 0; i < N1_len; ++i)
  {
    sn1[i] = 1000;
  }
  bool flag = false;

  for (int i = 0; i < N1_len; ++i)
  {
    if (flag)
      break;
    for (int j = 0; j < N2_len; ++j)
    {
      if (!find(sn1, contriMatrix[j][i].addr, sn1_len))
      {
        sn1[sn1_len++] = contriMatrix[j][i].addr;
        // update curSum
        int index = findIndex(map, contriMatrix[j][i].addr, N1_len);
        int count = 0;
        for (int k = 0; k < N2_len; ++k)
        {
          cur_sum[k] += oriContriMatrix[k][index].val;
          if (cur_sum[k] >= lb_log[k])
            count++;
        }
        if (count == N2_len)
        {
          flag = true;
          break;
        }
      }
      else
      {
        continue;
      }
    }
  }
  if (sn1[0] == 1000)
  { // can not find the solution,selected all,this can be remove for sn1 selected all already(maybe)
    for (int i = 0; i < N1_len; i++)
    {
      sn1[i] = map[i];
    }
  }
  for (int i = 0; i < N1_len; ++i)
  {
    if (sn1[i] != 1000)
    {
      // insert into mpr
      olsrMprTuple_t tmp;
      tmp.m_addr = sn1[i];
      olsrInsertToMprSet(&olsrMprSet, &tmp);
      DEBUG_PRINT_OLSR_MPR("Mpr addr %d\n", tmp.m_addr);
    }
  }
}

void olsrPrintAll()
{
  olsrPrintLinkSet(&olsrLinkSet);
  olsrPrintNeighborSet(&olsrNeighborSet);
  olsrPrintTwoHopNeighborSet(&olsrTwoHopNeighborSet);
  olsrPrintMprSet(&olsrMprSet);
  olsrPrintTopologySet(&olsrTopologySet);
  olsrPrintMprSelectorSet(&olsrMprSelectorSet);
  olsrPrintRoutingSet(&olsrRoutingSet);
  olsrPrintNsSet(&olsrNsSet);
  // olsrPrintDuplicateSet(&olsrDuplicateSet);
}
