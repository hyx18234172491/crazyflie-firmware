#ifndef HELLO_STRUCT
#define HELLO_STRUCT
#include <stdbool.h>

#include "FreeRTOS.h"
#include "dwTypes.h"
#include "adhocdeck.h"
#include "stdint.h"
#include "semphr.h"
#include <queue.h>
#include <string.h>
#include "setStruct.h"
#include "mprMessage.h"

#define HELLO_RX_QUEUE_SIZE 10
#define HELLO_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)
#define HELLO_INTERVAL 500
#define MAX_SURVIVE_TIME 1000

typedef struct
{
  uint16_t address;
  dwTime_t mtime;
  uint8_t emptyOrNot;
  uint16_t two_nei_address[Max_Nei_Number];
  uint16_t two_nei_distance[Max_Nei_Number];
  uint8_t two_nei_number;
  /* data */
}Nei_Table_unit_t;

typedef struct
{
  uint8_t one_nei_number;
  Nei_Table_unit_t one_nei_address[Max_Nei_Number];
  /* data */
}Nei_Table_t;
Nei_Table_t m_Nei_Table;



static void processHelloMessage(olsrMessage_t *helloMsg);
static void uwbHelloTxTask(void *parameters);
static void uwbHelloRxTask(void *parameters);
void linkSensing(const olsrMessage_t *helloMsg);
void populateNeighborSet(const olsrMessage_t *helloMsg);
void populateTwoHopNeighborSet(const olsrMessage_t *helloMsg);
void populateMprSelectorSet(const olsrMessage_t *helloMsg);
void mprCompute();
void olsrPrintAll();
#endif 