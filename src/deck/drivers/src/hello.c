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

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static TaskHandle_t uwbHelloTxTaskHandle = 0;
static TaskHandle_t uwbHelloRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;

void helloRxCallback(void *parameters)
{
    DEBUG_PRINT("helloRxCallback \n");
}

void helloTxCallback(void *parameters)
{
    DEBUG_PRINT("helloTxCallback \n");
}

void processHello(UWB_Packet_t *packet)
{
    for (int i = 0; i < m_Nei_Table.one_nei_number; i++)
    {
        if (message->header.srcAddress == m_Nei_Table.one_nei_address[i].address)
        {
            m_Nei_Table.one_nei_address[i].mtime = mtime;
            uint8_t two_nei_number = (message->header.msgLength - 20) / sizeof(Body_Unit_t);
            m_Nei_Table.one_nei_address[i].two_nei_number = two_nei_number;
            for (int j = 0; j < two_nei_number; j++)
            {
                m_Nei_Table.one_nei_address[i].two_nei_address[j] = message->bodyUnits[j].address;
                m_Nei_Table.one_nei_address[i].two_nei_distance[j] = message->bodyUnits[j].distance;
            }
        }
        // else{}
    }
}

int generateRoutingDataMessage(MockData_t *message)
{
    int msgLen = sizeof(MockData_t);
    message->seqNumber = seqNumber++;
    return msgLen;
}

static void processHelloDataMessage(UWB_Packet_t *packet)
{
    MockData_t *mockData = (MockData_t *)packet->payload;
    DEBUG_PRINT("received routing data, seq number = %d \n", mockData->seqNumber);
}

static void uwbHelloTxTask(void *parameters)
{
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.type = DATA;
    //  txPacketCache.header.mac = ? TODO init mac header
    while (true)
    {
        int msgLen = generateRoutingDataMessage((MockData_t *)&txPacketCache.payload);
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
            processRoutingDataMessage(&rxPacketCache);
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

    xTaskCreate(uwbRoutingTxTask, ADHOC_DECK_ROUTING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbHelloTxTaskHandle);
    xTaskCreate(uwbRoutingRxTask, ADHOC_DECK_ROUTING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbHelloRxTaskHandle);
}

static void helloProcess(Ranging_Message_t *message, dwTime_t mtime)
{
    for (int i = 0; i < m_Nei_Table.one_nei_number; i++)
    {
        if (message->header.srcAddress == m_Nei_Table.one_nei_address[i].address)
        {
            m_Nei_Table.one_nei_address[i].mtime = mtime;
            uint8_t two_nei_number = (message->header.msgLength - 20) / sizeof(Body_Unit_t);
            m_Nei_Table.one_nei_address[i].two_nei_number = two_nei_number;
            for (int j = 0; j < two_nei_number; j++)
            {
                m_Nei_Table.one_nei_address[i].two_nei_address[j] = message->bodyUnits[j].address;
                m_Nei_Table.one_nei_address[i].two_nei_distance[j] = message->bodyUnits[j].distance;
            }
        }
        // else{}
    }
}

void updateTask()
{
    for (int i = 0; i < m_Nei_Table.one_nei_number; i++)
    {
        if (xTaskGetTickCount() - m_Nei_Table.one_nei_address[i].mtime.full > M2T(MAX_SURVIVE_TIME))
        {
            for (int j = i; j < m_Nei_Table.one_nei_number; j++)
            {
                m_Nei_Table.one_nei_address[j] = m_Nei_Table.one_nei_address[j + 1];
                m_Nei_Table.one_nei_number--;
            }
            i--;
        }
    }
}
