#include "routingAlgo.h"
#include "routingStruct.h"
#include "routingPacket.h"
#include "floodingStruct.h"
#include "rangingProtocolInit.h"
#include "rangingProtocolPacket.h"
#include "rangingProtocolDebug.h"

static int rMessageSeq = 0;

static void ComputeRoutingTable(uint16_t originatorAddress, uint16_t destinationAddress)
{
    if(destinationAddress == myAddress)
    {
        return;
    }
    // 插入或更新路由表
    if (originatorAddress == myAddress)
    {
        index_t index = RoutingTableFind(&routingTable, destinationAddress);
        // 更新
        if (index != -1)
        {
            RoutingTableUpdate(&routingTable, index, destinationAddress);
        }
        // 插入
        else
        {
            RoutingTableInsert(&routingTable, destinationAddress, destinationAddress);
        }
    }
    else
    {
        index_t index = RoutingTableFind(&routingTable, destinationAddress);
        // 更新
        if (index != -1)
        {
            uint16_t nextAddress = RoutingTableFindRoute(&routingTable, originatorAddress);
            RoutingTableUpdate(&routingTable, index, nextAddress);
        }
        // 插入
        else
        {
            uint16_t nextAddress = RoutingTableFindRoute(&routingTable, originatorAddress);
            RoutingTableInsert(&routingTable, destinationAddress, nextAddress);
        }
    }
}

void Routing()
{
    // 错误机制
    if (fTopologyTable.fTopologyTableSize == 0)
    {
        return;
    }
    // 定义数据
    nodeStateTable_t nodeStateTable;
    NodeStateTableInit(&nodeStateTable);
    nodeStateTableItem_t item;
    index_t itemIndex;
    // 插入自己这个节点
    NodeStateTableInsert(&nodeStateTable, myAddress, 0);
    // 定义优先队列
    priorityQueue_t priorityQueue;
    priorityQueueInit(&priorityQueue);
    // 压入自己这个节点
    PriorityQueuePush(&priorityQueue, &nodeStateTable.itemSet[0]);
    
    while (!isPriorityQueueEmpty(&priorityQueue))
    {
        // 从优先队列中弹出距离最短的节点
        PriorityQueueFindFront(&priorityQueue, &item);
        PriorityQueuePopFront(&priorityQueue);
        // 检查是否已访问
        if (item.visited)
        {
            continue;
        }
        else
        {
            itemIndex = FindInNodeStateTable(&nodeStateTable, item.address);
            nodeStateTable.itemSet[itemIndex].visited = true;
        }
        
        index_t i = fTopologyTable.fTopologyTableFullEntry;
        while (i != -1)
        {
            if (fTopologyTable.fTopologyTableItemSet[i].fTopologyTableTuple.originatorAddress == nodeStateTable.itemSet[itemIndex].address)
            {
                uint16_t originatorAddress = fTopologyTable.fTopologyTableItemSet[i].fTopologyTableTuple.originatorAddress;
                uint16_t edgeDistance = fTopologyTable.fTopologyTableItemSet[i].fTopologyTableTuple.distance;
                uint16_t destinationAddress = fTopologyTable.fTopologyTableItemSet[i].fTopologyTableTuple.destinationAddress;
                index_t destinationIndex = FindInNodeStateTable(&nodeStateTable, destinationAddress);
                if (destinationIndex != -1)
                {
                    if (nodeStateTable.itemSet[destinationIndex].distance > nodeStateTable.itemSet[itemIndex].distance + edgeDistance)
                    {
                        nodeStateTable.itemSet[destinationIndex].distance = nodeStateTable.itemSet[itemIndex].distance + edgeDistance;
                        PriorityQueuePush(&priorityQueue, &nodeStateTable.itemSet[destinationIndex]);
                        //
                        ComputeRoutingTable(originatorAddress, destinationAddress);
                    }
                }
                else
                {
                    index_t destIndex = NodeStateTableInsert(&nodeStateTable, destinationAddress, nodeStateTable.itemSet[itemIndex].distance + edgeDistance);
                    PriorityQueuePush(&priorityQueue, &nodeStateTable.itemSet[destIndex]);
                    //
                    ComputeRoutingTable(originatorAddress, destinationAddress);
                }
            }
            i = fTopologyTable.fTopologyTableItemSet[i].next;
        }
    }
    // DEBUG_PRINT_ALGO_DATA("ROUT SIZE: %u\n", routingTable.size);
    // DEBUG_PRINT_ALGO_DATA("NODE SIZE: %u\n", nodeStateTable.size);
}

bool GenerateR(packet_t* txRPacket, uint16_t destinationAddress, uint8_t timeToLive)
{
    // 路由消息报文头生成
    message_t* message = (message_t*) txRPacket->payload;
    rMessageHeader_t* rMessageHeader = (rMessageHeader_t*) message->messagePayload;
    rMessageHeader->originatorAddress = myAddress;
    rMessageHeader->destinationAddress = destinationAddress;
    rMessageHeader->nextAddress = RoutingTableFindRoute(&routingTable, destinationAddress);
    rMessageHeader->sequence = rMessageSeq;
    rMessageHeader->size = sizeof(rMessageHeader_t);
    rMessageHeader->timeToLive = timeToLive;

    // 错误机制
    if (rMessageHeader->nextAddress == INF)
    {
        return false;
    }
    //DEBUG_PRINT_ALGO_DATA("ORIG: %u, DIST: %u, NEXT: %u\n", rMessageHeader->originatorAddress, rMessageHeader->destinationAddress, rMessageHeader->nextAddress);
    // TODO: 报文负载

    // 数据更新
    rMessageSeq++;
    message->messageHeader.messageLength = sizeof(messageHeader_t) + rMessageHeader->size;

    return true;
}

static void ForwardR(rMessageHeader_t* rMessageHeader)
{
    rMessageHeader->nextAddress = RoutingTableFindRoute(&routingTable, rMessageHeader->destinationAddress);
}

bool DispatchR(message_t* message)
{
    rMessageHeader_t* rMessageHeader = (rMessageHeader_t *) message->messagePayload;
    DEBUG_PRINT_ALGO_DATA("ORIG: %u, DIST: %u, NEXT: %u\n", rMessageHeader->originatorAddress, rMessageHeader->destinationAddress, rMessageHeader->nextAddress);
    enum ROUTING_TYPE type;
    // 首先检查下一跳是否是自己
    if (rMessageHeader->nextAddress == myAddress)
    {
        type = ROUTING_TO_NEXT;
    }
    // 检查目标地址是否是自己。下一跳地址是自己，目标也可能是自己，那么type就应该为ROUTING_TO_LOCAL
    if (rMessageHeader->destinationAddress == myAddress)
    {
        type = ROUTING_TO_LOCAL;
    }
    // 如果两种情况都不符合，则该路由包不属于自己的管辖范畴
    if (rMessageHeader->nextAddress != myAddress && rMessageHeader->destinationAddress != myAddress)
    {
        type = ROUTING_TO_OTHER;
    }

    switch (type)
    {
        case ROUTING_TO_LOCAL:
        // TODO: 路由信息处理
            DEBUG_PRINT_ALGO("ROUTING HANDLE\n");
            return false;
        case ROUTING_TO_NEXT:
        // TODO: 路由转发处理
            ForwardR(rMessageHeader);
            DEBUG_PRINT_ALGO("ROUTING FORWARD\n");
            return true;
        case ROUTING_TO_OTHER:
        // 不作任何处理
            DEBUG_PRINT_ALGO("ROUTING NOTHING\n");
            return false;
        default:
            return false;
    }
}