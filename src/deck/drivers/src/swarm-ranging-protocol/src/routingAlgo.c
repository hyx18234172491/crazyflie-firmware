#include "routingAlgo.h"
#include "routingStruct.h"
#include "routingPacket.h"
#include "floodingStruct.h"
#include "rangingProtocolInit.h"
#include "rangingProtocolPacket.h"
#include "rangingProtocolDebug.h"

static int dMessageSeq = 0;

void Routing()
{
    // 错误机制
    if (fTopologyTable.fTopologyTableSize == 0)
    {
        return;
    }
    // 清除路由树上一次记录
    RoutingShortestTreeClear(routingShortestTree);
    // 定义节点状态表
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
                        // 构建临时路由最短树
                        routingShortestTree[destinationAddress] = originatorAddress;
                    }
                }
                else
                {
                    index_t destIndex = NodeStateTableInsert(&nodeStateTable, destinationAddress, nodeStateTable.itemSet[itemIndex].distance + edgeDistance);
                    PriorityQueuePush(&priorityQueue, &nodeStateTable.itemSet[destIndex]);
                    // 构建临时路由最短树
                    routingShortestTree[destinationAddress] = originatorAddress;
                }
            }
            i = fTopologyTable.fTopologyTableItemSet[i].next;
        }
    }

    // DEBUG_PRINT_ALGO_DATA("ROUT SIZE: %u\n", routingTable.size);
    DEBUG_PRINT_ALGO_DATA("NODE SIZE: %u\n", nodeStateTable.size);
}

bool GenerateR(packet_t* txRPacket, uint16_t destinationAddress, uint8_t timeToLive)
{
    // 路由消息报文头生成
    message_t* message = (message_t*) txRPacket->payload;
    dMessageHeader_t* dMessageHeader = (dMessageHeader_t*) message->messagePayload;
    dMessageHeader->originatorAddress = myAddress;
    dMessageHeader->destinationAddress = destinationAddress;
    dMessageHeader->nextAddress = RoutingShortestTreeFindRoute(routingShortestTree, destinationAddress);
    dMessageHeader->sequence = dMessageSeq;
    dMessageHeader->size = sizeof(dMessageHeader_t);
    dMessageHeader->timeToLive = timeToLive;

    // 错误机制
    if (dMessageHeader->nextAddress == INF)
    {
        return false;
    }
    //DEBUG_PRINT_ALGO_DATA("ORIG: %u, DIST: %u, NEXT: %u\n", dMessageHeader->originatorAddress, dMessageHeader->destinationAddress, dMessageHeader->nextAddress);
    // TODO: 报文负载

    // 数据更新
    dMessageSeq++;
    message->messageHeader.messageLength = sizeof(messageHeader_t) + dMessageHeader->size;

    return true;
}

static void ForwardR(dMessageHeader_t* dMessageHeader)
{
    dMessageHeader->nextAddress = RoutingShortestTreeFindRoute(routingShortestTree, dMessageHeader->destinationAddress);
}

bool DispatchR(message_t* message)
{
    dMessageHeader_t* dMessageHeader = (dMessageHeader_t *) message->messagePayload;
    DEBUG_PRINT_ALGO_DATA("ORIG: %u, DIST: %u, NEXT: %u\n", dMessageHeader->originatorAddress, dMessageHeader->destinationAddress, dMessageHeader->nextAddress);
    enum ROUTING_TYPE type;
    // 首先检查下一跳是否是自己
    if (dMessageHeader->nextAddress == myAddress)
    {
        type = ROUTING_TO_NEXT;
    }
    // 检查目标地址是否是自己。下一跳地址是自己，目标也可能是自己，那么type就应该为ROUTING_TO_LOCAL
    if (dMessageHeader->destinationAddress == myAddress)
    {
        type = ROUTING_TO_LOCAL;
    }
    // 如果两种情况都不符合，则该路由包不属于自己的管辖范畴
    if (dMessageHeader->nextAddress != myAddress && dMessageHeader->destinationAddress != myAddress)
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
            ForwardR(dMessageHeader);
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