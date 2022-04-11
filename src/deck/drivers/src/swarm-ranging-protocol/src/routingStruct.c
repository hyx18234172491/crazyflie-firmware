#include <stdlib.h>
#include <string.h>

#include "adhocdeck.h"

#include "routingStruct.h"
#include "rangingProtocolDebug.h"
#include "rangingProtocolInit.h"

// 优先队列
// 初始化
void priorityQueueInit(priorityQueue_t* priorityQueue)
{
    memset(priorityQueue, 0, sizeof(priorityQueue_t));
    // 初始化优先队列数组成员
    priorityQueue->maxSize = NODE_MAX_SIZE;
    priorityQueue->size = 0;
}   

// 判断是否为空
bool isPriorityQueueEmpty(priorityQueue_t* priorityQueue)
{
    if (priorityQueue == NULL) return true;
    if (priorityQueue->size == 0) return true;
    else return false;
}

// 判断是否为满
bool isPriorityQueueFull(priorityQueue_t* priorityQueue)
{
    if (priorityQueue == NULL) return false;
    if (priorityQueue->size == priorityQueue->maxSize) return true;
    else return false;
}

// 压入
bool PriorityQueuePush(priorityQueue_t* priorityQueue, nodeStateTableItem_t* item)
{
    // 判断是否为满
    if (isPriorityQueueFull(priorityQueue)) return false;
    // 小顶堆排序插入
    int i;
    for (i = priorityQueue->size + 1; item->distance < priorityQueue->itemSet[i/2].distance && i > 1; i /= 2)
    {
        priorityQueue->itemSet[i] = priorityQueue->itemSet[i/2];
    }
    memcpy(&priorityQueue->itemSet[i], item, sizeof(nodeStateTableItem_t));
    priorityQueue->size++;

    return true;
}

// 查询最前面的值,即最小值
bool PriorityQueueFindFront(priorityQueue_t* priorityQueue, nodeStateTableItem_t* item)
{
    // 检查是否为空
    if (isPriorityQueueEmpty(priorityQueue)) return false;
    // 赋值
    memcpy(item, &priorityQueue->itemSet[1], sizeof(nodeStateTableItem_t));

    return true;
}

// 弹出最前的值,即最小值
bool PriorityQueuePopFront(priorityQueue_t* priorityQueue)
{
    // 判断是否为空
    if (isPriorityQueueEmpty(priorityQueue)) return false;
    // 
    nodeStateTableItem_t* lastItem = &priorityQueue->itemSet[priorityQueue->size];
    // nodeStateTableItem_t* minItem = &priorityQueue->itemSet[1];
    priorityQueue->size--;
    // 如果只有一个元素,删除后就清空优先队列数组
    if (priorityQueue->size == 0)
    {
        memset(priorityQueue->itemSet, 0, (priorityQueue->maxSize) * sizeof(nodeStateTableItem_t));
        return true;
    }
    // 小顶堆排序
    int i, minChild = 0;
    for (i = 1; i * 2 <= priorityQueue->size; i = minChild)
    {
        minChild = i * 2;
        // 选择子中较小的子
        if (priorityQueue->itemSet[minChild].distance > priorityQueue->itemSet[minChild + 1].distance && minChild != priorityQueue->size)
        {
            minChild += 1;
        }
        // 如果lastItem更大,则还需往下检索排序
        if (lastItem->distance > priorityQueue->itemSet[minChild].distance)
        {
            memcpy(&priorityQueue->itemSet[i], &priorityQueue->itemSet[minChild], sizeof(nodeStateTableItem_t));
        }
        else
        {
            break;
        }
    }
    memcpy(&priorityQueue->itemSet[i], lastItem, sizeof(nodeStateTableItem_t));

    return true;
}

// 节点状态表
// 初始化
void NodeStateTableInit(nodeStateTable_t* nodeStateTable)
{
    // 初始化
    index_t i;
    for (i = 0; i < NODE_MAX_SIZE - 1; i++)
    {
        nodeStateTable->itemSet[i].distance = INF;
        nodeStateTable->itemSet[i].next = i + 1;
        nodeStateTable->itemSet[i].visited = false;
    }
    nodeStateTable->itemSet[i].distance = INF;
    nodeStateTable->itemSet[i].next = -1;
    nodeStateTable->itemSet[i].visited = false;

    nodeStateTable->freeEntry = 0;
    nodeStateTable->fullEntry = -1;
    nodeStateTable->size = 0;
}

// 插入节点状态
index_t NodeStateTableInsert(nodeStateTable_t* nodeStateTable, uint16_t address, uint16_t distance)
{
    if (nodeStateTable->freeEntry == -1)
    {
        // 插入失败
        return -1;
    }
    else
    {
        index_t candidate = nodeStateTable->freeEntry;
        nodeStateTable->freeEntry = nodeStateTable->itemSet[candidate].next;
        index_t tmp = nodeStateTable->fullEntry;
        nodeStateTable->fullEntry = candidate;
        nodeStateTable->itemSet[candidate].next = tmp;

        nodeStateTable->itemSet[candidate].address = address;
        nodeStateTable->itemSet[candidate].distance = distance;

        nodeStateTable->size++;

        return candidate;
    }
}

// 查询节点状态
index_t FindInNodeStateTable(nodeStateTable_t* nodeStateTable, uint16_t address)
{
    index_t candidate = nodeStateTable->fullEntry;
    while (candidate != -1)
    {
        if (nodeStateTable->itemSet[candidate].address == address)
        {
            break;
        }
        candidate = nodeStateTable->itemSet[candidate].next;
    }

    return candidate;
}

// 路由表
void RoutingTableInit(routingTable_t *routingTable)
{
    index_t i;
    for (i = 0; i < NODE_MAX_SIZE - 1; i++)
    {
        routingTable->itemSet[i].next = i + 1;
    }
    routingTable->itemSet[i].next = -1;
    routingTable->freeEntry = 0;
    routingTable->fullEntry = -1;
    routingTable->size = 0;
}

static index_t RoutingTableMalloc(routingTable_t *routingTable)
{
    if (routingTable->freeEntry == -1)
    {
        return -1;
    }
    else
    {
        index_t candidate = routingTable->freeEntry;
        routingTable->freeEntry = routingTable->itemSet[candidate].next;
        index_t tmp = routingTable->fullEntry;
        routingTable->fullEntry = candidate;
        routingTable->itemSet[candidate].next = tmp;
        return candidate;
    }
}

index_t RoutingTableInsert(routingTable_t *routingTable, uint16_t destinationAddress, uint16_t nextAddress)
{
    index_t candidate = RoutingTableMalloc(routingTable);
    // 路由表有剩余空间
    if(candidate != -1)
    {
        routingTable->itemSet[candidate].destinationAddress = destinationAddress;
        routingTable->itemSet[candidate].nextAddress = nextAddress;

        routingTable->size++;
    }

    return candidate;
}

void RoutingTableUpdate(routingTable_t *routingTable, index_t index, uint16_t nextAddress)
{
    if(routingTable->itemSet[index].nextAddress != nextAddress)
    {
        routingTable->itemSet[index].nextAddress = nextAddress;
    }
}

index_t RoutingTableFind(routingTable_t *routingTable, uint16_t destinationAddress)
{
    index_t candidate = routingTable->fullEntry;
    while(candidate != -1)
    {
        routingTableItem_t item = routingTable->itemSet[candidate];
        if(item.destinationAddress == destinationAddress)
        {
            break;
        }
        candidate = item.next;
    }

    return candidate;
}

uint16_t RoutingTableFindRoute(routingTable_t *routingTable, uint16_t destinationAddress)
{
    index_t candidate = routingTable->fullEntry;
    while(candidate != -1)
    {
        routingTableItem_t item = routingTable->itemSet[candidate];
        if(item.destinationAddress == destinationAddress)
        {
            break;
        }
        candidate = item.next;
    }

    if (candidate == -1)
    {
        return INF;
    }
    else
    {
        return routingTable->itemSet[candidate].nextAddress;
    }
}