#ifndef __ROUTINGSTRUCT_H__
#define __ROUTINGSTRUCT_H__

#include "mac.h"

#include "floodingStruct.h"

#define NODE_MAX_SIZE 9
#define INF 34463

// 节点状态表
typedef struct {
    uint16_t address;
    bool visited;
    uint16_t distance;
    index_t next;
} __attribute__((packed)) nodeStateTableItem_t;

typedef struct {
    nodeStateTableItem_t itemSet[NODE_MAX_SIZE];
    index_t freeEntry;
    index_t fullEntry;
    uint8_t size;
} __attribute__((packed)) nodeStateTable_t;

void NodeStateTableInit(nodeStateTable_t* nodeStateTable);
index_t NodeStateTableInsert(nodeStateTable_t* nodeStateTable, uint16_t address, uint16_t distance);
index_t FindInNodeStateTable(nodeStateTable_t* nodeStateTable, uint16_t address);

// 优先队列
typedef struct {
    uint8_t size; // 优先队列目前容量大小
    uint8_t maxSize; // 优先队列最大大小
    nodeStateTableItem_t itemSet[NODE_MAX_SIZE]; // 优先队列数组
} __attribute__((packed)) priorityQueue_t;

void priorityQueueInit(priorityQueue_t* priorityQueue);
bool isPriorityQueueEmpty(priorityQueue_t* priorityQueue);
bool isPriorityQueueFull(priorityQueue_t* priorityQueue);
bool PriorityQueuePush(priorityQueue_t* priorityQueue, nodeStateTableItem_t* item);
bool PriorityQueueFindFront(priorityQueue_t* priorityQueue, nodeStateTableItem_t* item);
bool PriorityQueuePopFront(priorityQueue_t* priorityQueue);

// 路由表
typedef struct {
    uint16_t destinationAddress;
    uint16_t nextAddress;
    index_t next;
} __attribute__((packed)) routingTableItem_t;

typedef struct {
    routingTableItem_t itemSet[NODE_MAX_SIZE];
    index_t freeEntry;
    index_t fullEntry;
    uint8_t size;
} __attribute__((packed)) routingTable_t;

routingTable_t routingTable;

void RoutingTableInit(routingTable_t *routingTable);
index_t RoutingTableInsert(routingTable_t *routingTable, uint16_t destinationAddress, uint16_t nextAddress);
void RoutingTableUpdate(routingTable_t *routingTable, index_t index, uint16_t nextAddress);
index_t RoutingTableFind(routingTable_t *routingTable, uint16_t destinationAddress);
uint16_t RoutingTableFindRoute(routingTable_t *routingTable, uint16_t destinationAddress);

#endif /* __ROUTINGSTRUCT_H__ */