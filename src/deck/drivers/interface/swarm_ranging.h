#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_
#include "adhocdeck.h"
#include "ranging_struct.h"

/* Function Switch */
#define ENABLE_BUS_BOARDING_SCHEME

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 20
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20  // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
#define TX_PERIOD_IN_MS 30
/*---自己添加---start---*/
typedef struct
{
    uint16_t distanceTowards[RANGING_TABLE_SIZE + 1]; // cm
    short velocityXInWorld[RANGING_TABLE_SIZE + 1];   // 2byte m/s 在世界坐标系下的速度（不是机体坐标系）
    short velocityYInWorld[RANGING_TABLE_SIZE + 1];   // 2byte cm/s 在世界坐标系下的速度（不是机体坐标系）
    float gyroZ[RANGING_TABLE_SIZE + 1];
    uint16_t positionZ[RANGING_TABLE_SIZE + 1]; // cm
    bool refresh[RANGING_TABLE_SIZE + 1];
    bool isNewAdd[RANGING_TABLE_SIZE + 1]; // 这个邻居是否是新加入的
} neighborStateInfo_t;

typedef struct
{
    address_t address[RANGING_TABLE_SIZE + 1];
    int size;
} currentNeighborAddressInfo_t; /*当前正在和本无人机进行通信的邻居地址信息*/
/*---自己添加---end---*/

/* Ranging Operations */
void rangingInit();
int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
int generateRangingMessage(Ranging_Message_t *rangingMessage);
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);
/*---自己添加---start---*/
void setNeighborStateInfo(uint16_t neighborAddress, int16_t distance, Ranging_Message_Header_t *rangingMessageHeader, bool isNewAddNeighbor);
bool getNeighborStateInfo(uint16_t neighborAddress, uint16_t *distance, short *vx, short *vy, float *gyroZ, uint16_t *height, bool *isNewAddNeighbor);
bool getOrSetKeepflying(uint16_t RobIDfromControl, bool keep_flying);
void getCurrentNeighborAddressInfo_t(currentNeighborAddressInfo_t *currentNeighborAddressInfo); /*供外部调用，获取当前无人机的所有正在通信的邻居的地址*/
/*---自己添加---end---*/

#endif