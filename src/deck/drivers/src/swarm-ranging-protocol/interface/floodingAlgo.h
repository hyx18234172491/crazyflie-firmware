#ifndef __FLOODINGALGO_H__
#define __FLOODINGALGO_H__

#include "mac.h"

typedef TickType_t fTime_t;

#define F_INTERVAL 2000

void GenerateF(uint16_t myAddress, packet_t *txFPacket, uint8_t timeToLive);        // 生成泛洪消息
bool CheckRxF(const message_t* message, uint16_t myAddress);                        // 检查接收的泛洪消息是否重复并决定转发
void UpdateRxF(const message_t* message);                                           // 更新拓扑结构表

#endif