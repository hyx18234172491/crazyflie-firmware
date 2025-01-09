#ifndef SENTINEL_H
#define SENTINEL_H
#include "adhocdeck.h"

/* Queue Constants */
#define SNIFFER_SWARMRANGING_RX_QUEUE_SIZE 10
#define SNIFFER_SWARMRANGING_RX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_With_Timestamp_t)

#define DOWN_AFTER_MILLISECOND 500


typedef enum {
  IS_MASTER_DOWN_BY_ADDRESS,               // 看看当前master是否已经宕机了
  IS_MASTER_DOWN_BY_ADDRESS_REPLY,         // 看看当前master是否已经宕机了的回复

} SENTINEL_COMMAND_TYPE;

typedef struct 
{
    UWB_Address_t address;  // 被sentinel判断为主观下线的master的地址
    uint16_t currentEpoch; // 当前的配置epoch,用于选举leader
    UWB_Address_t runAddress;     // 发消息者的地址，或者空
}Is_Master_Down_By_Address_Message_t;

typedef struct 
{
    uint8_t downState;      // 返回哨兵对master服务器的检查结果, 1代表master下线，0代表master未下线
    UWB_Address_t leaderRunAddress;
    uint16_t leaderEpoch;
}Is_Master_Down_By_Address_Reply_Message_t;

typedef struct Sentinel_Message_t
{
    UWB_Address_t srcAddress;
    SENTINEL_COMMAND_TYPE commandType;
    UWB_Address_t runId;

}Sentinel_Message_t;


typedef struct Sentinel_Message_t
{
    UWB_Address_t currLeader;
    uint16_t currentTerm;

    UWB_Address_t voteFor;
    
}Sentinel_Node_t;



#define SENTINE_RX_QUEUE_SIZE 5


// 哨兵节点的地址都是事先配置好的

// 需要一个数据结构存储当前leader信息
// 维护每一个无人机的最新收到报文的时间，相当于心跳
// 定期检查是否过期
// 如果是follower过期直接从邻居表中清除掉
// 如果是leader主观下线，则向其他哨兵节点发送消息，请求判断，如果大于1/2投票，则主观下线



/* 主观下线后

1. 哨兵之间进行选举，选举出一个leader
    - 如何选举（是否上面成功判断主观下线的人就可以）
2. leader选举一个节点作为master（随机挑一个）
    - 如何选举
3. 通知定位集群当前master节点，（任期，三个节点要一致？）
    - 发布消息
4. 需要其他节点下次发报文均带上当前master节点，如果均带上，则故障切换完成，否则重新广播

*/ 


#endif