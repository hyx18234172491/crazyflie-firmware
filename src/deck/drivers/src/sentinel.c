#include "sentinel.h"

#include "FreeRTOSConfig.h"
#include "timers.h"

static TimerHandle_t neighborStateTableSetEvictionTimer;
static Neighbor_State_Table_Set_t neighborStateTableSet;

void initNeighborStateTableSet(Neighbor_State_Table_Set_t *set) {
    set->mu = xSemaphoreCreateMutex();
    set->size = 0;
    set->head = -1;
    for (int i = 0; i < SNIFFER_SWARMRANGING_SIZE_MAX; i++) {
        set->isValid[i] = 0;
        set->prev[i] = -1;
        set->next[i] = -1;
        set->lastRecvNeighborTick[i] = 0;
    }
}

static void neighborStateTableSetEvictionTimerCallback(TimerHandle_t timer) {
  Time_t curTime = xTaskGetTickCount();
  // 对过期时间进行处理
  for (int i = neighborStateTableSet.head; i != -1;) {
        if (neighborStateTableSet.lastRecvNeighborTick[i] + DOWN_AFTER_MILLISECOND < curTime) {  // 超时未收到消息
            
            // 双向链表维护
            
            int16_t prev = neighborStateTableSet.prev[i];
            int16_t next = neighborStateTableSet.next[i];

            // 1. 前边没有节点
            if ( prev == -1) {
                // 说明它自己就是头节点，因此更新头节点
                neighborStateTableSet.head = next;
                // 2.1 后面有节点
                if (next != -1) {
                    neighborStateTableSet.prev[next] = -1;  // 更新他的下一个节点的prev
                }
            } else {
                // 2. 前面一定有节点
                // 2.1 后面有节点
                neighborStateTableSet.next[prev] = next;
                if(next != -1){
                    neighborStateTableSet.prev[next] = prev;
                }
            }
            // 删除这条数据
            neighborStateTableSet.size--;
            neighborStateTableSet.isValid[i] = 0;
            neighborStateTableSet.prev[i] = -1;
            neighborStateTableSet.next[i] = -1;
            // 因为节点i已经被删除，所以直接跳到下一个节点
            i = next;  
        }else{
            // 如果没有删除当前节点，则继续遍历下一个节点
            i = neighborStateTableSet.next[i];
        }
    }
}

void initSentinel() {
  neighborStateTableSetEvictionTimer = xTimerCreate(
      "rangingTableSetEvictionTimer", M2T(RANGING_TABLE_HOLD_TIME / 2), pdTRUE,
      (void *)0, neighborStateTableSetEvictionTimerCallback);
  xTimerStart(neighborStateTableSetEvictionTimer, M2T(0));
}
