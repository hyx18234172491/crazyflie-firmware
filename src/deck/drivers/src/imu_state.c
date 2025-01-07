#include "imu_state.h"

// Initialize the circular buffer
void initImuStateList(ImuStateList_t *list)
{
    list->head = 0;
    list->tail = 0;
    list->size = 0;
}

// Check if the buffer is full
int isBufferFull(ImuStateList_t *list)
{
    return list->size == IMU_STATE_LIST_LENGTH;
}

// Check if the buffer is empty
int isBufferEmpty(ImuStateList_t *list)
{
    return list->size == 0;
}

// Add a new IMU state to the buffer
void addImuState(ImuStateList_t *list, ImuState_t state)
{
    if (isBufferFull(list))
    {
        // Overwrite the oldest element if buffer is full
        list->tail = (list->tail + 1) % IMU_STATE_LIST_LENGTH; // Move tail forward
    }
    else
    {
        list->size++;
    }

    list->ImuStateList[list->head] = state;
    list->curr = list->head;
    list->head = (list->head + 1) % IMU_STATE_LIST_LENGTH; // Move head forward
}

void updateImuState(ImuStateList_t *list, ImuState_t newState, bool isFirstAdd)
{
    // 如果现在是空的，或者指定是新插入的，则插入
    if (isBufferEmpty(list) || isFirstAdd == true)
    {
        addImuState(list, newState);
    }
    else
    {

        ImuState_t *currState = &list->ImuStateList[list->curr];
        // 和当前的进行更新
        uint32_t diffTickCount = newState.lastUpdateTick - currState->lastUpdateTick;
        uint32_t allTickCount = diffTickCount + currState->allTickCount;
        currState->velocityXInWorld = ((currState->allTickCount * currState->velocityXInWorld) + (diffTickCount * newState.velocityXInWorld) / (allTickCount));
        currState->velocityYInWorld = ((currState->allTickCount * currState->velocityYInWorld) + (diffTickCount * newState.velocityYInWorld) / (allTickCount));
        currState->gyroZ = ((currState->allTickCount * currState->gyroZ) + (diffTickCount * newState.gyroZ) / (allTickCount));
        currState->posiZ = newState.posiZ;

        // 更新最新的均值的时间
        currState->lastUpdateTick = newState.lastUpdateTick;
        // 更新当前均值持续的时间
        currState->allTickCount = allTickCount;
    }
}

static void collectHistoryImuStateTimerCallback(TimerHandle_t timer)
{
    ImuState_t newImuState;
    newImuState.lastUpdateTick = xTaskGetTickCount();
    newImuState.allTickCount = COLLECT_FREQUENCY_TICK;
    estimatorKalmanGetSwarmInfo(&newImuState.velocityXInWorld, &newImuState.velocityYInWorld, &newImuState.gyroZ, &newImuState.posiZ);
    updateImuState(&imuStateList, newImuState, false);
}

void imuStateInit()
{

    collectHistoryImuStateTimer = xTimerCreate("rangingTableSetEvictionTimer",
                                               M2T(COLLECT_FREQUENCY_TICK),
                                               pdTRUE,
                                               (void *)0,
                                               collectHistoryImuStateTimerCallback);
    xTimerStart(collectHistoryImuStateTimer, M2T(0));
}