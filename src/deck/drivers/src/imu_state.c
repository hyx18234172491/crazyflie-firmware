#include "imu_state.h"
#include "debug.h"

ImuStateList_t imuStateList;

// Initialize the circular buffer
void initImuStateList(ImuStateList_t *list)
{
    list->head = 0;
    list->curr = 0;
    list->size = 0;
    list->mu = xSemaphoreCreateBinary();
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
    xSemaphoreTake(list->mu, portMAX_DELAY);
    if (isBufferFull(list))
    {
    }
    else
    {
        list->size++;
    }
    list->imuStateList[list->head] = state;
    list->curr = list->head;
    list->head = (list->head + 1) % IMU_STATE_LIST_LENGTH; // Move head forward
    xSemaphoreGive(list->mu);
}

void updateImuState(ImuStateList_t *list, ImuState_t newState, bool isFirstAdd)
{
    xSemaphoreTake(list->mu, portMAX_DELAY);
    DEBUG_PRINT("update\n");
    // 如果现在是空的，或者指定是新插入的，则插入
    if (isBufferEmpty(list) || isFirstAdd == true)
    {
        addImuState(list, newState);
    }
    else
    {
        ImuState_t *currState = &list->imuStateList[list->curr];
        if (currState->allTickCount != 0)
        {
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
        else
        {
            currState->allTickCount = newState.allTickCount;
            currState->gyroZ = newState.gyroZ;
            currState->lastUpdateTick = newState.lastUpdateTick;
            currState->posiZ = newState.posiZ;
            currState->velocityXInWorld = newState.velocityXInWorld;
            currState->velocityYInWorld = newState.velocityYInWorld;
        }
    }
    xSemaphoreGive(list->mu);
}

ImuStateList_t *getGlobalImuState()
{
    return &imuStateList;
}

static void collectHistoryImuStateTimerCallback(TimerHandle_t timer)
{
    DEBUG_PRINT("call_back\n");
    ImuState_t newImuState;
    newImuState.lastUpdateTick = xTaskGetTickCount();
    newImuState.allTickCount = COLLECT_FREQUENCY_TICK;
    estimatorKalmanGetSwarmInfo(&newImuState.velocityXInWorld, &newImuState.velocityYInWorld, &newImuState.gyroZ, &newImuState.posiZ);
    updateImuState(&imuStateList, newImuState, false);
}

void initImuStateTimer()
{
    DEBUG_PRINT("timerStart\n");
    initImuStateList(&imuStateList);
    collectHistoryImuStateTimer = xTimerCreate("imu_state_timer",
                                               M2T(COLLECT_FREQUENCY_TICK * 2),
                                               pdTRUE,
                                               (void *)0,
                                               collectHistoryImuStateTimerCallback);
    if (collectHistoryImuStateTimer != NULL)
    {
        xTimerStart(collectHistoryImuStateTimer, M2T(0));
        DEBUG_PRINT("succ timer");
    }else{
        DEBUG_PRINT("fail timer");
    }
}