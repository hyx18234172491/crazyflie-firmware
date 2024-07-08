#include "formation.h"

#define RUNNING_STAGE 1 // 0代码debug阶段，1代码运行阶段

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
static bool onGround = true;               // 无人机当前是否在地面上?
static bool isCompleteTaskAndLand = false; // 无人机是否已经执行了飞行任务并落地?
bool keepFlying = false;
static float_t set_height0 = 0.5;
static float_t set_height1 = 0.5;
static float_t set_height2 = 0.5;
static int takeoff_time0 = 2;
static int takeoff_time1 = 2;
static int takeoff_time2 = 2;
static float gotoPosiTime0 = 1;
static float gotoPosiTime1 = 1;
static float gotoPosiTime2 = 1;
typedef enum
{
  XIndex,
  YIndex,
  ZIndex
} PosiIndex;

void formationTask(void *arg)
{
  const float initDist = 1;
  const float doubInitDist = 2;
  static const float targetList[25][3] = {
      {0.0f, 0.0f, 0.0f},                   // 0
      {0.0f, -initDist, 0.0f},              // 1
      {-initDist, -initDist, 0.0f},         // 2
      {-initDist, 0.0f, 0.0f},              // 3
      {-initDist, initDist, 0.0f},          // 4
      {0.0f, initDist, 0.0f},               // 5
      {initDist, initDist, 0.0f},           // 6
      {initDist, 0.0f, 0.0f},               // 7
      {initDist, -initDist, 0.0f},          // 8
      {doubInitDist, initDist, 0.0f},       // 9
      {doubInitDist, 0.0f, 0.0f},           // 10
      {doubInitDist, -initDist, 0.0f},      // 11
      {doubInitDist, -doubInitDist, 0.0f},  // 12
      {initDist, -doubInitDist, 0.0f},      // 13
      {0.0f, -doubInitDist, 0.0f},          // 14
      {-initDist, -doubInitDist, 0.0f},     // 15
      {-doubInitDist, -doubInitDist, 0.0f}, // 16
      {-doubInitDist, -initDist, 0.0f},     // 17
      {-doubInitDist, 0.0f, 0.0f},          // 18
      {-doubInitDist, initDist, 0.0f},      // 19
      {-doubInitDist, doubInitDist, 0.0f},  // 20
      {-initDist, doubInitDist, 0.0f},      // 21
      {0.0f, doubInitDist, 0.0f},           // 22
      {initDist, doubInitDist, 0.0f},       // 23
      {doubInitDist, doubInitDist, 0.0f}};  // 24

  uint8_t SQURE3_3_NUM = 9; // 3阶段转圈的无人机数量+1（0号无人机）
  int8_t targetShift = 0;
  static const uint8_t targetSquere3_3[15] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8 // 8个位置,为了使得索引和值一一对应，所以有0
  };
  static const uint8_t indexToPosi3_4[15] = {
      0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9 // 11个位置
  };
  static const int8_t posiToIndex3_4[15] = {// -1代码无效
                                            -1, 1, 2, 3, 4, 5, 6, 7, -1, 10, 9, 8};
  /*
  posi  0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9
  index 0  1  2  3  4  5  6  7  8   9   10
  */
  systemWaitStart();
  while (1)
  {
    vTaskDelay(10);
    keepFlying = getOrSetKeepflying(MY_UWB_ADDRESS, keepFlying);

    int8_t leaderStage = getLeaderStage();

    if (RUNNING_STAGE == 1) // debug阶段就不能让无人机飞
    {
      if (keepFlying && !isCompleteTaskAndLand)
      {
        // take off
        if (onGround) // 起飞
        {
          if (MY_UWB_ADDRESS == 0) // 0号设置到0号高度
          {
            crtpCommanderHighLevelTakeoff(set_height0, takeoff_time0);
            vTaskDelay(M2T(takeoff_time0));
          }
          else if (MY_UWB_ADDRESS > 0 && MY_UWB_ADDRESS < SQURE3_3_NUM)
          {
            crtpCommanderHighLevelTakeoff(set_height1, takeoff_time1);
            vTaskDelay(M2T(takeoff_time1));
          }
          else
          {
            crtpCommanderHighLevelTakeoff(set_height2, takeoff_time2);
            vTaskDelay(M2T(takeoff_time2));
          }
        }
        if (leaderStage == ZERO_STAGE) // 第0个阶段到达目标点
        {
          if (MY_UWB_ADDRESS == 0)
          {
            crtpCommanderHighLevelGoTo(targetList[MY_UWB_ADDRESS][XIndex], targetList[MY_UWB_ADDRESS][YIndex], targetList[MY_UWB_ADDRESS][ZIndex], 0, gotoPosiTime0, false);
            vTaskDelay(M2T(gotoPosiTime0));
          }
          else if (MY_UWB_ADDRESS > 0 && MY_UWB_ADDRESS < SQURE3_3_NUM)
          {
            crtpCommanderHighLevelGoTo(targetList[MY_UWB_ADDRESS][XIndex], targetList[MY_UWB_ADDRESS][YIndex], targetList[MY_UWB_ADDRESS][ZIndex], 0, gotoPosiTime1, false);
            vTaskDelay(M2T(gotoPosiTime1));
          }
          else
          {
            crtpCommanderHighLevelGoTo(targetList[MY_UWB_ADDRESS][XIndex], targetList[MY_UWB_ADDRESS][YIndex], targetList[MY_UWB_ADDRESS][ZIndex], 0, gotoPosiTime2, false);
            vTaskDelay(M2T(gotoPosiTime2));
          }
        }
        else if (leaderStage >= -30 && leaderStage <= 30) // 第3个阶段，3*3转圈
        {
          if (MY_UWB_ADDRESS == 0)
          {
            crtpCommanderHighLevelGoTo(targetList[MY_UWB_ADDRESS][XIndex], targetList[MY_UWB_ADDRESS][YIndex], targetList[MY_UWB_ADDRESS][ZIndex], 0, gotoPosiTime0, false);
            vTaskDelay(M2T(gotoPosiTime0));
          }
          else if (MY_UWB_ADDRESS > 0 && MY_UWB_ADDRESS < SQURE3_3_NUM)
          {
            int8_t index = MY_UWB_ADDRESS;

            targetShift = leaderStage;
            // 使得targetList在1~UAV_NUM之间偏移
            index = (MY_UWB_ADDRESS + targetShift) % (SQURE3_3_NUM - 1) + 1; // 目标地址索引
          }
          else
          {
            int8_t index = MY_UWB_ADDRESS;
            targetShift = leaderStage + (MY_UWB_ADDRESS - 9) / 3;
            // 使得targetList在1~UAV_NUM之间偏移
            index = (MY_UWB_ADDRESS + targetShift + 1) % 25; // 目标地址索引
            if (index < 9)
              index += 9;
          }
        }
        else
        {
          land(set_height0);
        }
      }
      else
      {
        land(set_height0);
      }
    }
  }
}