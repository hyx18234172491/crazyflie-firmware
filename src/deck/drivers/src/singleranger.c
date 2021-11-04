/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2021, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;

#define MR_PIN_UP     PCA95X4_P0
#define MR_PIN_FRONT  PCA95X4_P4
#define MR_PIN_BACK   PCA95X4_P1
#define MR_PIN_LEFT   PCA95X4_P6
#define MR_PIN_RIGHT  PCA95X4_P2

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

// Hard Code: only OFFSET
#define OFFSET 1
#define SIZE 4
#define COUNT (16 - SIZE) / OFFSET + 1
// VL53L1_Dev_t devOri[] = [devFront, devBack, devUp, devLeft, devRight];
/**
 * Mode of ROI setting, setting by PARAMETER.
 * 0 | Default value, select ROI according to the setting coornidates.
 * 1 | Select ROI according to the pre-generated coornidate array and the polling index.
 */
uint16_t mode = 1;
// coornidate pair for mode 0, setting by PARAMETER.
uint16_t topLeftX = 0, topLeftY = 15, botRightX = 15, botRightY = 0;
// coornidate pairs for mode 1, setting by beforeRanging()

VL53L1_UserRoi_t roiConfig[COUNT * COUNT];
uint16_t roiIndex;

/**
 * Ranging orientation, setting by PARAMETER.
 * 1 | front
 * 2 | back
 * 3 | up
 * 4 | left
 * 5 | right
 */
uint16_t orientation = 3;

/**
 * Pre-generate ROI coornidates for mode 1, set the orientation.
 */
static void beforeRanging()
{
    uint16_t x, y;
    roiIndex = 0;
	for (y = 0; y < COUNT; y++) {
		for (x = 0; x < COUNT; x++) {
            VL53L1_UserRoi_t cur = {OFFSET*x, (15-OFFSET*y), (OFFSET*x+3), (15-OFFSET*y-3)};
			roiConfig[roiIndex] = cur;
			roiIndex++;
		}
	}
    roiIndex = 0;
}


static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;

    VL53L1_StopMeasurement(dev);

    if(mode == 0) {
        VL53L1_UserRoi_t roiConfigStruct = {topLeftX, topLeftY, botRightX, botRightY};
        status = VL53L1_SetUserROI(dev, &roiConfigStruct);
    } else {
        status = VL53L1_SetUserROI(dev, &roiConfig[roiIndex]);
    }
    
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

static void mrSingleTask(void *param)
{
    beforeRanging();
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Restart the sensor
    status = VL53L1_StopMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devFront);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(20));

        // Select ROI config circularly
        roiIndex++;
        if(roiIndex == COUNT * COUNT) roiIndex = 0;

        rangeSet(rangeSingle, mrGetMeasurementAndRestart(&devFront)/1000.0f);
        roiSet(roiIndex);
        topLeftXSet(topLeftX);
        topLeftYSet(topLeftY);
        botRightXSet(botRightX);
        botRightYSet(botRightY);
        orientationSet(orientation);

        // Test ROI setting
        VL53L1_UserRoi_t pRoi;
        VL53L1_GetUserROI(&devFront, &pRoi);
        // DEBUG_PRINT("[INFO]pRoi: %d, %d, %d, %d\n", (int)pRoi.TopLeftX, (int)pRoi.TopLeftY, (int)pRoi.BotRightX, (int)pRoi.BotRightY);
    }
}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    pca95x4ConfigOutput(~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrSingleTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL, MULTIRANGER_TASK_PRI, NULL);
}

static bool mrInitSensor(VL53L1_Dev_t *pdev, uint32_t pca95pin, char *name)
{
  bool status;

  // Bring up VL53 by releasing XSHUT
  pca95x4SetOutput(pca95pin);
  // Let VL53 boot
  vTaskDelay(M2T(2));
  // Init VL53
  if (vl53l1xInit(pdev, I2C1_DEV))
  {
      DEBUG_PRINT("Init %s sensor [OK]\n", name);
      status = true;
  }
  else
  {
      DEBUG_PRINT("Init %s sensor [FAIL]\n", name);
      status = false;
  }

  return status;
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    isPassed &= mrInitSensor(&devFront, MR_PIN_FRONT, "front");
    isPassed &= mrInitSensor(&devBack, MR_PIN_BACK, "back");
    isPassed &= mrInitSensor(&devUp, MR_PIN_UP, "up");
    isPassed &= mrInitSensor(&devLeft, MR_PIN_LEFT, "left");
    isPassed &= mrInitSensor(&devRight, MR_PIN_RIGHT, "right");

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "seuSingleranger",

    .usedGpio = 0,
    .usedPeriph = DECK_USING_I2C,

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)


PARAM_GROUP_STOP(deck)

/**
 * Parameter group for the multi-ranger deck in single mode.
 */
PARAM_GROUP_START(mrdeck)
PARAM_ADD_CORE(PARAM_UINT16, mode, &mode)
PARAM_ADD_CORE(PARAM_UINT16, orientation, &orientation)

PARAM_ADD_CORE(PARAM_UINT16, topleftx, &topLeftX)
PARAM_ADD_CORE(PARAM_UINT16, toplefty, &topLeftY)
PARAM_ADD_CORE(PARAM_UINT16, botrightx, &botRightX)
PARAM_ADD_CORE(PARAM_UINT16, botrighty, &botRightY)
PARAM_GROUP_STOP(mrdeck)
