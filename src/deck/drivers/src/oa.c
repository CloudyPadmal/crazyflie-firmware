/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
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
/* oa.c: Codename Obstacle Avoidance driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "OA"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca9555.h"

#include "vl53l1x.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;

#define OA_PIN_LIGHT  PCA9555_P00
#define OA_PIN_NORTH  PCA9555_P02
#define OA_PIN_NEAST  PCA9555_P03
#define OA_PIN_EFLAT  PCA9555_P04
#define OA_PIN_EDOWN  PCA9555_P05
#define OA_PIN_ERISE  PCA9555_P06
#define OA_PIN_SEAST  PCA9555_P07
#define OA_PIN_SOUTH  PCA9555_P10
#define OA_PIN_SWEST  PCA9555_P11
#define OA_PIN_WRISE  PCA9555_P12
#define OA_PIN_WDOWN  PCA9555_P13
#define OA_PIN_WFLAT  PCA9555_P14
#define OA_PIN_NWEST  PCA9555_P01
#define OA_PIN_UPPER  PCA9555_P17

static VL53L1_Dev_t devNORTH; // 50
static VL53L1_Dev_t devNEAST; // 51
static VL53L1_Dev_t devEFLAT; // 52
static VL53L1_Dev_t devEDOWN; // 53
static VL53L1_Dev_t devERISE; // 54
static VL53L1_Dev_t devSEAST; // 55
static VL53L1_Dev_t devSOUTH; // 56
static VL53L1_Dev_t devSWEST; // 57
static VL53L1_Dev_t devWRISE; // 58
static VL53L1_Dev_t devWDOWN; // 59
static VL53L1_Dev_t devWFLAT; // 60
static VL53L1_Dev_t devNWEST; // 49
static VL53L1_Dev_t devUPPER; // 61

static uint16_t rNORTH;
static uint16_t rNEAST;
static uint16_t rEFLAT;
static uint16_t rEDOWN;
static uint16_t rERISE;
static uint16_t rSEAST;
static uint16_t rSOUTH;
static uint16_t rSWEST;
static uint16_t rWRISE;
static uint16_t rWDOWN;
static uint16_t rWFLAT;
static uint16_t rNWEST;
static uint16_t rUPPER;

static void setupROIs(VL53L1_Dev_t *dev, int lx, int ly, int rx, int ry)
{
	VL53L1_CalibrationData_t d;
	VL53L1_GetCalibrationData(&devUPPER, &d);
	DEBUG_PRINT("Optical Center Decimal (%u, %u)\n", d.optical_centre.x_centre, d.optical_centre.y_centre);
	VL53L1_UserRoi_t t;
	t.TopLeftX = lx;
	t.TopLeftY = ly;
	t.BotRightX = rx;
	t.BotRightY = ry;
	VL53L1_SetUserROI(dev, &t);
	VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_MEDIUM);
}

static uint16_t oaGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0) {
    	VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;
    VL53L1_StopMeasurement(dev);VL53L1_StartMeasurement(dev);

    return range;
}

static void oaTask(void *param) {

	systemWaitStart();

	VL53L1_StopMeasurement(&devNORTH);VL53L1_StartMeasurement(&devNORTH);
	VL53L1_StopMeasurement(&devNEAST);VL53L1_StartMeasurement(&devNEAST);
	VL53L1_StopMeasurement(&devEFLAT);VL53L1_StartMeasurement(&devEFLAT);
	VL53L1_StopMeasurement(&devEDOWN);VL53L1_StartMeasurement(&devEDOWN);
	VL53L1_StopMeasurement(&devERISE);VL53L1_StartMeasurement(&devERISE);
	VL53L1_StopMeasurement(&devSEAST);VL53L1_StartMeasurement(&devSEAST);
	VL53L1_StopMeasurement(&devSOUTH);VL53L1_StartMeasurement(&devSOUTH);
	VL53L1_StopMeasurement(&devSWEST);VL53L1_StartMeasurement(&devSWEST);
	VL53L1_StopMeasurement(&devWRISE);VL53L1_StartMeasurement(&devWRISE);
	VL53L1_StopMeasurement(&devWDOWN);VL53L1_StartMeasurement(&devWDOWN);
	VL53L1_StopMeasurement(&devWFLAT);VL53L1_StartMeasurement(&devWFLAT);
	VL53L1_StopMeasurement(&devNWEST);VL53L1_StartMeasurement(&devNWEST);
	VL53L1_StopMeasurement(&devUPPER);VL53L1_StartMeasurement(&devUPPER);

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));
		rNORTH = oaGetMeasurementAndRestart(&devNORTH);
		rNEAST = oaGetMeasurementAndRestart(&devNEAST);
		rEFLAT = oaGetMeasurementAndRestart(&devEFLAT);
		rEDOWN = oaGetMeasurementAndRestart(&devEDOWN);
		rERISE = oaGetMeasurementAndRestart(&devERISE);
		rSEAST = oaGetMeasurementAndRestart(&devSEAST);
		rSOUTH = oaGetMeasurementAndRestart(&devSOUTH);
		rSWEST = oaGetMeasurementAndRestart(&devSWEST);
		rWRISE = oaGetMeasurementAndRestart(&devWRISE);
		rWDOWN = oaGetMeasurementAndRestart(&devWDOWN);
		rWFLAT = oaGetMeasurementAndRestart(&devWFLAT);
		rNWEST = oaGetMeasurementAndRestart(&devNWEST);
		rUPPER = oaGetMeasurementAndRestart(&devUPPER);
	}
}

static void oaInit() {
	if (isInit) {
		return;
	}
	// Initiate PCA Driver
	pca9555Init();
	// Output port configuration
	pca9555ConfigOutputRegA(~(
			OA_PIN_LIGHT | OA_PIN_NWEST | OA_PIN_NORTH |
			OA_PIN_NEAST | OA_PIN_EFLAT | OA_PIN_EDOWN |
			OA_PIN_ERISE | OA_PIN_SEAST
	));
	pca9555ConfigOutputRegB(~(
			OA_PIN_SOUTH | OA_PIN_SWEST | OA_PIN_WRISE |
			OA_PIN_WDOWN | OA_PIN_WFLAT | OA_PIN_UPPER
	));
	// Clear output ports
	pca9555ClearOutputRegA(
			OA_PIN_LIGHT | OA_PIN_NWEST | OA_PIN_NORTH |
			OA_PIN_NEAST | OA_PIN_EFLAT | OA_PIN_EDOWN |
			OA_PIN_ERISE | OA_PIN_SEAST
	);
	pca9555ClearOutputRegB(
			OA_PIN_SOUTH | OA_PIN_SWEST | OA_PIN_WRISE |
			OA_PIN_WDOWN | OA_PIN_WFLAT | OA_PIN_UPPER
	);

	isInit = true;
	DEBUG_PRINT("Initiated FYP bcOA Deck [OK]\n");
	xTaskCreate(oaTask, "oa", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
}

static bool oaTest() {
	bool pass = isInit;

	if (isTested) {
		DEBUG_PRINT("Cannot test FYP bcOA deck a second time\n");
		return false;
	}

	pca9555SetOutputRegA(OA_PIN_LIGHT);
	DEBUG_PRINT("Initiating ToF Setup!\n");

	pca9555SetOutputRegA(OA_PIN_NWEST);
	if (vl53l1xInit(&devNWEST, I2C1_DEV)) {
		DEBUG_PRINT("Init NWEST sensor [OK]\n");
		setupROIs(&devNWEST, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init NWEST sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_NORTH);
	if (vl53l1xInit(&devNORTH, I2C1_DEV)) {
		DEBUG_PRINT("Init NORTH sensor [OK]\n");
		setupROIs(&devNORTH, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init NORTH sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_NEAST);
	if (vl53l1xInit(&devNEAST, I2C1_DEV)) {
		DEBUG_PRINT("Init NEAST sensor [OK]\n");
		setupROIs(&devNEAST, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init NEAST sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_EFLAT);
	if (vl53l1xInit(&devEFLAT, I2C1_DEV)) {
		DEBUG_PRINT("Init EFLAT sensor [OK]\n");
		setupROIs(&devEFLAT, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init EFLAT sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_EDOWN);
	if (vl53l1xInit(&devEDOWN, I2C1_DEV)) {
		DEBUG_PRINT("Init EDOWN sensor [OK]\n");
		setupROIs(&devEDOWN, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init EDOWN sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_ERISE);
	if (vl53l1xInit(&devERISE, I2C1_DEV)) {
		DEBUG_PRINT("Init ERISE sensor [OK]\n");
		setupROIs(&devERISE, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init ERISE sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_SEAST);
	if (vl53l1xInit(&devSEAST, I2C1_DEV)) {
		DEBUG_PRINT("Init SEAST sensor [OK]\n");
		setupROIs(&devSEAST, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init SEAST sensor [FAIL]\n");
		pass = false;
	}

	pca9555SetOutputRegB(OA_PIN_SOUTH);
	if (vl53l1xInit(&devSOUTH, I2C1_DEV)) {
		DEBUG_PRINT("Init SOUTH sensor [OK]\n");
		setupROIs(&devSOUTH, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init SOUTH sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_SWEST);
	if (vl53l1xInit(&devSWEST, I2C1_DEV)) {
		DEBUG_PRINT("Init SWEST sensor [OK]\n");
		setupROIs(&devSWEST, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init SWEST sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WRISE);
	if (vl53l1xInit(&devWRISE, I2C1_DEV)) {
		DEBUG_PRINT("Init WRISE sensor [OK]\n");
		setupROIs(&devWRISE, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init WRISE sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WDOWN);
	if (vl53l1xInit(&devWDOWN, I2C1_DEV)) {
		DEBUG_PRINT("Init WDOWN sensor [OK]\n");
		setupROIs(&devWDOWN, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init WDOWN sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WFLAT);
	if (vl53l1xInit(&devWFLAT, I2C1_DEV)) {
		DEBUG_PRINT("Init WFLAT sensor [OK]\n");
		setupROIs(&devWFLAT, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init WFLAT sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_UPPER);
	if (vl53l1xInit(&devUPPER, I2C1_DEV)) {
		DEBUG_PRINT("Init UPPER sensor [OK]\n");
		setupROIs(&devUPPER, 6, 8, 9, 5);
	} else {
		DEBUG_PRINT("Init UPPER sensor [FAIL]\n");
		pass = false;
	}
	// Turn LED ON
	turnLEDON();
	isTested = true;
	return pass;
}

static const DeckDriver oa_deck = {
		.vid = 0xBC,
		.pid = 0x0B,
		.name = "bcOA",
		.usedGpio = 0,
		.init = oaInit, .test = oaTest, };

DECK_DRIVER(oa_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(oa)
LOG_ADD(LOG_UINT16, NORTH, &rNORTH)
LOG_ADD(LOG_UINT16, NEAST, &rNEAST)
LOG_ADD(LOG_UINT16, EFLAT, &rEFLAT)
LOG_ADD(LOG_UINT16, EDOWN, &rEDOWN)
LOG_ADD(LOG_UINT16, ERISE, &rERISE)
LOG_ADD(LOG_UINT16, SEAST, &rSEAST)
LOG_ADD(LOG_UINT16, SOUTH, &rSOUTH)
LOG_ADD(LOG_UINT16, SWEST, &rSWEST)
LOG_ADD(LOG_UINT16, WRISE, &rWRISE)
LOG_ADD(LOG_UINT16, WDOWN, &rWDOWN)
LOG_ADD(LOG_UINT16, WFLAT, &rWFLAT)
LOG_ADD(LOG_UINT16, NWEST, &rNWEST)
LOG_ADD(LOG_UINT16, UPPER, &rUPPER)
LOG_GROUP_STOP(oa)
