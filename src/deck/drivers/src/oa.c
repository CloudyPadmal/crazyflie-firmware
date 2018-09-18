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

#define OA_PIN_UP     PCA9555_P0
#define OA_PIN_FRONT  PCA9555_P4
#define OA_PIN_BACK   PCA9555_P1
#define OA_PIN_LEFT   PCA9555_P6
#define OA_PIN_RIGHT  PCA9555_P2

// TODO: Assign pins correctly
#define OA_PIN_NORTH  PCA9555_P
#define OA_PIN_NEAST  PCA9555_P
#define OA_PIN_EFLAT  PCA9555_P
#define OA_PIN_EDOWN  PCA9555_P
#define OA_PIN_ERISE  PCA9555_P
#define OA_PIN_SEAST  PCA9555_P
#define OA_PIN_SOUTH  PCA9555_P
#define OA_PIN_SWEST  PCA9555_P
#define OA_PIN_WRISE  PCA9555_P
#define OA_PIN_WDOWN  PCA9555_P
#define OA_PIN_WFLAT  PCA9555_P
#define OA_PIN_NWEST  PCA9555_P
#define OA_PIN_UPPER  PCA9555_P

// New ToFs
static VL53L1_DEV devNORTH;
static VL53L1_DEV devNEAST;
static VL53L1_DEV devEFLAT;
static VL53L1_DEV devEDOWN;
static VL53L1_DEV devERISE;
static VL53L1_DEV devSEAST;
static VL53L1_DEV devSOUTH;
static VL53L1_DEV devSWEST;
static VL53L1_DEV devWRISE;
static VL53L1_DEV devWDOWN;
static VL53L1_DEV devWFLAT;
static VL53L1_DEV devNWEST;
static VL53L1_DEV devUPPER;

// New ranges
static VL53L1_RangingMeasurementData_t rangeNORTH;
static int16_t rNORTH;
static VL53L1_RangingMeasurementData_t rangeNEAST;
static int16_t rNEAST;
static VL53L1_RangingMeasurementData_t rangeEFLAT;
static int16_t rEFLAT;
static VL53L1_RangingMeasurementData_t rangeEDOWN;
static int16_t rEDOWN;
static VL53L1_RangingMeasurementData_t rangeERISE;
static int16_t rERISE;
static VL53L1_RangingMeasurementData_t rangeSEAST;
static int16_t rSEAST;
static VL53L1_RangingMeasurementData_t rangeSOUTH;
static int16_t rSOUTH;
static VL53L1_RangingMeasurementData_t rangeSWEST;
static int16_t rSWEST;
static VL53L1_RangingMeasurementData_t rangeWRISE;
static int16_t rWRISE;
static VL53L1_RangingMeasurementData_t rangeWDOWN;
static int16_t rWDOWN;
static VL53L1_RangingMeasurementData_t rangeWFLAT;
static int16_t rWFLAT;
static VL53L1_RangingMeasurementData_t rangeNWEST;
static int16_t rNWEST;
static VL53L1_RangingMeasurementData_t rangeUPPER;
static int16_t rUPPER;

static void setToFMeasurementModes(VL53L1_DEV Dev) {
	// Dev->Data->LLData->measurement_mode = VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT;
}

static void oaTask(void *param) {
	systemWaitStart();

	setToFMeasurementModes(devNORTH);
	VL53L1_StartMeasurement(devNORTH);
	setToFMeasurementModes(devNEAST);
	VL53L1_StartMeasurement(devNEAST);
	setToFMeasurementModes(devEFLAT);
	VL53L1_StartMeasurement(devEFLAT);
	setToFMeasurementModes(devEDOWN);
	VL53L1_StartMeasurement(devEDOWN);
	setToFMeasurementModes(devERISE);
	VL53L1_StartMeasurement(devERISE);
	setToFMeasurementModes(devSEAST);
	VL53L1_StartMeasurement(devSEAST);
	setToFMeasurementModes(devSOUTH);
	VL53L1_StartMeasurement(devSOUTH);
	setToFMeasurementModes(devSWEST);
	VL53L1_StartMeasurement(devSWEST);
	setToFMeasurementModes(devWRISE);
	VL53L1_StartMeasurement(devWRISE);
	setToFMeasurementModes(devWDOWN);
	VL53L1_StartMeasurement(devWDOWN);
	setToFMeasurementModes(devWFLAT);
	VL53L1_StartMeasurement(devWFLAT);
	setToFMeasurementModes(devNWEST);
	VL53L1_StartMeasurement(devNWEST);
	setToFMeasurementModes(devUPPER);
	VL53L1_StartMeasurement(devUPPER);

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));

		VL53L1_Error eNORTH = VL53L1_GetRangingMeasurementData(devNORTH, &rangeNORTH);
		rNORTH = (eNORTH == VL53L1_ERROR_NONE) ? (rangeNORTH.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devNORTH);

		VL53L1_Error eNEAST = VL53L1_GetRangingMeasurementData(devNEAST, &rangeNEAST);
		rNEAST = (eNEAST == VL53L1_ERROR_NONE) ? (rangeNEAST.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devNEAST);

		VL53L1_Error eEFLAT = VL53L1_GetRangingMeasurementData(devEFLAT, &rangeEFLAT);
		rEFLAT = (eEFLAT == VL53L1_ERROR_NONE) ? (rangeEFLAT.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devEFLAT);

		VL53L1_Error eEDOWN = VL53L1_GetRangingMeasurementData(devEDOWN, &rangeEDOWN);
		rEDOWN = (eEDOWN == VL53L1_ERROR_NONE) ? (rangeEDOWN.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devEDOWN);

		VL53L1_Error eERISE = VL53L1_GetRangingMeasurementData(devERISE, &rangeERISE);
		rERISE = (eERISE == VL53L1_ERROR_NONE) ? (rangeERISE.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devERISE);

		VL53L1_Error eSEAST = VL53L1_GetRangingMeasurementData(devSEAST, &rangeSEAST);
		rSEAST = (eSEAST == VL53L1_ERROR_NONE) ? (rangeSEAST.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devSEAST);

		VL53L1_Error eSOUTH = VL53L1_GetRangingMeasurementData(devSOUTH, &rangeSOUTH);
		rSOUTH = (eSOUTH == VL53L1_ERROR_NONE) ? (rangeSOUTH.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devSOUTH);

		VL53L1_Error eSWEST = VL53L1_GetRangingMeasurementData(devSWEST, &rangeSWEST);
		rSWEST = (eSWEST == VL53L1_ERROR_NONE) ? (rangeSWEST.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devSWEST);

		VL53L1_Error eWRISE = VL53L1_GetRangingMeasurementData(devWRISE, &rangeWRISE);
		rWRISE = (eWRISE == VL53L1_ERROR_NONE) ? (rangeWRISE.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devWRISE);

		VL53L1_Error eWDOWN = VL53L1_GetRangingMeasurementData(devWDOWN, &rangeWDOWN);
		rWDOWN = (eWDOWN == VL53L1_ERROR_NONE) ? (rangeWDOWN.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devWDOWN);

		VL53L1_Error eWFLAT = VL53L1_GetRangingMeasurementData(devWFLAT, &rangeWFLAT);
		rWFLAT = (eWFLAT == VL53L1_ERROR_NONE) ? (rangeWFLAT.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devWFLAT);

		VL53L1_Error eNWEST = VL53L1_GetRangingMeasurementData(devNWEST, &rangeNWEST);
		rNWEST = (eNWEST == VL53L1_ERROR_NONE) ? (rangeNWEST.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devNWEST);

		VL53L1_Error eUPPER = VL53L1_GetRangingMeasurementData(devUPPER, &rangeUPPER);
		rUPPER = (eUPPER == VL53L1_ERROR_NONE) ? (rangeUPPER.RangeMilliMeter) : 0;
		VL53L1_ClearInterruptAndStartMeasurement(devUPPER);
	}
}

static void oaInit() {
	if (isInit) {
		return;
	}

	// Initiate PCA Driver
	pca9555Init();
	// Output port configuration
	pca9555ConfigOutputRegA(~(OA_PIN_UP |
	OA_PIN_RIGHT |
	OA_PIN_LEFT |
	OA_PIN_FRONT |
	OA_PIN_BACK));
	pca9555ConfigOutputRegB(~(OA_PIN_UP |
	OA_PIN_RIGHT |
	OA_PIN_LEFT |
	OA_PIN_FRONT |
	OA_PIN_BACK));
	// Clear output ports
	pca9555ClearOutputRegA(OA_PIN_UP |
	OA_PIN_RIGHT |
	OA_PIN_LEFT |
	OA_PIN_FRONT |
	OA_PIN_BACK);
	pca9555ClearOutputRegB(OA_PIN_UP |
	OA_PIN_RIGHT |
	OA_PIN_LEFT |
	OA_PIN_FRONT |
	OA_PIN_BACK);

	isInit = true;

	xTaskCreate(oaTask, "oa", 2*configMINIMAL_STACK_SIZE, NULL, 3, NULL);
}

static bool oaTest() {
	bool pass = isInit;

	if (isTested) {
		DEBUG_PRINT("Cannot test OA deck a second time\n");
		return false;
	}
/*
	pca9555SetOutputRegA(OA_PIN_FRONT);
	if (vl53l0xInit(&devFront, I2C1_DEV, true)) {
		DEBUG_PRINT("Init front sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init front sensor [FAIL]\n");
		pass = false;
	}

	pca9555SetOutputRegA(OA_PIN_BACK);
	if (vl53l0xInit(&devBack, I2C1_DEV, true)) {
		DEBUG_PRINT("Init back sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init back sensor [FAIL]\n");
		pass = false;
	}

	pca9555SetOutputRegA(OA_PIN_UP);
	if (vl53l0xInit(&devUp, I2C1_DEV, true)) {
		DEBUG_PRINT("Init up sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init up sensor [FAIL]\n");
		pass = false;
	}

	pca9555SetOutputRegB(OA_PIN_LEFT);
	if (vl53l0xInit(&devLeft, I2C1_DEV, true)) {
		DEBUG_PRINT("Init left sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init left sensor [FAIL]\n");
		pass = false;
	}

	pca9555SetOutputRegB(OA_PIN_RIGHT);
	if (vl53l0xInit(&devRight, I2C1_DEV, true)) {
		DEBUG_PRINT("Init right sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init right sensor [FAIL]\n");
		pass = false;
	}*/

	// TODO: Add VL53L1X Sensors


	isTested = true;

	return pass;
}

static const DeckDriver oa_deck = {
		.vid = 0xBC,
		.pid = 0x0B,
		.name = "bcOA",
		.usedGpio = 0,  // FIXME: set the used pins
		.init = oaInit, .test = oaTest, };

DECK_DRIVER(oa_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(oa)
LOG_ADD(LOG_UINT16, NORTH, &rNORTH)/*
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
LOG_ADD(LOG_UINT16, UPPER, &rUPPER)*/
LOG_GROUP_STOP(oa)

