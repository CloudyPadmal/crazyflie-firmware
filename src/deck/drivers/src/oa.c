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

/*static VL53L0xDev devFront;
static VL53L0xDev devBack;
static VL53L0xDev devUp;
static VL53L0xDev devLeft;
static VL53L0xDev devRight;*/

// New ToFs
static VL53L1_DEV devNORTH;/*
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
static VL53L1_DEV devUPPER;*/

/*static uint16_t rangeFront;
static uint16_t rangeBack;
static uint16_t rangeUp;
static uint16_t rangeLeft;
static uint16_t rangeRight;*/

// New ranges
static VL53L1_RangingMeasurementData_t rangeNORTH;
static int16_t rNORTH;/*
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
static int16_t rUPPER;*/

static void oaTask(void *param) {
	systemWaitStart();

	VL53L1_StartMeasurement(devNORTH);

	/*vl53l0xStartContinuous(&devFront, 0);
	vl53l0xStartContinuous(&devBack, 0);
	vl53l0xStartContinuous(&devUp, 0);
	vl53l0xStartContinuous(&devLeft, 0);
	vl53l0xStartContinuous(&devRight, 0);*/

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));

		VL53L1_Error eNORTH = VL53L1_GetRangingMeasurementData(devNORTH, &rangeNORTH);
		rNORTH = (eNORTH == VL53L1_ERROR_NONE) ? (rangeNORTH.RangeMilliMeter) : 0;

		/*rangeFront = vl53l0xReadRangeContinuousMillimeters(&devFront);
		rangeBack = vl53l0xReadRangeContinuousMillimeters(&devBack);
		rangeUp = vl53l0xReadRangeContinuousMillimeters(&devUp);
		rangeLeft = vl53l0xReadRangeContinuousMillimeters(&devLeft);
		rangeRight = vl53l0xReadRangeContinuousMillimeters(&devRight);*/
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

	xTaskCreate(oaTask, "oa", 2*configMINIMAL_STACK_SIZE, NULL, /*priority*/3,
			NULL);
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

