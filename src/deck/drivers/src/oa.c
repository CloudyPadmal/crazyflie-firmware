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

#define OA_PIN_LIGHT  PCA9555_P00 // OK
#define OA_PIN_NORTH  PCA9555_P02 // OK
#define OA_PIN_NEAST  PCA9555_P03 // FAIL // FIXME: Check this sensor
#define OA_PIN_EFLAT  PCA9555_P04 // OK
#define OA_PIN_EDOWN  PCA9555_P05 // NOT CONNECTED
#define OA_PIN_ERISE  PCA9555_P06 // NOT CONNECTED
#define OA_PIN_SEAST  PCA9555_P07 // NOT CONNECTED
#define OA_PIN_SOUTH  PCA9555_P10 // OK
#define OA_PIN_SWEST  PCA9555_P11 // NOT CONNECTED
#define OA_PIN_WRISE  PCA9555_P12 // NOT CONNECTED
#define OA_PIN_WDOWN  PCA9555_P13 // NOT CONNECTED
#define OA_PIN_WFLAT  PCA9555_P14 // OK
#define OA_PIN_NWEST  PCA9555_P01 // OK
#define OA_PIN_UPPER  PCA9555_P17 // OK

static VL53L1_Dev_t devNORTH;
static VL53L1_Dev_t devNEAST;
static VL53L1_Dev_t devEFLAT;
static VL53L1_Dev_t devEDOWN;
static VL53L1_Dev_t devERISE;
static VL53L1_Dev_t devSEAST;
static VL53L1_Dev_t devSOUTH;
static VL53L1_Dev_t devSWEST;
static VL53L1_Dev_t devWRISE;
static VL53L1_Dev_t devWDOWN;
static VL53L1_Dev_t devWFLAT;
static VL53L1_Dev_t devNWEST;
static VL53L1_Dev_t devUPPER;

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

static uint16_t oaGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0) {
        VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    VL53L1_Error e = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    DEBUG_PRINT("%d", e);
    range = rangingData.RangeMilliMeter;
    //DEBUG_PRINT("%d\n", range);
    //VL53L1_ClearInterruptAndStartMeasurement(dev);
    VL53L1_StopMeasurement(dev);VL53L1_StartMeasurement(dev);

    return range;
}

static void oaTask(void *param) {

	systemWaitStart();

	VL53L1_StopMeasurement(&devNORTH);VL53L1_StartMeasurement(&devNORTH);
	//VL53L1_StopMeasurement(&devNEAST);VL53L1_StartMeasurement(&devNEAST);
	VL53L1_StopMeasurement(&devEFLAT);VL53L1_StartMeasurement(&devEFLAT);
	//VL53L1_StopMeasurement(&devEDOWN);VL53L1_StartMeasurement(&devEDOWN);
	//VL53L1_StopMeasurement(&devERISE);VL53L1_StartMeasurement(&devERISE);
	//VL53L1_StopMeasurement(&devSEAST);VL53L1_StartMeasurement(&devSEAST);
	VL53L1_StopMeasurement(&devSOUTH);VL53L1_StartMeasurement(&devSOUTH);
	//VL53L1_StopMeasurement(&devSWEST);VL53L1_StartMeasurement(&devSWEST);
	//VL53L1_StopMeasurement(&devWRISE);VL53L1_StartMeasurement(&devWRISE);
	//VL53L1_StopMeasurement(&devWDOWN);VL53L1_StartMeasurement(&devWDOWN);
	VL53L1_StopMeasurement(&devWFLAT);VL53L1_StartMeasurement(&devWFLAT);
	VL53L1_StopMeasurement(&devNWEST);VL53L1_StartMeasurement(&devNWEST);
	VL53L1_StopMeasurement(&devUPPER);VL53L1_StartMeasurement(&devUPPER);

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));
		DEBUG_PRINT("Getting North");
		rNORTH = oaGetMeasurementAndRestart(&devNORTH);
		//rNEAST = oaGetMeasurementAndRestart(&devNEAST);
		DEBUG_PRINT("Getting EFlat");
		rEFLAT = oaGetMeasurementAndRestart(&devEFLAT);
		//rEDOWN = oaGetMeasurementAndRestart(&devEDOWN);
		//rERISE = oaGetMeasurementAndRestart(&devERISE);
		//rSEAST = oaGetMeasurementAndRestart(&devSEAST);
		rSOUTH = oaGetMeasurementAndRestart(&devSOUTH);
		//rSWEST = oaGetMeasurementAndRestart(&devSWEST);
		//rWRISE = oaGetMeasurementAndRestart(&devWRISE);
		//rWDOWN = oaGetMeasurementAndRestart(&devWDOWN);
		rWFLAT = oaGetMeasurementAndRestart(&devWFLAT);
		DEBUG_PRINT("Getting NWest");
		rNWEST = oaGetMeasurementAndRestart(&devNWEST);
		DEBUG_PRINT("Getting Upper");
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
			OA_PIN_LIGHT |
			OA_PIN_NWEST |
			OA_PIN_NORTH |
			OA_PIN_NEAST |
			OA_PIN_EFLAT |
			OA_PIN_EDOWN |
			OA_PIN_ERISE |
			OA_PIN_SEAST
	));
	pca9555ConfigOutputRegB(~(
			OA_PIN_SOUTH |
			OA_PIN_SWEST |
			OA_PIN_WRISE |
			OA_PIN_WDOWN |
			OA_PIN_WFLAT |
			OA_PIN_UPPER
	));
	// Clear output ports
	pca9555ClearOutputRegA(
			OA_PIN_LIGHT |
			OA_PIN_NWEST |
			OA_PIN_NORTH |
			OA_PIN_NEAST |
			OA_PIN_EFLAT |
			OA_PIN_EDOWN |
			OA_PIN_ERISE |
			OA_PIN_SEAST
	);
	pca9555ClearOutputRegB(
			OA_PIN_SOUTH |
			OA_PIN_SWEST |
			OA_PIN_WRISE |
			OA_PIN_WDOWN |
			OA_PIN_WFLAT |
			OA_PIN_UPPER
	);

	isInit = true;
	DEBUG_PRINT("Initiated FYP OA Deck [OK]\n");
	xTaskCreate(oaTask, "oa", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
}

static bool oaTest() {
	bool pass = isInit;

	if (isTested) {
		DEBUG_PRINT("Cannot test FYP OA deck a second time\n");
		return false;
	}

	pca9555SetOutputRegA(OA_PIN_NWEST);
	if (vl53l1xInit(&devNWEST, I2C1_DEV)) {
		DEBUG_PRINT("Init NWEST sensor [OK]\n"); // OK
	} else {
		DEBUG_PRINT("Init NWEST sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_NORTH);
	if (vl53l1xInit(&devNORTH, I2C1_DEV)) {
		DEBUG_PRINT("Init NORTH sensor [OK]\n"); // OK
	} else {
		DEBUG_PRINT("Init NORTH sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_NEAST);
	if (vl53l1xInit(&devNEAST, I2C1_DEV)) {
		DEBUG_PRINT("Init NEAST sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init NEAST sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_EFLAT);
	if (vl53l1xInit(&devEFLAT, I2C1_DEV)) {
		DEBUG_PRINT("Init EFLAT sensor [OK]\n"); // OK
	} else {
		DEBUG_PRINT("Init EFLAT sensor [FAIL]\n");
		pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_EDOWN);
	if (vl53l1xInit(&devEDOWN, I2C1_DEV)) {
		DEBUG_PRINT("Init EDOWN sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init EDOWN sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_ERISE);
	if (vl53l1xInit(&devERISE, I2C1_DEV)) {
		DEBUG_PRINT("Init ERISE sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init ERISE sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegA(OA_PIN_SEAST);
	if (vl53l1xInit(&devSEAST, I2C1_DEV)) {
		DEBUG_PRINT("Init SEAST sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init SEAST sensor [FAIL]\n");
		//pass = false;
	}

	pca9555SetOutputRegB(OA_PIN_SOUTH);
	if (vl53l1xInit(&devSOUTH, I2C1_DEV)) {
		DEBUG_PRINT("Init SOUTH sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init SOUTH sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_SWEST);
	if (vl53l1xInit(&devSWEST, I2C1_DEV)) {
		DEBUG_PRINT("Init SWEST sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init SWEST sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WRISE);
	if (vl53l1xInit(&devWRISE, I2C1_DEV)) {
		DEBUG_PRINT("Init WRISE sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init WRISE sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WDOWN);
	if (vl53l1xInit(&devWDOWN, I2C1_DEV)) {
		DEBUG_PRINT("Init WDOWN sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init WDOWN sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_WFLAT);
	if (vl53l1xInit(&devWFLAT, I2C1_DEV)) {
		DEBUG_PRINT("Init WFLAT sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init WFLAT sensor [FAIL]\n");
		//pass = false;
	}
	pca9555SetOutputRegB(OA_PIN_UPPER);
	if (vl53l1xInit(&devUPPER, I2C1_DEV)) {
		DEBUG_PRINT("Init UPPER sensor [OK]\n");
	} else {
		DEBUG_PRINT("Init UPPER sensor [FAIL]\n");
		//pass = false;
	}

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

