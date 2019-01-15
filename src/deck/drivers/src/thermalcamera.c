/*
 * thermalcamera.c
 *
 *  Created on: Dec 28, 2018
 *      Author: bitcraze
 */

#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "TC"

#include "system.h"
#include "debug.h"
#include "log.h"

#include "amg8833.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

// Calibrate values for heat source
#define heatSourceTemperature 		29
#define heatLevelThreshold			10

#define LEFT 						 4
#define MIDDLE						 5
#define RIGHT						 6

static void checkPixelsForThreshold(void);

static AMG8833_Dev_t devAMG8833;

static bool isInit = false;
static bool isTested = false;

static float ambientTemperature;
static float maxTemp;
// Side will be 4 for left, 5 for middle, 6 for right
static uint8_t side;
// This will be 1 for detection and 0 for non
static uint8_t heatSourceDetected;
static uint8_t heatLevel;


static float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
/* =======================
 * <-Camera faced this way
 * =======================
 * 64 63 62 61 60 59 58 57 - 0
 * 56 55 54 53 52 51 50 49 - 1
 * 48 47 46 45 44 43 42 41 - 2
 * 40 39 38 37 36 35 34 33 - 3
 * 32 31 30 29 28 27 26 25 - 4
 * 24 23 22 21 20 19 18 17 - 5
 * 16 15 14 13 12 11 10 09 - 6
 * 08 07 06 05 04 03 02 01 - 7
 */
static uint8_t pixelRows01To57[AMG88xx_PIXEL_ARRAY_SIZE / 8];
static uint8_t pixelRows01To08[AMG88xx_PIXEL_ARRAY_SIZE / 8];

static void tcTask(void *param) {

	systemWaitStart();
	clearInterrupt(&devAMG8833);

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));
		readPixels(&devAMG8833, pixels, AMG88xx_PIXEL_ARRAY_SIZE);
		ambientTemperature = readThermistor(&devAMG8833);
		checkPixelsForThreshold();
	}
}

static void checkPixelsForThreshold() {
	// Initiate iterators
	int step = 0;
	int stepIndex = 0;
	maxTemp = 0;
	heatLevel = 0;
	side = 0;
	heatSourceDetected = 0;
	// Populate pixel row values
	for (int i = 0; i < 8; i++) {
		pixelRows01To57[i] = 0;
		pixelRows01To08[i] = 0;
	}
	// Iterate through each pixel and filter out
	for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
		// Get current temperature value
		float currentPixelTemperature = pixels[i];
		// Prepare a value for bitwise operation
		int bitwise = 0;
		if (currentPixelTemperature > heatSourceTemperature) {
			bitwise = 1 << (7 - stepIndex);
			heatLevel++;
			pixelRows01To57[step] = pixelRows01To57[step] | bitwise;
			bitwise = 0;
			bitwise = 1 << (7 - step);
			pixelRows01To08[stepIndex] = pixelRows01To08[stepIndex] | bitwise;
		}
		step++;
		if (step == 8) {
			step = 0;
			stepIndex++;
		}
		if (maxTemp < currentPixelTemperature) {
			maxTemp = currentPixelTemperature;
		}
	}
	if (heatLevel > heatLevelThreshold) {
		heatSourceDetected = 1;
		if (pixelRows01To08[7] > pixelRows01To08[3] &&
				pixelRows01To08[7] > pixelRows01To08[0]) {
			side = LEFT;
		} else if (pixelRows01To08[0] > pixelRows01To08[3] &&
				pixelRows01To08[0] > pixelRows01To08[7]) {
			side = RIGHT;
		} else {
			side = MIDDLE;
		}
	}
}

static void tcInit() {
	if (isInit) {
		return;
	}
	isInit = begin(&devAMG8833, I2C1_DEV);

	DEBUG_PRINT("Initiated AMG8833 Thermal Camera [%s]\n", isInit ? "OK" : "FAIL");
	xTaskCreate(tcTask, "tc", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
}

static bool tcTest() {
	bool pass = isInit;

	if (isTested) {
		DEBUG_PRINT("Cannot test AMG8833 Thermal camera deck a second time\n");
		return false;
	}

	DEBUG_PRINT("AMG8833 Thermal camera test [%s]\n", pass ? "PASS" : "FAIL");

	isTested = true;
	return pass;
}

static const DeckDriver thermal_deck = {
		.vid = 0xBC,
		.pid = 0x0B,
		.name = "bcThermal",
		.usedGpio = 0,
		.init = tcInit, .test = tcTest, };

DECK_DRIVER(thermal_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcThermal, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(tc)
LOG_ADD(LOG_FLOAT, TEMPERATURE, &ambientTemperature)
LOG_ADD(LOG_UINT8, HEAT_LEVEL, &heatLevel)
LOG_ADD(LOG_FLOAT, MTEMPERATURE, &maxTemp)
LOG_ADD(LOG_UINT8, HEAT_SOURCE_DETECTED, &heatSourceDetected)
LOG_ADD(LOG_UINT8, HEAT_SOURCE_SIDE, &side)

LOG_ADD(LOG_UINT8, PIXEL_ROW_00, &(pixelRows01To57[0]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_01, &(pixelRows01To57[1]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_02, &(pixelRows01To57[2]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_03, &(pixelRows01To57[3]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_04, &(pixelRows01To57[4]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_05, &(pixelRows01To57[5]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_06, &(pixelRows01To57[6]))
LOG_ADD(LOG_UINT8, PIXEL_ROW_07, &(pixelRows01To57[7]))
LOG_GROUP_STOP(tc)
