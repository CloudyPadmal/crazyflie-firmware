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

static void checkPixelsForThreshold(void);

static AMG8833_Dev_t devAMG8833;

static bool isInit = false;
static bool isTested = false;

static float ambientTemperature;
static float maxTemp;
static uint8_t severity;
//static const float heatSourceTemperature = 28;

static float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
static uint32_t pixelRows[AMG88xx_PIXEL_ARRAY_SIZE / 8];

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
	severity = 0;
	// Populate pixel row values
	for (int i = 0; i < 8; i++) {
		pixelRows[i] = 0;
	}
	// Iterate through each pixel and filter out
	for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
		// Get current temperature value
		float currentPixelTemperature = pixels[i];
		// Prepare a value for bitwise operation
		int bitwise = 0;
		if (currentPixelTemperature > ambientTemperature) {
			severity++;
			bitwise = 1 << (7 - step);
		}
		pixelRows[stepIndex] = pixelRows[stepIndex] | bitwise;
		step++;
		if (step == 8) {
			step = 0;
			stepIndex++;
		}
		// Check for maximum temperature
		if (maxTemp < currentPixelTemperature) {
			maxTemp = currentPixelTemperature;
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
LOG_ADD(LOG_FLOAT, MTEMPERATURE, &maxTemp)
LOG_ADD(LOG_UINT32, PIXEL_ROW_00, &(pixelRows[0]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_01, &(pixelRows[1]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_02, &(pixelRows[2]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_03, &(pixelRows[3]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_04, &(pixelRows[4]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_05, &(pixelRows[5]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_06, &(pixelRows[6]))
LOG_ADD(LOG_UINT32, PIXEL_ROW_07, &(pixelRows[7]))
LOG_ADD(LOG_UINT8, SEVERITY, &severity)/*
LOG_ADD(LOG_FLOAT, PIXEL_00, &(pixels[0]))
LOG_ADD(LOG_FLOAT, PIXEL_01, &(pixels[1]))
LOG_ADD(LOG_FLOAT, PIXEL_02, &(pixels[2]))
LOG_ADD(LOG_FLOAT, PIXEL_03, &(pixels[13]))
LOG_ADD(LOG_FLOAT, PIXEL_04, &(pixels[24]))
LOG_ADD(LOG_FLOAT, PIXEL_05, &(pixels[35]))
LOG_ADD(LOG_FLOAT, PIXEL_06, &(pixels[46]))
LOG_ADD(LOG_FLOAT, PIXEL_07, &(pixels[57]))
LOG_ADD(LOG_FLOAT, PIXEL_08, &(pixels[63]))*/
LOG_GROUP_STOP(tc)
