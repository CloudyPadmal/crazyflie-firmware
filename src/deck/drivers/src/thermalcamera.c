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

static AMG8833_Dev_t devAMG8833;

static bool isInit = false;
static bool isTested = false;

static float temperature;

static float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

static void tcTask(void *param) {

	systemWaitStart();
	clearInterrupt(&devAMG8833);

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&lastWakeTime, M2T(50));
		readPixels(&devAMG8833, pixels, AMG88xx_PIXEL_ARRAY_SIZE);
		temperature = readThermistor(&devAMG8833);
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
LOG_ADD(LOG_FLOAT, TEMPERATURE, &temperature)
LOG_ADD(LOG_FLOAT, PIXEL_00, &(pixels[0]))
LOG_ADD(LOG_FLOAT, PIXEL_01, &(pixels[1]))
LOG_ADD(LOG_FLOAT, PIXEL_02, &(pixels[2]))
LOG_ADD(LOG_FLOAT, PIXEL_03, &(pixels[13]))
LOG_ADD(LOG_FLOAT, PIXEL_04, &(pixels[24]))
LOG_ADD(LOG_FLOAT, PIXEL_05, &(pixels[35]))
LOG_ADD(LOG_FLOAT, PIXEL_06, &(pixels[46]))
LOG_ADD(LOG_FLOAT, PIXEL_07, &(pixels[57]))
LOG_ADD(LOG_FLOAT, PIXEL_08, &(pixels[63]))
LOG_GROUP_STOP(tc)
