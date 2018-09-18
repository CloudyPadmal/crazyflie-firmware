/* pca9555.c - Functions for interfacing PCA9555 I2C GPIO extender */

#define DEBUG_MODULE "PCA9555"

#include "i2cdev.h"

#include "pca9555.h"

#include "debug.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;

void pca9555Init() {
	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = PCA9555_DEFAULT_ADDRESS;
}

/**
 * Reads the config register and checks if there is any error in reading
 * the register
 */
bool pca9555Test() {
	uint8_t tb;
	bool pass_set1, pass_set2;

	// Test reading the config register
	pass_set1 = i2cdevReadByte(I2Cx, devAddr, PCA9555_CONFIG_REGA, &tb);
	pass_set2 = i2cdevReadByte(I2Cx, devAddr, PCA9555_CONFIG_REGB, &tb);

	// TODO: We will turn on the LED at this level
	i2cdevWrite(I2Cx, devAddr, PCA9555_OUTPUT_REGA, /*Find where the LED is and bits are here*/0x1);

	return pass_set1 & pass_set2;
}

bool pca9555ConfigOutput(uint32_t val) {
	bool pass_set1, pass_set2;

	pass_set1 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGA, val);
	pass_set2 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGB, val);

	return pass_set1 & pass_set2;
}

/**
 * TODO: Check how this mask is set as we have two different registers now
 */
bool pca9555SetOutput(uint32_t mask) {
	uint8_t val_1, val_2;
	bool pass_set1, pass_set2;

	pass_set1 = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val_1);
	pass_set2 = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val_2);
	val_1 |= mask;
	val_2 |= mask;
	pass_set1 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val_1);
	pass_set2 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val_2);

	return pass_set1 & pass_set2;
}

bool pca9555ClearOutput(uint32_t mask) {
	uint8_t val_1, val_2;
	bool pass_set1, pass_set2;

	pass_set1 = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val_1);
	pass_set2 = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val_2);
	val_1 &= ~mask;
	val_2 &= ~mask;
	pass_set1 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val_1);
	pass_set2 = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val_2);

	return pass_set1 & pass_set2;
}
