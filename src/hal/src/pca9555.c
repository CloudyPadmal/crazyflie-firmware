/* pca9555.c - Functions for interfacing PCA9555 I2C GPIO extender */

#define DEBUG_MODULE "PCA9555"

#include "i2cdev.h"
#include "pca9555.h"
#include "debug.h"
#include "task.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;

bool pca9555Init() {
	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = PCA9555_DEFAULT_ADDRESS;

	return pca9555Test();
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

	return pass_set1 & pass_set2;
}

bool pca9555ConfigOutputRegA(uint32_t val) {
	return i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGA, val);
}

bool pca9555ConfigOutputRegB(uint32_t val) {
	return i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGB, val);
}

bool pca9555SetOutputRegA(uint32_t mask) {
	uint8_t val;
	bool pass;

	pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val);
	val |= mask;
	pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val);

	return pass;
}

bool pca9555SetOutputRegB(uint32_t mask) {
	uint8_t val;
	bool pass;

	pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val);
	val |= mask;
	pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val);

	return pass;
}

bool pca9555ClearOutputRegA(uint32_t mask) {
	uint8_t val;
	bool pass;

	pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val);
	val &= ~mask;
	pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val);

	return pass;
}

bool pca9555ClearOutputRegB(uint32_t mask) {
	uint8_t val;
	bool pass;

	pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val);
	val &= ~mask;
	pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val);

	return pass;
}

void turnLEDON() {
	uint8_t val;

	i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val);
	val = val - 1;
	i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val);
}
