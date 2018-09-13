/* pca9555.c - Functions for interfacing PCA9555 I2C GPIO extender */

#define DEBUG_MODULE "PCA9555"

#include "i2cdev.h"

#include "pca9555.h"

#include "debug.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;

void pca9555Init()
{
  i2cdevInit(I2C1_DEV);
  I2Cx = I2C1_DEV;
  devAddr = PCA9555_DEFAULT_ADDRESS;
}

bool pca9555Test()
{
  uint8_t tb;
  bool pass;

  // Test reading the config register
  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_CONFIG_REG, &tb);

  return pass;
}

bool pca9555ConfigOutput(uint32_t val) {
  bool pass;

  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REG, val);
  return pass;
}

bool pca9555SetOutput(uint32_t mask) {
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REG, &val);
  val |= mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REG, val);

  return pass;
}

bool pca9555ClearOutput(uint32_t mask) {
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REG, &val);
  val &= ~mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REG, val);

  return pass;
}
