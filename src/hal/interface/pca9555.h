/* pca9554.h - Functions for interfacing PCA95X4 I2C GPIO extender */

#ifndef __PCA9555_H__
#define __PCA9555_H__

#include <stdint.h>
#include <stdbool.h>

#define PCA9555_DEFAULT_ADDRESS 0b0100000

#define PCA9555_INPUT_REG   (0x00)
#define PCA9555_OUTPUT_REG  (0x01)
#define PCA9555_POL_REG     (0x02)
#define PCA9555_CONFIG_REG  (0x03)

#define PCA9555_P0   (1 << 0)
#define PCA9555_P1   (1 << 1)
#define PCA9555_P2   (1 << 2)
#define PCA9555_P3   (1 << 3)
#define PCA9555_P4   (1 << 4)
#define PCA9555_P5   (1 << 5)
#define PCA9555_P6   (1 << 6)
#define PCA9555_P7   (1 << 7)
#define PCA9555_P8   (1 << 8)
#define PCA9555_P9   (1 << 9)
#define PCA9555_P10  (1 << 10)
#define PCA9555_P11  (1 << 11)
#define PCA9555_P12  (1 << 12)
#define PCA9555_P13  (1 << 13)
#define PCA9555_P14  (1 << 14)

/**
 * Initialize the PCA9555 sub-system.
 */
void pca9555Init();

/**
 * Test the PCA9555 sub-system.
 */
bool pca9555Test();

/**
 * Set the output register value.
 */
bool pca9555ConfigOutput(uint32_t val);

/**
 * Set output bits.
 */
bool pca9555SetOutput(uint32_t mask);

/**
 * Reset output bits.
 */
bool pca95x4ClearOutput(uint32_t mask);

#endif //__PCA9555_H__
