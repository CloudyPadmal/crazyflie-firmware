/*
 * pca9554.h - Functions for interfacing PCA95X4 I2C GPIO extender
 *
 * Refer to http://www.ti.com/lit/ds/symlink/pca9555.pdf for more information
 * Also refer http://www.ti.com/lit/ds/symlink/pca9554.pdf for comparisons
 * */

#ifndef __PCA9555_H__
#define __PCA9555_H__

#include <stdint.h>
#include <stdbool.h>

#define PCA9555_DEFAULT_ADDRESS 0b0100000

#define PCA9555_INPUT_REGA   	(0x00)
#define PCA9555_INPUT_REGB   	(0x01)
#define PCA9555_OUTPUT_REGA  	(0x02)
#define PCA9555_OUTPUT_REGB  	(0x03)
#define PCA9555_POL_REGA     	(0x04)
#define PCA9555_POL_REGB     	(0x05)
#define PCA9555_CONFIG_REGA  	(0x06)
#define PCA9555_CONFIG_REGB  	(0x07)

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
bool pca9555Init();

/**
 * Test the PCA9555 sub-system.
 */
bool pca9555Test();

/**
 * Set the output register values.
 */
bool pca9555ConfigOutputRegA(uint32_t val);
bool pca9555ConfigOutputRegB(uint32_t val);

/**
 * Set output bits.
 */
bool pca9555SetOutputRegA(uint32_t mask);
bool pca9555SetOutputRegB(uint32_t mask);

/**
 * Reset output bits.
 */
bool pca9555ClearOutputRegA(uint32_t mask);
bool pca9555ClearOutputRegB(uint32_t mask);

#endif //__PCA9555_H__
