/*
 * pca9554.h - Functions for interfacing PCA9555 I2C GPIO extender
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

// Belongs to Set A
#define PCA9555_P00   (1 << 0)
#define PCA9555_P01   (1 << 1)
#define PCA9555_P02   (1 << 2)
#define PCA9555_P03   (1 << 3)
#define PCA9555_P04   (1 << 4)
#define PCA9555_P05   (1 << 5)
#define PCA9555_P06   (1 << 6)
#define PCA9555_P07   (1 << 7)
// Belongs to Set B
#define PCA9555_P10   (1 << 0)
#define PCA9555_P11   (1 << 1)
#define PCA9555_P12   (1 << 2)
#define PCA9555_P13   (1 << 3)
#define PCA9555_P14   (1 << 4)
#define PCA9555_P15   (1 << 5)
#define PCA9555_P16   (1 << 6)
#define PCA9555_P17   (1 << 7)

/**
 * Initialize the PCA9555 sub-system.
 */
void pca9555Init();

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

/**
 * Turns LED on to indicate deck has initiated successfully.
 */
void turnLEDON();

#endif //__PCA9555_H__
