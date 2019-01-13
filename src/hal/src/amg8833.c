#include "amg8833.h"
#include "i2cdev.h"
#include "task.h"
#include "num.h"

#include "debug.h"

const float AMG88xx_TEMP_CONVERSION = 0.25;
const float AMG88xx_THRM_CONVERSION = 0.0625;

static uint8_t mode = 1;

/**************************************************************************/
/*!
 @brief  Setups the I2C interface and hardware
 @returns True if device is set up, false on any failure
 */
/**************************************************************************/
bool begin(AMG8833_Dev_t *pdev, I2C_Dev *I2Cx)
{
	// Set I2C parameters
	pdev->I2Cx = I2Cx;
	pdev->devAddr = AMG88xx_ADDRESS;
	bool i2c_complete = i2cdevInit(pdev->I2Cx);
	// Enter normal mode
	bool mode_selected = write8(pdev, AMG88xx_PCTL, AMG88xx_NORMAL_MODE);
	// Software reset
	bool software_resetted = write8(pdev, AMG88xx_RST, AMG88xx_INITIAL_RESET);
	//disable interrupts by default
	bool interrupts_set = disableInterrupt(pdev);
	//set to 10 FPS
	bool fps_set = write8(pdev, AMG88xx_FPSC, (AMG88xx_FPS_10 & 0x01));
	// Delay
	vTaskDelay(M2T(10));
	return i2c_complete && mode_selected && software_resetted && interrupts_set && fps_set;
}

/**************************************************************************/
/*!
 @brief  Set the moving average mode.
 @param  mode if True is passed, output will be twice the moving average
 */
/**************************************************************************/
void setMovingAverageMode(AMG8833_Dev_t *dev, bool mode)
{
	write8(dev, AMG88xx_AVE, (mode << 5));
}

/**************************************************************************/
/*!
 @brief  Set the interrupt levels. The hysteresis value defaults to .95 * high
 @param  high the value above which an interrupt will be triggered
 @param  low the value below which an interrupt will be triggered
 */
/**************************************************************************/
void setInterruptLevels_N(AMG8833_Dev_t *dev, float high, float low)
{
	setInterruptLevels_H(dev, high, low, high * 0.95f);
}

/**************************************************************************/
/*!
 @brief  Set the interrupt levels
 @param  high the value above which an interrupt will be triggered
 @param  low the value below which an interrupt will be triggered
 @param hysteresis the hysteresis value for interrupt detection
 */
/**************************************************************************/
void setInterruptLevels_H(AMG8833_Dev_t *dev, float high, float low, float hysteresis)
{
	int highConv = high / AMG88xx_TEMP_CONVERSION;
	highConv = constrain(highConv, -4095, 4095);
	write8(dev, AMG88xx_INTHL, (highConv & 0xFF));
	write8(dev, AMG88xx_INTHH, ((highConv & 0xF) >> 4));

	int lowConv = low / AMG88xx_TEMP_CONVERSION;
	lowConv = constrain(lowConv, -4095, 4095);
	write8(dev, AMG88xx_INTLL, (lowConv & 0xFF));
	write8(dev, AMG88xx_INTLH, (((lowConv & 0xF) >> 4) & 0xF));

	int hysConv = hysteresis / AMG88xx_TEMP_CONVERSION;
	hysConv = constrain(hysConv, -4095, 4095);
	write8(dev, AMG88xx_IHYSL, (hysConv & 0xFF));
	write8(dev, AMG88xx_IHYSH, (((hysConv & 0xF) >> 4) & 0xF));
}

/**************************************************************************/
/*!
 @brief  enable the interrupt pin on the device.
 */
/**************************************************************************/
bool enableInterrupt(AMG8833_Dev_t *dev)
{
	// 0 = Difference interrupt mode
	// 1 = absolute value interrupt mode
	return write8(dev, AMG88xx_INTC, (mode << 1 | 1) & 0x03);
}

/**************************************************************************/
/*!
 @brief  disable the interrupt pin on the device
 */
/**************************************************************************/
bool disableInterrupt(AMG8833_Dev_t *dev)
{
	// 0 = Difference interrupt mode
	// 1 = absolute value interrupt mode
	return write8(dev, AMG88xx_INTC, (mode << 1 | 0) & 0x03);
}

/**************************************************************************/
/*!
 @brief  Set the interrupt to either absolute value or difference mode
 @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
 */
/**************************************************************************/
void setInterruptMode(AMG8833_Dev_t *dev, uint8_t m)
{
	mode = m;
	write8(dev, AMG88xx_INTC, (mode << 1 | 1) & 0x03);
}

/**************************************************************************/
/*!
 @brief  Read the state of the triggered interrupts on the device. The full interrupt register is 8 bytes in length.
 @param  buf the pointer to where the returned data will be stored
 @param  size Optional number of bytes to read. Default is 8 bytes.
 @returns up to 8 bytes of data in buf
 */
/**************************************************************************/
void getInterrupt(AMG8833_Dev_t *dev, uint8_t *buf, uint8_t size)
{
	uint8_t bytesToRead = min(size, (uint8_t) 8);
	read(dev, AMG88xx_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************/
/*!
 @brief  Clear any triggered interrupts
 */
/**************************************************************************/
void clearInterrupt(AMG8833_Dev_t *dev)
{
	write8(dev, AMG88xx_RST, AMG88xx_FLAG_RESET);
}

/**************************************************************************/
/*!
 @brief  read the onboard thermistor
 @returns a the floating point temperature in degrees Celsius
 */
/**************************************************************************/
float readThermistor(AMG8833_Dev_t *dev)
{
	uint8_t raw[2];
	read(dev, AMG88xx_TTHL, raw, 2);
	uint16_t recast = ((uint16_t) raw[1] << 8) | ((uint16_t) raw[0]);
	return signedMag12ToFloat(recast) * AMG88xx_THRM_CONVERSION;
}

/**************************************************************************/
/*!
 @brief  Read Infrared sensor values
 @param  buf the array to place the pixels in
 @param  size Optionsl number of bytes to read (up to 64). Default is 64 bytes.
 @return up to 64 bytes of pixel data in buf
 */
/**************************************************************************/
void readPixels(AMG8833_Dev_t *dev, float *buf, uint8_t size)
{
	uint16_t recast;
	float converted;
	uint8_t bytesToRead = min((uint8_t) (size << 1), (uint8_t) (AMG88xx_PIXEL_ARRAY_SIZE << 1));
	uint8_t rawArray[bytesToRead];
	read(dev, AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);

	for (int i = 0; i < size; i++) {
		uint8_t pos = i << 1;
		recast = ((uint16_t) rawArray[pos + 1] << 8) | ((uint16_t) rawArray[pos]);
		converted = int12ToFloat(recast) * AMG88xx_TEMP_CONVERSION;
		buf[i] = converted;
	}
}

/**************************************************************************/
/*!
 @brief  write one byte of data to the specified register
 @param  reg the register to write to
 @param  value the value to write
 @returns result of the write operation
 */
/**************************************************************************/
bool write8(AMG8833_Dev_t *dev, uint16_t reg, uint8_t value)
{
	return (i2cdevWrite16(dev->I2Cx, dev->devAddr, reg, 1, &value));
}

/**************************************************************************/
/*!
 @brief  read one byte of data from the specified register
 @param  reg the register to read
 @returns one byte of register data
 */
/**************************************************************************/
uint8_t read8(AMG8833_Dev_t *dev, uint8_t reg)
{
	uint8_t ret;
	read(dev, reg, &ret, 1);
	return ret;
}

void read(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t bytesToRead)
{
	i2cdevRead(dev->I2Cx, dev->devAddr, reg, bytesToRead, buf);
}

void write(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t num)
{
	for (int i = 0; i < num; i++) {
		write8(dev, reg, buf[i]);
	}
}

/**************************************************************************/
/*!
 @brief  convert a 12-bit signed magnitude value to a floating point number
 @param  val the 12-bit signed magnitude value to be converted
 @returns the converted floating point value
 */
/**************************************************************************/
float signedMag12ToFloat(uint16_t val)
{
	// Take first 11 bits as absolute val
	uint16_t absVal = (val & 0x7FF);
	return (val & 0x800) ? 0 - (float) absVal : (float) absVal;
}

/**************************************************************************/
/*!
 @brief  convert a 12-bit integer two's complement value to a floating point number
 @param  val the 12-bit integer  two's complement value to be converted
 @returns the converted floating point value
 */
/**************************************************************************/
float int12ToFloat(uint16_t val)
{
	// Shift to left so that sign bit of 12 bit integer number is placed on
	// sign bit of 16 bit signed integer number
	int16_t sVal = (val << 4);
	// Shift back the signed number, return converts to float
	return sVal >> 4;
}

uint8_t min(uint8_t a, uint8_t b)
{
    return (a < b) ? a : b;
}
