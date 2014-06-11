/***********************************************************************
** Bosch BMP180 pressure / temperature sensor control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef __LPMS_BMP180_H
#define __LPMS_BMP180_H

#include "stm32f2xx.h"
#include "LpmsTimebase.h"

// I2c communication definitions
#define PRESSURE_IO_PORT	GPIOB
#define PRESSURE_I2C_SCL_PIN	GPIO_Pin_10
#define PRESSURE_I2C_SDA_PIN	GPIO_Pin_11
#define PRESSURE_I2C_SCL_SOURCE	GPIO_PinSource10
#define PRESSURE_I2C_SDA_SOURCE	GPIO_PinSource11
#define PRESSURE_I2C_SCL_AF	GPIO_AF_I2C2
#define PRESSURE_I2C_SDA_AF	GPIO_AF_I2C2
#define PRESSURE_I2C_PORT	I2C2
#define PRESSURE_I2C_RCC_PERIPH	RCC_APB1Periph_I2C2
#define PRESSURE_I2C_RCC_PORT	RCC_AHB1Periph_GPIOB
#define PRESSURE_I2C_ADDRESS	0xEE
#define PRESSURE_I2C_SPEED	400000
#define LPMS_I2C_ADDRESS	0xA0

// Pressur sensor power modes
#define PRESSURE_LOW_POWER_MODE		0
#define PRESSURE_STD_POWER_MODE		1
#define PRESSURE_HIGH_POWER_MODE	2
#define PRESSURE_ULTRA_HIGH_POWER_MODE	3

// Chip ID
#define BMP180_CHIP_ID	0x55

// BMP180 chip registers
#define BMP180_OUT_XLSB_REG		0xF8
#define BMP180_OUT_LSB_REG		0xF7
#define BMP180_OUT_MSB_REG		0xF6
#define BMP180_CTRL_MEAS_REG		0xF4
#define BMP180_SOFT_RESET_REG		0xE0
#define BMP180_CHIP_ID_REG		0xD0
#define BMP180_E2PROM_CALIB_START_ADDR	0xAA
#define BMP180_E2PROM_LEN		22

// BMP180 commands
#define BMP180_T_MEASURE_START			0x2E
#define BMP180_P_LOW_MEASURE_START		0x34
#define BMP180_P_STD_MEASURE_START		0x74
#define BMP180_P_HIGH_MEASURE_START		0xB4
#define BMP180_P_ULTRA_HIGH_MEASURE_START	0xF4

// BMP180 power modes
#define BMP180_LOW_POWER_MODE	0
#define BMP180_STD_MODE		1
#define BMP180_HIGH_MODE 	2
#define BMP180_ULTRA_HIGH_MODE	3

// Conversion time settings
#define BMP180_T_CONVERSION_TIME		4.5f
#define BMP180_P_LOW_CONVERSION_TIME		4.5f
#define BMP180_P_STD_CONVERSION_TIME		7.5f
#define BMP180_P_HIGH_CONVERSION_TIME		13.5f
#define BMP180_P_ALTRA_HIGH_CONVERSION_TIME	25.5
#define BMP180_P_ADVANCED_CONVERSION_TIME	76.5

// Altitude calculation constants
#define BMP180_PARAM_MG	3038
#define BMP180_PARAM_MH	(-7357)
#define BMP180_PARAM_MI	3791

#ifdef __cplusplus
extern "C" {
#endif 

// BMP180 calibration parameters
typedef struct _Bmp180CalibrationParam {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
} CalibrationParam;

// Sets BMP180 i2c bus configuration
void setPressureI2CConfig(void);

// Waits while sensor is in standby state
uint8_t waitPressureI2CStandbyState(void);

// Writes chip register
void writePressureRegister(uint8_t address, uint8_t data);

// Reads chip register
void readPressureRegister(uint8_t* pBuffer, uint8_t address);

// Initializes pressure sensor
uint8_t initPressureSensor(void);

// Retrieves calibration parameters
uint8_t getCalibParam(CalibrationParam* calibParam);

// Retrieves UT parameter
uint8_t getUT(uint16_t* uT);

// Retrievs UP parameter
uint8_t getUP(uint32_t* uP, uint8_t oss);

// Retrives temperature and pressure
uint8_t getTempAndPressure(int16_t* temp, int32_t* pressure, uint8_t oss);

#ifdef __cplusplus
}
#endif 

#endif