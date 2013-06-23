/***********************************************************************
** LSM303DLHC accelerometer and magnetometer control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_LSM303DLHC_H
#define LPMS_LSM303DLHC_H

#include "stm32f2xx.h"
#include "LpmsTimebase.h"

// Accelerometer/magnetometer i2c bus settings
#define ACC_MAG_IO_PORT				GPIOB
#define ACC_MAG_I2C_SCL_PIN			GPIO_Pin_6
#define ACC_MAG_I2C_SDA_PIN			GPIO_Pin_7
#define ACC_MAG_I2C_SCL_SOURCE		GPIO_PinSource6
#define ACC_MAG_I2C_SDA_SOURCE		GPIO_PinSource7
#define ACC_MAG_I2C_SCL_AF			GPIO_AF_I2C1
#define ACC_MAG_I2C_SDA_AF			GPIO_AF_I2C1
#define ACC_MAG_I2C_PORT			I2C1
#define ACC_MAG_I2C_RCC_PERIPH		RCC_APB1Periph_I2C1
#define ACC_MAG_I2C_RCC_PORT		RCC_AHB1Periph_GPIOB
#define ACC_I2C_ADDRESS				0x32
#define MAG_I2C_ADDRESS				0x3C
#define ACC_MAG_I2C_SPEED			400000
#define LPMS_ACC_MAG_I2C_ADDRESS	0xA0

// Accelerometer ranges
#define ACC_RANGE_2G	2
#define ACC_RANGE_4G	4
#define ACC_RANGE_8G	8
#define ACC_RANGE_16G	16

//  Accelerometer gains
#define ACC_GAIN_2G	(1.0f/16384.0f)
#define ACC_GAIN_4G	(1.0f/8192.0f)
#define ACC_GAIN_8G	(1.0f/4096.0f)
#define ACC_GAIN_16G	(1.0f/1365.3333333333333333333333333333f)

// Accelerometer output data rate
#define ACC_OUTPUT_DATA_RATE_0HZ	0
#define ACC_OUTPUT_DATA_RATE_1HZ	1
#define ACC_OUTPUT_DATA_RATE_10HZ	10
#define ACC_OUTPUT_DATA_RATE_25HZ	25
#define ACC_OUTPUT_DATA_RATE_50HZ	50
#define ACC_OUTPUT_DATA_RATE_100HZ	100
#define ACC_OUTPUT_DATA_RATE_200HZ	200
#define ACC_OUTPUT_DATA_RATE_400HZ	400
#define ACC_OUTPUT_DATA_RATE_1344HZ	1344

// Accelerometer power modes
#define ACC_NORMAL_POWER_MODE	0
#define ACC_LOW_POWER_MODE	1

// Magnetometer ranges
#define MAG_RANGE_130UT	130
#define MAG_RANGE_190UT	190
#define MAG_RANGE_250UT	250
#define MAG_RANGE_400UT	400
#define MAG_RANGE_470UT	470
#define MAG_RANGE_560UT	560
#define MAG_RANGE_810UT	810

// Magnetometer gains
#define MAG_GAIN_XY_130UT	(100.0f/1100.0f)
#define MAG_GAIN_Z_130UT	(100.0f/980.0f)
#define MAG_GAIN_XY_190UT	(100.0f/855.0f)
#define MAG_GAIN_Z_190UT	(100.0f/760.0f)
#define MAG_GAIN_XY_250UT	(100.0f/670.0f)
#define MAG_GAIN_Z_250UT	(100.0f/600.0f)
#define MAG_GAIN_XY_400UT	(100.0f/450.0f)
#define MAG_GAIN_Z_400UT	(100.0f/400.0f)
#define MAG_GAIN_XY_470UT	(100.0f/400.0f)
#define MAG_GAIN_Z_470UT	(100.0f/355.0f)
#define MAG_GAIN_XY_560UT	(100.0f/330.0f)
#define MAG_GAIN_Z_560UT	(100.0f/295.0f)
#define MAG_GAIN_XY_810UT	(100.0f/230.0f)
#define MAG_GAIN_Z_810UT	(100.0f/205.0f)

// Magnetometer output data rates
#define MAG_OUTPUT_DATA_RATE_1HZ	1
#define MAG_OUTPUT_DATA_RATE_2HZ	2
#define MAG_OUTPUT_DATA_RATE_3HZ	3
#define MAG_OUTPUT_DATA_RATE_8HZ	8
#define MAG_OUTPUT_DATA_RATE_15HZ	15
#define MAG_OUTPUT_DATA_RATE_30HZ	30
#define MAG_OUTPUT_DATA_RATE_75HZ	75
#define MAG_OUTPUT_DATA_RATE_220HZ	220

// Magnetometer power modes
#define MAG_NORMAL_POWER_MODE	0
#define MAG_SLEEP_POWER_MODE	1

// Accelerometer temperature gain
#define ACC_MAG_TEMPERATURE_GAIN	(1/8)

// Accelerometer/magnetometer control registers
#define LSM303DLHC_CTRL_REG1_A		0x20		
#define LSM303DLHC_CTRL_REG2_A		0x21	        
#define LSM303DLHC_CTRL_REG3_A		0x22	        
#define LSM303DLHC_CTRL_REG4_A		0x23	        
#define LSM303DLHC_CTRL_REG5_A		0x24	        
#define LSM303DLHC_CTRL_REG6_A		0x25	        
#define LSM303DLHC_REFERENCE_A		0x26
#define LSM303DLHC_STATUS_REG_A		0x27
#define LSM303DLHC_OUT_X_L_A		0x28
#define LSM303DLHC_OUT_X_H_A		0x29
#define LSM303DLHC_OUT_Y_L_A		0x2A
#define LSM303DLHC_OUT_Y_H_A		0x2B
#define LSM303DLHC_OUT_Z_L_A		0x2C
#define LSM303DLHC_OUT_Z_H_A		0x2D
#define LSM303DLHC_FIFO_CTRL_REG_A	0x2E
#define LSM303DLHC_FIFO_SRC_REG_A	0x2F
#define LSM303DLHC_INT1_CFG_A		0x30
#define LSM303DLHC_INT1_SOURCE_A	0x31
#define LSM303DLHC_INT1_THS_A		0x32
#define LSM303DLHC_INT1_DURATION_A	0x33
#define LSM303DLHC_INT2_CFG_A		0x34
#define LSM303DLHC_INT2_SOURCE_A	0x35
#define LSM303DLHC_INT2_THS_A		0x36
#define LSM303DLHC_INT2_DURATION_A	0x37
#define LSM303DLHC_CLICK_CFG_A		0x38
#define LSM303DLHC_CLICK_SRC_A		0x39
#define LSM303DLHC_CLICK_THS_A		0x3A
#define LSM303DLHC_TIME_LIMIT_A		0x3B
#define LSM303DLHC_TIME_LATENCY_A	0x3C
#define LSM303DLHC_TIME_WINDOW_A	0x3D
#define LSM303DLHC_CRA_REG_M		0x00
#define LSM303DLHC_CRB_REG_M		0x01
#define LSM303DLHC_MR_REG_M			0x02
#define LSM303DLHC_OUT_X_H_M		0x03
#define LSM303DLHC_OUT_X_L_M		0x04
#define LSM303DLHC_OUT_Z_H_M		0x05
#define LSM303DLHC_OUT_Z_L_M		0x06
#define LSM303DLHC_OUT_Y_H_M		0x07
#define LSM303DLHC_OUT_Y_L_M		0x08
#define LSM303DLHC_SR_REG_M			0x09
#define LSM303DLHC_IRA_REG_M		0x0A
#define LSM303DLHC_IRB_REG_M		0x0B
#define LSM303DLHC_IRC_REG_M		0x0C
#define LSM303DLHC_TEMP_OUT_H_M		0x31
#define LSM303DLHC_TEMP_OUT_L_M		0x32
#define LSM303DLHC_ACC_ODR_0HZ		0x00
#define LSM303DLHC_ACC_ODR_1HZ		0x10
#define LSM303DLHC_ACC_ODR_10HZ		0x20
#define LSM303DLHC_ACC_ODR_25HZ		0x30
#define LSM303DLHC_ACC_ODR_50HZ		0x40
#define LSM303DLHC_ACC_ODR_100HZ	0x50
#define LSM303DLHC_ACC_ODR_200HZ	0x60
#define LSM303DLHC_ACC_ODR_400HZ	0x70
#define LSM303DLHC_ACC_ODR_1620HZ_LOW		0x80
#define LSM303DLHC_ACC_ODR_1344HZ_5376HZ	0x90
#define LSM303DLHC_ACC_NORMAL_POWER_MODE	0x00
#define LSM303DLHC_ACC_LOW_POWER_MODE		0x08
#define LSM303DLHC_ACC_XYZ_ENABLED			0x07
#define LSM303DLHC_ACC_HPM_NORMAL_RESET		0x00
#define LSM303DLHC_ACC_HPM_REF				0x40
#define LSM303DLHC_ACC_HPM_NORMAL			0x80
#define LSM303DLHC_ACC_HPM_AUTORESET		0xC0
#define LSM303DLHC_ACC_FDS_BYPASSED			0x00
#define LSM303DLHC_ACC_FDS_FILTER			0x08
#define LSM303DLHC_ACC_HPCLICK_DISABLED		0x00
#define LSM303DLHC_ACC_HPCLICK_ENABLED		0x04
#define LSM303DLHC_ACC_HPIS_DISABLED		0x00
#define LSM303DLHC_ACC_HPIS_ENABLED			0x03
#define LSM303DLHC_ACC_ALLINT_DISABLED		0x00
#define LSM303DLHC_ACC_ALLINT_ENABLED		0xFF
#define LSM303DLHC_ACC_BDU_CONTINUOUS		0x00
#define LSM303DLHC_ACC_BDU_AFTER_READING	0x80
#define LSM303DLHC_ACC_BLE_LSB				0x00
#define LSM303DLHC_ACC_BLE_MSB				0x40
#define LSM303DLHC_ACC_FS_2G				0x00
#define LSM303DLHC_ACC_FS_4G				0x10
#define LSM303DLHC_ACC_FS_8G				0x20	
#define LSM303DLHC_ACC_FS_16G				0x30
#define LSM303DLHC_ACC_HR_DISABLED			0x00
#define LSM303DLHC_ACC_HR_ENABLED			0x08
#define LSM303DLHC_ACC_FIFO_DISABLED		0x00
#define LSM303DLHC_ACC_FIFO_ENABLED			0x40
#define LSM303DLHC_ACC_PAD2_DISABLED		0x00
#define LSM303DLHC_ACC_PAD2_ENABLED			0xFF
#define LSM303DLHC_ACC_FIFO_MODE_BYPASSED	0x00
#define LSM303DLHC_ACC_FIFO_MODE_FIFO		0x40
#define LSM303DLHC_ACC_FIFO_MODE_STREAM		0x80
#define LSM303DLHC_ACC_FIFO_MODE_TRIGGER	0xC0
#define LSM303DLHC_INT1_SRC_DEFAULT			0x00
#define LSM303DLHC_INT1_THS_DEFAULT			0x00
#define LSM303DLHC_INT1_DURATION_DEFAULT	0x00
#define LSM303DLHC_INT2_SRC_DEFAULT			0x00
#define LSM303DLHC_INT2_THS_DEFAULT			0x00
#define LSM303DLHC_INT2_DURATION_DEFAULT	0x00
#define LSM303DLHC_TEMP_DISABLED			0x00
#define LSM303DLHC_TEMP_ENABLED				0x80
#define LSM303DLHC_MAG_ODR_075HZ			0x00
#define LSM303DLHC_MAG_ODR_1_5HZ			0x04
#define LSM303DLHC_MAG_ODR_3HZ				0x08
#define LSM303DLHC_MAG_ODR_7_5HZ			0x0C
#define LSM303DLHC_MAG_ODR_15HZ				0x10
#define LSM303DLHC_MAG_ODR_30HZ				0x14
#define LSM303DLHC_MAG_ODR_75HZ				0x18
#define LSM303DLHC_MAG_ODR_220HZ			0x1C
#define LSM303DLHC_MAG_FS_130UT				0x20
#define LSM303DLHC_MAG_FS_190UT				0x40
#define LSM303DLHC_MAG_FS_250UT				0x60
#define LSM303DLHC_MAG_FS_400UT				0x80
#define LSM303DLHC_MAG_FS_470UT				0xA0
#define LSM303DLHC_MAG_FS_560UT				0xC0
#define LSM303DLHC_MAG_FS_810UT				0xE0
#define LSM303DLHC_MAG_MD_CONTINUOUS		0x00
#define LSM303DLHC_MAG_MD_SINGLE			0x01
#define LSM303DLHC_MAG_MD_SLEEP				0x02

#ifdef __cplusplus
extern "C" {
#endif 

// Sets i2c bus configuration
void setAccMagI2CConfig(void);

// Waits accelerometer i2c stand-by state
void waitAccI2CStandbyState(void);

// Waits for magnetometer i2c stand-by state
void waitMagI2CStandbyState(void);

// Writes accelerometer register
void writeAccRegister(uint8_t address, uint8_t data);

// Writes magnetometer register
void writeMagRegister(uint8_t address, uint8_t data);

// Reads accelerometer register
void readAccRegister(uint8_t* pBuffer, uint8_t address);

// Reads magnetometer register
void readMagRegister(uint8_t* pBuffer, uint8_t address, uint8_t NumByteToRead);

// Sets accelerometer output data rate and power mode
uint8_t setAccOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode);

// Sets accelerometer range
uint8_t setAccFullScale(uint8_t fullScale);

// Sets magnetometer output data rate and power mode
uint8_t setMagOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode);

// Sets magnetometer range
uint8_t setMagFullScale(uint32_t fullScale);

// Initializes accelerometer
uint8_t initAcc(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale);

// Initializes magnetometer
uint8_t initMag(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale);

// Retrieves accelerometer raw data
uint8_t getAccRawData(int16_t* xAxis, int16_t* yAxix, int16_t* zAxis);

// Retrieves magnetometer raw data
uint8_t getMagRawData(int16_t* xAxis, int16_t* yAxix, int16_t* zAxis);

// Retrieves temeperature data
uint8_t getAccMagTempData(int16_t* temp);

// Checks if accelerometer is ready
uint8_t isAccDataReady(void);

// Checks if magnetometer is ready
uint8_t isMagDataReady(void);

#ifdef __cplusplus
}
#endif 

#endif