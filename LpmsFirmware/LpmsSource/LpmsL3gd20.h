/***********************************************************************
** L3GD20 gyroscope control
**
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef __LPMS_L3GD20_H
#define __LPMS_L3GD20_H

#include "stm32f2xx.h"
#include "LpmsTimebase.h"

// Gyroscope i2c definitions
#define GYR_IO_PORT				GPIOB
#define GYR_I2C_SCL_PIN			GPIO_Pin_10
#define GYR_I2C_SDA_PIN			GPIO_Pin_11
#define GYR_I2C_SCL_SOURCE		GPIO_PinSource10
#define GYR_I2C_SDA_SOURCE		GPIO_PinSource11
#define GYR_I2C_SCL_AF			GPIO_AF_I2C2
#define GYR_I2C_SDA_AF			GPIO_AF_I2C2
#define GYR_I2C_PORT			I2C2
#define GYR_I2C_RCC_PERIPH		RCC_APB1Periph_I2C2
#define GYR_I2C_RCC_PORT		RCC_AHB1Periph_GPIOB
#define GYR_I2C_ADDRESS			0xD4
#define GYR_I2C_SPEED			400000
#define LPMS_GYR_I2C_ADDRESS	0xA0

// Gyroscope ranges
#define GYR_RANGE_250DPS	250
#define GYR_RANGE_500DPS	500
#define GYR_RANGE_2000DPS	2000

// Gyroscope gains
#define GYR_GAIN_250DPS		0.00875f
#define GYR_GAIN_500DPS		0.0175f
#define GYR_GAIN_2000DPS	0.070f

// Gyroscope data rates
#define GYR_OUTPUT_DATA_RATE_95HZ	95
#define GYR_OUTPUT_DATA_RATE_190HZ 	190
#define GYR_OUTPUT_DATA_RATE_380HZ 	380
#define GYR_OUTPUT_DATA_RATE_760HZ	760

// Gyroscope opration modes
#define GYR_POWER_DOWN	0
#define GYR_SLEEP_MODE	1
#define GYR_NORMAL_MODE	2

// Gyroscope temperature gain
#define GYR_TEMPERATURE_GAIN	(-1)

// Gyroscope register settings
#define L3GD20_CTRL_REG1		0x20
#define L3GD20_CTRL_REG2		0x21
#define L3GD20_CTRL_REG3		0x22
#define L3GD20_CTRL_REG4		0x23
#define L3GD20_CTRL_REG5		0x24
#define L3GD20_REFERENCE		0x25
#define L3GD20_OUT_TEMP			0x26
#define L3GD20_STATUS_REG		0x27
#define L3GD20_OUT_X_L			0x28
#define L3GD20_OUT_X_H			0x29
#define L3GD20_OUT_Y_L			0x2A
#define L3GD20_OUT_Y_H			0x2B
#define L3GD20_OUT_Z_L			0x2C
#define L3GD20_OUT_Z_H			0x2D
#define L3GD20_FIFO_CTRL_REG	0x2E
#define L3GD20_FIFO_SRC_REG		0x2F
#define L3GD20_INT1_CFG			0x30
#define L3GD20_INT1_SRC			0x31
#define L3GD20_INT1_TSH_XH		0x32
#define L3GD20_INT1_TSH_XL		0x33
#define L3GD20_INT1_TSH_YH		0x34
#define L3GD20_INT1_TSH_YL		0x35
#define L3GD20_INT1_TSH_ZH		0x36
#define L3GD20_INT1_TSH_ZL		0x37
#define L3GD20_INT1_DURATION	0x38
#define L3GD20_ODR_95HZ_BW_12_5HZ	0x00
#define L3GD20_ODR_95HZ_BW_25HZ		0x10
#define L3GD20_ODR_190HZ_BW_12_5HZ	0x40
#define L3GD20_ODR_190HZ_BW_25HZ	0x50
#define L3GD20_ODR_190HZ_BW_50HZ	0x60
#define L3GD20_ODR_190HZ_BW_70HZ	0x70
#define L3GD20_ODR_380HZ_BW_20HZ	0x80
#define L3GD20_ODR_380HZ_BW_25HZ	0x90
#define L3GD20_ODR_380HZ_BW_50HZ	0xA0
#define L3GD20_ODR_380HZ_BW_100HZ	0xB0
#define L3GD20_ODR_760HZ_BW_30HZ	0xC0
#define L3GD20_ODR_760HZ_BW_35HZ	0xD0
#define L3GD20_ODR_760HZ_BW_50HZ	0xE0
#define L3GD20_ODR_760HZ_BW_100HZ	0xF0
#define L3GD20_POWER_DOWN_MODE		0x00
#define L3GD20_SLEEP_MODE			0x08
#define L3GD20_NORMAL_MODE			0x0F
#define L3GD20_HPM_NORMAL_RESET		0x00
#define L3GD20_HPM_REF				0x10
#define L3GD20_HPM_NORMAL			0x20
#define L3GD20_HPM_AUTORESET		0x30
#define L3GD20_HPCF_7_14_27_51HZ	0x00
#define L3GD20_HPCF_4_7_14_27HZ		0x01
#define L3GD20_HPCF_2_4_7_14		0x02
#define L3GD20_HPCF_09_2_4_7		0x03
#define L3GD20_HPCF_045_09_2_4		0x04
#define L3GD20_HPCF_018_045_09_2	0x05
#define L3GD20_HPCF_009_018_045_09	0x06
#define L3GD20_HPCF_0045_009_018_045	0x07
#define L3GD20_HPCF_0018_0045_009_018	0x08
#define L3GD20_HPCF_0009_0018_0045_009	0x09
#define L3GD20_ALL_INTERRUPT_DISABLE	0x00
#define L3GD20_ALL_INTERRUPT_ENABLE		0xEF
#define L3GD20_BDU_CONTINUOUS			0x00
#define L3GD20_BDU_AFTER_READING		0x80
#define L3GD20_BLE_LSB_LOWER_ADDRESS	0x00
#define L3GD20_BLE_MSB_LOWER_ADDRESS	0x40
#define L3GD20_FS_250DPS	0x00
#define L3GD20_FS_500DPS	0x10
#define L3GD20_FS_2000DPS	0x20
#define L3GD20_FIFO_DISABLE	0x00
#define L3GD20_FIFO_ENABLE	0x40
#define L3GD20_HPEN_DISABLE	0x00
#define L3GD20_HPEN_ENABLE	0x10
#define L3GD20_OUTSEL_LPF1		0x00
#define L3GD20_OUTSEL_LPF1_HPF	0x01
#define L3GD20_OUTSEL_LPF2		0x02
#define L3GD20_FM_BYPASS_MODE			0x00
#define L3GD20_FM_FIFO_MODE				0x20
#define L3GD20_FM_STREAM_MODE			0x40
#define L3GD20_FM_STREAM_TO_FIFO_MODE	0x60
#define L3GD20_FM_BYPASS_TO_STREAM_MODE	0x80

#ifdef __cplusplus
extern "C" {
#endif 

// Sets i2c bus configuration
void setGyrI2CConfig(void);

// Waits while i2c bus is in standby state
void waitGyrI2CStandbyState(void);

// Writes gyroscope register
void writeGyrRegister(uint8_t address, uint8_t data);

// Reads gyroscope register
void readGyrRegister(uint8_t* pBuffer, uint8_t address);

// Sets gyroscope output data rate and powerMode
uint8_t setGyroOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode);

// Sets gyroscope range
uint8_t setGyrFullScale(uint32_t fullScale);

// Initializes gyrsocope
uint8_t initGyr(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale);

// Retrieves gyrsocope data rate
uint8_t getGyrRawData(int16_t* xAxis, int16_t* yAxix, int16_t* zAxis);

// Retrieves gyroscope raw data buffer
uint8_t getGyrRawDataBuffer(uint8_t *data_buffer);

// Retrieves gyroscope temperature
uint8_t getGyrTempData(int8_t* temp);

// Checks if gyroscope data is ready
uint8_t isGyrDataReady(void);

#ifdef __cplusplus
}
#endif 

#endif