/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsLsm303dlhc.h"

void setAccMagI2CConfig(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(ACC_MAG_I2C_RCC_PERIPH, ENABLE);
	RCC_AHB1PeriphClockCmd(ACC_MAG_I2C_RCC_PORT, ENABLE);
	
	RCC_APB1PeriphResetCmd(ACC_MAG_I2C_RCC_PERIPH, ENABLE);
	RCC_APB1PeriphResetCmd(ACC_MAG_I2C_RCC_PERIPH, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin =  ACC_MAG_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(ACC_MAG_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  ACC_MAG_I2C_SDA_PIN;
	GPIO_Init(ACC_MAG_IO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(ACC_MAG_IO_PORT, ACC_MAG_I2C_SCL_SOURCE, ACC_MAG_I2C_SCL_AF);
	GPIO_PinAFConfig(ACC_MAG_IO_PORT, ACC_MAG_I2C_SDA_SOURCE, ACC_MAG_I2C_SDA_AF);

	I2C_DeInit(ACC_MAG_I2C_PORT);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = LPMS_ACC_MAG_I2C_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ACC_MAG_I2C_SPEED;
	
	I2C_Init(ACC_MAG_I2C_PORT, &I2C_InitStructure);
	
	I2C_Cmd(ACC_MAG_I2C_PORT, ENABLE);
}
 
void waitAccI2CStandbyState(void)      
{	
	do {
		I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
		I2C_ReadRegister(ACC_MAG_I2C_PORT, I2C_Register_SR1);
		I2C_Send7bitAddress(ACC_MAG_I2C_PORT, ACC_I2C_ADDRESS, I2C_Direction_Transmitter);
	} while(!(I2C_ReadRegister(ACC_MAG_I2C_PORT, I2C_Register_SR1) & 0x0001));
	
	I2C_ClearFlag(ACC_MAG_I2C_PORT, I2C_FLAG_AF);
}

void waitMagI2CStandbyState(void)      
{	
	do {
		I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
		I2C_ReadRegister(ACC_MAG_I2C_PORT, I2C_Register_SR1);
		I2C_Send7bitAddress(ACC_MAG_I2C_PORT, MAG_I2C_ADDRESS, I2C_Direction_Transmitter);
	} while(!(I2C_ReadRegister(ACC_MAG_I2C_PORT, I2C_Register_SR1) & 0x0001));
	
	I2C_ClearFlag(ACC_MAG_I2C_PORT, I2C_FLAG_AF);
}

void writeAccRegister(uint8_t address, uint8_t data)
{
	I2C_GenerateSTART (ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, ACC_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(ACC_MAG_I2C_PORT, address);
	while(! I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(ACC_MAG_I2C_PORT, data);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(ACC_MAG_I2C_PORT, ENABLE);
}

void writeMagRegister(uint8_t address, uint8_t data)
{
	I2C_GenerateSTART (ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT) );
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, MAG_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(ACC_MAG_I2C_PORT, address);
	while(! I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(ACC_MAG_I2C_PORT, data);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(ACC_MAG_I2C_PORT, ENABLE);
}

void readAccRegister(uint8_t* pBuffer, uint8_t address)
{
	I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, ACC_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Cmd(ACC_MAG_I2C_PORT, ENABLE);
	I2C_SendData(ACC_MAG_I2C_PORT, address);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, ACC_I2C_ADDRESS, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_AcknowledgeConfig(ACC_MAG_I2C_PORT, DISABLE);
	I2C_GenerateSTOP(ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED));
	*pBuffer = I2C_ReceiveData(ACC_MAG_I2C_PORT);
	I2C_AcknowledgeConfig(ACC_MAG_I2C_PORT, ENABLE);
}

void readMagRegister(uint8_t* pBuffer, uint8_t address, uint8_t NumByteToRead)
{
	I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, MAG_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Cmd(ACC_MAG_I2C_PORT, ENABLE);
	I2C_SendData(ACC_MAG_I2C_PORT, address);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(ACC_MAG_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(ACC_MAG_I2C_PORT, MAG_I2C_ADDRESS, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while (NumByteToRead) {
		if (NumByteToRead == 1) {
			I2C_AcknowledgeConfig(ACC_MAG_I2C_PORT, DISABLE);
			I2C_GenerateSTOP(ACC_MAG_I2C_PORT, ENABLE);
		}

		if (I2C_CheckEvent(ACC_MAG_I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			*pBuffer = I2C_ReceiveData(ACC_MAG_I2C_PORT);
			pBuffer++;
			NumByteToRead--;			
		}
	}
	
	I2C_AcknowledgeConfig(ACC_MAG_I2C_PORT, ENABLE);
}

uint8_t setAccOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode)
{
	uint8_t i = 0;
	
	i = (uint8_t)LSM303DLHC_ACC_XYZ_ENABLED;
	
	if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_0HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_0HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_1HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_1HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_10HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_10HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_25HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_25HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_50HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_50HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_100HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_100HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_200HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_200HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_400HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_400HZ;
	} else if (outputDataRate == (uint32_t)ACC_OUTPUT_DATA_RATE_1344HZ) {
		i |= (uint8_t)LSM303DLHC_ACC_ODR_1344HZ_5376HZ;
	} else {
		return 0;
	}
	
	if (powerMode == ACC_NORMAL_POWER_MODE) {
		i |= (uint8_t)LSM303DLHC_ACC_NORMAL_POWER_MODE;
	} else if (powerMode == ACC_LOW_POWER_MODE) {
		i |= (uint8_t)LSM303DLHC_ACC_LOW_POWER_MODE;
	} else {
		return 0;
	}
	
	writeAccRegister(LSM303DLHC_CTRL_REG1_A, i);
	
	return 1;
}

uint8_t setAccFullScale(uint8_t fullScale)
{
	uint8_t i = 0;
	
	i = (uint8_t)(LSM303DLHC_ACC_BDU_AFTER_READING | LSM303DLHC_ACC_BLE_LSB | LSM303DLHC_ACC_HR_ENABLED);
	
	if (fullScale == (uint8_t)ACC_RANGE_2G) {
		i |= (uint8_t)LSM303DLHC_ACC_FS_2G;
	} else if (fullScale == (uint8_t)ACC_RANGE_4G) {
		i |= (uint8_t)LSM303DLHC_ACC_FS_4G;
	} else if (fullScale == (uint8_t)ACC_RANGE_8G) {
		i |= (uint8_t)LSM303DLHC_ACC_FS_8G;
	} else if (fullScale == (uint8_t)ACC_RANGE_16G) {
		i |= (uint8_t)LSM303DLHC_ACC_FS_16G;
	} else {
		return 0;
	}
	
	writeAccRegister(LSM303DLHC_CTRL_REG4_A, i);
	
	return 1;
}

uint8_t setMagOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode)
{
	uint8_t i = 0;
	
	i = (uint8_t)LSM303DLHC_TEMP_ENABLED;
	
	if (outputDataRate == MAG_OUTPUT_DATA_RATE_1HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_075HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_2HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_1_5HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_3HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_3HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_8HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_7_5HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_15HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_15HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_30HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_30HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_75HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_75HZ;
	} else if (outputDataRate == MAG_OUTPUT_DATA_RATE_220HZ) {
		i |= (uint8_t)LSM303DLHC_MAG_ODR_220HZ;
	} else {
		return 0;
	}
	
	writeMagRegister(LSM303DLHC_CRA_REG_M, i);
	
	i = 0;
	if (powerMode == MAG_NORMAL_POWER_MODE) {
		i |= (uint8_t)LSM303DLHC_MAG_MD_CONTINUOUS;
	} else if (powerMode == MAG_SLEEP_POWER_MODE) {
		i |= (uint8_t)LSM303DLHC_MAG_MD_SLEEP;
	} else {
		return 0;
	}
	
	writeMagRegister(LSM303DLHC_MR_REG_M, i);
	
	return 1;
}

uint8_t setMagFullScale(uint32_t fullScale)
{
	uint8_t i = 0;
	
	if (fullScale == (uint32_t)MAG_RANGE_130UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_130UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_190UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_190UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_250UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_250UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_400UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_400UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_470UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_470UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_560UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_560UT;
	} else if (fullScale == (uint32_t)MAG_RANGE_810UT) {
		i |= (uint8_t)LSM303DLHC_MAG_FS_810UT;
	} else {
		return 0;
	}
	
	writeMagRegister(LSM303DLHC_CRB_REG_M, i);
	
	return 1;
}

uint8_t initAcc(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale)
{
	uint8_t result = 0;
	
	setAccMagI2CConfig();
	
	result = setAccOutputDataRateAndPowerMode(outputDataRate, powerMode);
	if (result == 0) {
		return 0;
	}
	
	writeAccRegister(LSM303DLHC_CTRL_REG2_A, LSM303DLHC_ACC_HPM_NORMAL | LSM303DLHC_ACC_FDS_BYPASSED | LSM303DLHC_ACC_HPCLICK_DISABLED | LSM303DLHC_ACC_HPIS_DISABLED);
	writeAccRegister(LSM303DLHC_CTRL_REG3_A, LSM303DLHC_ACC_ALLINT_DISABLED);
	
	result = setAccFullScale(fullScale);
	if (result == 0) {
		return 0;
	}
	
	writeAccRegister(LSM303DLHC_CTRL_REG5_A, LSM303DLHC_ACC_FIFO_DISABLED);
	writeAccRegister(LSM303DLHC_CTRL_REG6_A, LSM303DLHC_ACC_PAD2_DISABLED);
	writeAccRegister(LSM303DLHC_FIFO_CTRL_REG_A, LSM303DLHC_ACC_FIFO_MODE_BYPASSED);
	writeAccRegister(LSM303DLHC_INT1_SOURCE_A, LSM303DLHC_INT1_SRC_DEFAULT);
	writeAccRegister(LSM303DLHC_INT1_THS_A, LSM303DLHC_INT1_THS_DEFAULT);
	writeAccRegister(LSM303DLHC_INT1_DURATION_A, LSM303DLHC_INT1_DURATION_DEFAULT);
	writeAccRegister(LSM303DLHC_INT2_SOURCE_A, LSM303DLHC_INT2_SRC_DEFAULT);
	writeAccRegister(LSM303DLHC_INT2_THS_A, LSM303DLHC_INT2_THS_DEFAULT);	
	writeAccRegister(LSM303DLHC_INT2_DURATION_A, LSM303DLHC_INT2_DURATION_DEFAULT);
	
	return 1;
}

uint8_t initMag(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale)
{
	uint8_t result = 0;
	
	setAccMagI2CConfig();
	
	result = setMagOutputDataRateAndPowerMode(outputDataRate, powerMode);
	if (result == 0) {
		return 0;
	}
	
	result = setMagFullScale(fullScale);
	if (result == 0) {
		return 0;
	}
	
	return 1;
}

uint8_t getAccRawData(int16_t* xAxis, int16_t* yAxis, int16_t* zAxis)
{
	uint8_t data_buffer[6];
	
	/* readAccRegister(&data_buffer[0], (uint8_t)LSM303DLHC_OUT_X_L_A);      
	readAccRegister(&data_buffer[1], (uint8_t)LSM303DLHC_OUT_X_H_A);
	readAccRegister(&data_buffer[2], (uint8_t)LSM303DLHC_OUT_Y_L_A);
	readAccRegister(&data_buffer[3], (uint8_t)LSM303DLHC_OUT_Y_H_A);
	readAccRegister(&data_buffer[4], (uint8_t)LSM303DLHC_OUT_Z_L_A);
	readAccRegister(&data_buffer[5], (uint8_t)LSM303DLHC_OUT_Z_H_A); */

#ifdef USE_LPMSCU_NEW
	*xAxis = (int16_t)((((int16_t)data_buffer[1]) << 8) + data_buffer[0]);
	*yAxis = -(int16_t)((((int16_t)data_buffer[3]) << 8) + data_buffer[2]);
	*zAxis = (int16_t)((((int16_t)data_buffer[5]) << 8) + data_buffer[4]);
#else
        *xAxis = (int16_t)((((int16_t)data_buffer[1]) << 8) + data_buffer[0]);
	*yAxis = (int16_t)((((int16_t)data_buffer[3]) << 8) + data_buffer[2]);
	*zAxis = -(int16_t)((((int16_t)data_buffer[5]) << 8) + data_buffer[4]);
#endif
	
	return 1;	
}

uint8_t isAccDataReady(void) 
{
	uint8_t is_data_ready = 0;
	
	readAccRegister(&is_data_ready, (uint8_t)LSM303DLHC_STATUS_REG_A);
	if ((is_data_ready & 0x08) == 0) {
		return 0;  
	}

	return 1;
}	

uint8_t getMagRawData(int16_t* xAxis, int16_t* yAxis, int16_t* zAxis)
{
	uint8_t data_buffer[6];
		
	readMagRegister(data_buffer, (uint8_t)LSM303DLHC_OUT_X_H_M, 6);    
        
#ifdef USE_LPMSCU_NEW
	*xAxis = -(int16_t)((((int16_t)data_buffer[0]) << 8) + data_buffer[1]);
	*zAxis = -(int16_t)((((int16_t)data_buffer[2]) << 8) + data_buffer[3]);
	*yAxis = (int16_t)((((int16_t)data_buffer[4]) << 8) + data_buffer[5]);
#else
        *xAxis = -(int16_t)((((int16_t)data_buffer[0]) << 8) + data_buffer[1]);
	*zAxis = (int16_t)((((int16_t)data_buffer[2]) << 8) + data_buffer[3]);
	*yAxis = -(int16_t)((((int16_t)data_buffer[4]) << 8) + data_buffer[5]);
#endif
	return 1;
}

uint8_t isMagDataReady(void)
{
 	uint8_t is_data_ready = 0;	
	
	readMagRegister(&is_data_ready, LSM303DLHC_SR_REG_M, 1);
	if ((is_data_ready & 0x01) == 0) {
		return 0;  
	}
	
	return 1;
}

uint8_t getAccMagTempData(int16_t* temp)
{
	uint8_t data[2];

	readMagRegister(&data[0], (uint8_t)LSM303DLHC_TEMP_OUT_H_M, 1);
	readMagRegister(&data[1], (uint8_t)LSM303DLHC_TEMP_OUT_L_M, 1);
	
	*temp = (int16_t)(((uint16_t)data[0] << 4) + (data[1] >> 4));
	
	return 1;
}