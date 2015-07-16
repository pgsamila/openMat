/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#ifndef USE_LSM6DS3

#include "LpmsL3gd20.h"

void setGyrI2CConfig(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(GYR_I2C_RCC_PERIPH, ENABLE);
	RCC_AHB1PeriphClockCmd(GYR_I2C_RCC_PORT, ENABLE);
	
	RCC_APB1PeriphResetCmd(GYR_I2C_RCC_PERIPH, ENABLE);
	
	RCC_APB1PeriphResetCmd(GYR_I2C_RCC_PERIPH, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GYR_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GYR_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GYR_I2C_SDA_PIN;
	GPIO_Init(GYR_IO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GYR_IO_PORT, GYR_I2C_SCL_SOURCE, GYR_I2C_SCL_AF);
	GPIO_PinAFConfig(GYR_IO_PORT, GYR_I2C_SDA_SOURCE, GYR_I2C_SDA_AF);

	I2C_DeInit(GYR_I2C_PORT);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = LPMS_GYR_I2C_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = GYR_I2C_SPEED;
	
	I2C_Init(GYR_I2C_PORT, &I2C_InitStructure);
	
	I2C_Cmd(GYR_I2C_PORT, ENABLE);
}
 
void waitGyrI2CStandbyState(void)      
{
	do {
		I2C_GenerateSTART(GYR_I2C_PORT, ENABLE);
		I2C_ReadRegister(GYR_I2C_PORT, I2C_Register_SR1);
		I2C_Send7bitAddress(GYR_I2C_PORT, GYR_I2C_ADDRESS, I2C_Direction_Transmitter);
	} while(!(I2C_ReadRegister(GYR_I2C_PORT, I2C_Register_SR1) & 0x0002));
	
	I2C_ClearFlag(GYR_I2C_PORT, I2C_FLAG_AF);
}

void writeGyrRegister(uint8_t address, uint8_t data)
{
	uint8_t dataBuffer[8];

	dataBuffer[0] = data;
	gyrI2cWrite(address, 1, dataBuffer);
}

int gyrI2cWrite(unsigned char reg_addr, unsigned char length, unsigned char const *data)
{	
  	uint8_t ok = 0;
	uint32_t to = 0;
	
	while (ok == 0) {
		I2C_GenerateSTART(GYR_I2C_PORT, ENABLE);
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
	
		I2C_Send7bitAddress(GYR_I2C_PORT, GYR_I2C_ADDRESS, I2C_Direction_Transmitter);
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
		
		ok = 1;	
	}

	I2C_SendData(GYR_I2C_PORT, reg_addr);	
	to = 0;
	while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && to < GYR_I2C_TIMEOUT);
	
	while (length) {
		I2C_SendData(GYR_I2C_PORT, *data);

		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		
		++data;
		--length;
	}

	I2C_GenerateSTOP(GYR_I2C_PORT, ENABLE);
		
	return 0;
}

void readGyrRegister(uint8_t* pBuffer, unsigned char length, uint8_t address)
{
	gyrI2cRead(address, length, pBuffer);
}

int gyrI2cRead(unsigned char reg_addr, unsigned char length, unsigned char *data)
{   	  
  	uint8_t ok = 0;
	uint32_t to = 0;
	
	while (ok == 0) {
		I2C_GenerateSTART(GYR_I2C_PORT, ENABLE);
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
		
		I2C_Send7bitAddress(GYR_I2C_PORT, GYR_I2C_ADDRESS, I2C_Direction_Transmitter);
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
		
		ok = 1;	
	}
	
	I2C_SendData(GYR_I2C_PORT, reg_addr);	
	to = 0;
	while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && to < GYR_I2C_TIMEOUT);
	
	I2C_GenerateSTOP(GYR_I2C_PORT, ENABLE);

	ok = 0;
	while (ok == 0) {		
		I2C_GenerateSTART(GYR_I2C_PORT, ENABLE);	
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
		
		I2C_Send7bitAddress(GYR_I2C_PORT, GYR_I2C_ADDRESS, I2C_Direction_Receiver);
		to = 0;
		while (!I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && to < GYR_I2C_TIMEOUT) ++to;
		if (to >= GYR_I2C_TIMEOUT) continue;
		
		ok = 1;	
	}

	to = 0;
	while (length && to < GYR_I2C_TIMEOUT) {
		++to;
		
		if (length == 1) {
			I2C_AcknowledgeConfig(GYR_I2C_PORT, DISABLE);
			I2C_GenerateSTOP(GYR_I2C_PORT, ENABLE);
		}

		if (I2C_CheckEvent(GYR_I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			*data = I2C_ReceiveData(GYR_I2C_PORT);
			++data;
			--length;
		} else {
		}
	}
	
	I2C_AcknowledgeConfig(GYR_I2C_PORT, ENABLE);
	
	return 0;
}

uint8_t setGyrOutputDataRateAndPowerMode(uint32_t outputDataRate, uint8_t powerMode)
{
	uint8_t i = 0;
	
	if (outputDataRate == (uint32_t)GYR_OUTPUT_DATA_RATE_95HZ) {
		i |= (uint8_t)L3GD20_ODR_95HZ_BW_25HZ;
	} else if (outputDataRate == (uint32_t)GYR_OUTPUT_DATA_RATE_190HZ) {
		i |= (uint8_t)L3GD20_ODR_190HZ_BW_70HZ;
	} else if (outputDataRate == (uint32_t)GYR_OUTPUT_DATA_RATE_380HZ) {
		i |= (uint8_t)L3GD20_ODR_380HZ_BW_100HZ;
	} else if (outputDataRate == (uint32_t)GYR_OUTPUT_DATA_RATE_760HZ) {
		i |= (uint8_t)L3GD20_ODR_760HZ_BW_100HZ;
	} else {
		return 0;
	}
	
	if (powerMode == GYR_POWER_DOWN) {
		i |= (uint8_t)L3GD20_POWER_DOWN_MODE;
	} else if (powerMode == GYR_SLEEP_MODE) {
		i |= (uint8_t)L3GD20_SLEEP_MODE;
	} else if (powerMode == GYR_NORMAL_MODE) {
		i |= (uint8_t)L3GD20_NORMAL_MODE;
	} else {
		return 0;
	}
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_CTRL_REG1, i);
	
	return 1;
}

uint8_t setGyrFullScale(uint32_t fullScale)
{
	uint8_t i = 0;
	
	i = (uint8_t)(L3GD20_BDU_AFTER_READING | L3GD20_BLE_LSB_LOWER_ADDRESS);
	
	if (fullScale == (uint32_t)GYR_RANGE_250DPS) {
		i |= (uint8_t)L3GD20_FS_250DPS;
	} else if (fullScale == (uint32_t)GYR_RANGE_500DPS) {
		i |= (uint8_t)L3GD20_FS_500DPS;
	} else if (fullScale == (uint32_t)GYR_RANGE_2000DPS) {
		i |= (uint8_t)L3GD20_FS_2000DPS;
	} else {
		return 0;
	}
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_CTRL_REG4, i);
	
	return 1;
}

uint8_t initGyr(uint32_t outputDataRate, uint8_t powerMode, uint32_t fullScale)
{
	uint8_t result = 0;
	
	setGyrI2CConfig();
	
	result = setGyrOutputDataRateAndPowerMode(outputDataRate, powerMode);
	if (result == 0) {
		return 0;
	}
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_CTRL_REG2, L3GD20_HPM_NORMAL_RESET | L3GD20_HPCF_7_14_27_51HZ);
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_CTRL_REG3, L3GD20_ALL_INTERRUPT_DISABLE);
	
	result = setGyrFullScale(fullScale);
	if (result == 0) {
		return 0;
	}
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_CTRL_REG5, L3GD20_FIFO_DISABLE | L3GD20_HPEN_DISABLE | L3GD20_OUTSEL_LPF1);
	
	waitGyrI2CStandbyState();
	writeGyrRegister(L3GD20_FIFO_CTRL_REG, L3GD20_FM_BYPASS_MODE);
	
	return 1;
}

uint8_t isGyrDataReady(void)
{
 	uint8_t is_data_ready = 0;	
	
	readGyrRegister(&is_data_ready, 1, L3GD20_STATUS_REG);
	if ((is_data_ready & 0x08) == 0) return 0;
	
	return 1;
}

uint8_t isGyrXDataReady(void)
{
 	uint8_t is_data_ready = 0;	
	
	readGyrRegister(&is_data_ready, 1, L3GD20_STATUS_REG);
	if ((is_data_ready & 0x01) == 0) return 0;
	
	return 1;
}

uint8_t isGyrYDataReady(void)
{
 	uint8_t is_data_ready = 0;	
	
	readGyrRegister(&is_data_ready, 1, L3GD20_STATUS_REG);
	if ((is_data_ready & 0x02) == 0) return 0;
	
	return 1;
}

uint8_t isGyrZDataReady(void)
{
 	uint8_t is_data_ready = 0;	
	
	readGyrRegister(&is_data_ready, 1, L3GD20_STATUS_REG);
	if ((is_data_ready & 0x04) == 0) return 0;
	
	return 1;
}

uint8_t getGyrRawData(float* xAxis, float* yAxis, float* zAxis)
{
	uint8_t data_buffer[6];
	int16_t temp;	
	
	waitGyrI2CStandbyState();
	readGyrRegister(&data_buffer[0], 6, (uint8_t)L3GD20_OUT_X_L|0x80); 
	
#ifdef USE_LPMSCU_NEW
	temp = data_buffer[1];
	temp = (temp << 8) | data_buffer[0];
	*xAxis = -(float) temp;

	temp = data_buffer[3];
	temp = (temp << 8) | data_buffer[2];
	*yAxis = (float) temp;

	temp = data_buffer[5];
	temp = (temp << 8) | data_buffer[4];
	*zAxis = -(float)  temp;
#else
	temp = data_buffer[3];
	temp = (temp << 8) | data_buffer[2];
	*xAxis = -(float) temp;

	temp = data_buffer[1];
	temp = (temp << 8) | data_buffer[0];
	*yAxis = (float) temp;

	temp = data_buffer[5];
	temp = (temp << 8) | data_buffer[4];
	*zAxis =(float) temp;
#endif
	
	return 1;	
}

uint8_t getGyrTempData(int8_t* temp)
{
	uint8_t data;
	
	waitGyrI2CStandbyState();
	readGyrRegister(&data, 1, (uint8_t)L3GD20_OUT_TEMP);
	
	*temp = (int8_t)(data);
	
	return 1;
}

#endif