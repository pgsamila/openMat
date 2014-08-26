/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsBmp180.h"

void setPressureI2CConfig(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(PRESSURE_I2C_RCC_PERIPH, ENABLE);
	RCC_AHB1PeriphClockCmd(PRESSURE_I2C_RCC_PORT, ENABLE);
	
	RCC_APB1PeriphResetCmd(PRESSURE_I2C_RCC_PERIPH, ENABLE);
	RCC_APB1PeriphResetCmd(PRESSURE_I2C_RCC_PERIPH, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin =  PRESSURE_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(PRESSURE_IO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  PRESSURE_I2C_SDA_PIN;
	GPIO_Init(PRESSURE_IO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(PRESSURE_IO_PORT, PRESSURE_I2C_SCL_SOURCE, PRESSURE_I2C_SCL_AF);
	GPIO_PinAFConfig(PRESSURE_IO_PORT, PRESSURE_I2C_SDA_SOURCE, PRESSURE_I2C_SDA_AF);

	I2C_DeInit(PRESSURE_I2C_PORT);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = LPMS_I2C_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = PRESSURE_I2C_SPEED;
	
	I2C_Init(PRESSURE_I2C_PORT, &I2C_InitStructure);
	I2C_Cmd(PRESSURE_I2C_PORT, ENABLE);
}

uint8_t waitPressureI2CStandbyState(void)      
{
	startTimeoutCounting();
	do {
		I2C_GenerateSTART(PRESSURE_I2C_PORT, ENABLE);
		I2C_ReadRegister(PRESSURE_I2C_PORT, I2C_Register_SR1);
		I2C_Send7bitAddress(PRESSURE_I2C_PORT, PRESSURE_I2C_ADDRESS, I2C_Direction_Transmitter);
		
		if (getTimeout() > I2C_TIMEOUT_S) {
			return 0;
		}	
	} while (!(I2C_ReadRegister(PRESSURE_I2C_PORT, I2C_Register_SR1) & 0x0002));
	
	I2C_ClearFlag(PRESSURE_I2C_PORT, I2C_FLAG_AF);

	return 1;
}

void writePressureRegister(uint8_t address, uint8_t data)
{
	I2C_GenerateSTART (PRESSURE_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT) );
	I2C_Send7bitAddress(PRESSURE_I2C_PORT, PRESSURE_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Cmd(PRESSURE_I2C_PORT, ENABLE);
	I2C_SendData(PRESSURE_I2C_PORT, address);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(PRESSURE_I2C_PORT, data);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(PRESSURE_I2C_PORT, ENABLE);
}

void readPressureRegister(uint8_t* pBuffer, uint8_t address)
{
	I2C_GenerateSTART(PRESSURE_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(PRESSURE_I2C_PORT, PRESSURE_I2C_ADDRESS, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Cmd(PRESSURE_I2C_PORT, ENABLE);
	I2C_SendData(PRESSURE_I2C_PORT, address);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(PRESSURE_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(PRESSURE_I2C_PORT, PRESSURE_I2C_ADDRESS, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_Cmd(PRESSURE_I2C_PORT, ENABLE);	
	I2C_AcknowledgeConfig(PRESSURE_I2C_PORT, DISABLE);
	I2C_GenerateSTOP(PRESSURE_I2C_PORT, ENABLE);
	while(!I2C_CheckEvent(PRESSURE_I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED));
	*pBuffer = I2C_ReceiveData(PRESSURE_I2C_PORT);
	I2C_AcknowledgeConfig(PRESSURE_I2C_PORT, ENABLE);
}

uint8_t initPressureSensor(void)
{
	uint8_t result;
	
	setPressureI2CConfig();
	
	if (waitPressureI2CStandbyState() == 0) return 0;

	writePressureRegister(BMP180_CTRL_MEAS_REG, 0x00);
	waitPressureI2CStandbyState();
	writePressureRegister(BMP180_SOFT_RESET_REG, 0x00);
	waitPressureI2CStandbyState();
	writePressureRegister(BMP180_CTRL_MEAS_REG, 0x00);
	waitPressureI2CStandbyState();
	readPressureRegister(&result, BMP180_CHIP_ID_REG);

	if (result != BMP180_CHIP_ID) return 0;
	
	return 1;
}

uint8_t getCalibParam(CalibrationParam* calibParam)
{
	uint8_t data[BMP180_E2PROM_LEN];
		
	for (uint8_t i = 0; i < (BMP180_E2PROM_LEN); i++) {
		waitPressureI2CStandbyState();
		readPressureRegister(&data[i], BMP180_E2PROM_CALIB_START_ADDR + i);
	}
	
	calibParam->ac1 = (data[0] << 8) | data[1];
	calibParam->ac2 = (data[2] << 8) | data[3];
	calibParam->ac3 = (data[4] << 8) | data[5];
	calibParam->ac4 = (data[6] << 8) | data[7];
	calibParam->ac5 = (data[8] << 8) | data[9];
	calibParam->ac6 = (data[10] << 8) | data[11];
	calibParam->b1 = (data[12] << 8) | data[13];
	calibParam->b2 = (data[14] << 8) | data[15];
	calibParam->mb = (data[16] << 8) | data[17];
	calibParam->mc = (data[18] << 8) | data[19];
	calibParam->md = (data[20] << 8) | data[21];
	
	return 1;
}

uint8_t getUT(uint16_t* uT)
{
	uint8_t data[2];
	uint8_t isDataReady = 0;
	uint8_t conversionStatus;
	
	waitPressureI2CStandbyState();
	writePressureRegister(BMP180_CTRL_MEAS_REG, BMP180_T_MEASURE_START);
	
	while (isDataReady == 0) {
		waitPressureI2CStandbyState();
		readPressureRegister(&conversionStatus, BMP180_CTRL_MEAS_REG);
		if (!(conversionStatus & 0x20)) {
			isDataReady = 1;
			waitPressureI2CStandbyState();
			readPressureRegister(&data[0], BMP180_OUT_MSB_REG);
			waitPressureI2CStandbyState();
			readPressureRegister(&data[1], BMP180_OUT_LSB_REG);
			
			*uT = (((uint16_t)data[0]) << 8) | (uint16_t)data[1];
		}
	}
	
	return 1;	
}

uint8_t getUP(uint32_t* uP, uint8_t oss)
{
	uint8_t data[3];
	uint8_t isDataReady = 0;
	uint8_t conversionStatus;
	
	switch (oss) {
	case BMP180_LOW_POWER_MODE:
		waitPressureI2CStandbyState();
		writePressureRegister(BMP180_CTRL_MEAS_REG, BMP180_P_LOW_MEASURE_START);
	break;
	
	case BMP180_STD_MODE:
		waitPressureI2CStandbyState();
		writePressureRegister(BMP180_CTRL_MEAS_REG, BMP180_P_STD_MEASURE_START);
	break;
	
	case BMP180_HIGH_MODE:
		waitPressureI2CStandbyState();
		writePressureRegister(BMP180_CTRL_MEAS_REG, BMP180_P_HIGH_MEASURE_START);
	break;
	
	case BMP180_ULTRA_HIGH_MODE:
		waitPressureI2CStandbyState();
		writePressureRegister(BMP180_CTRL_MEAS_REG, BMP180_P_ULTRA_HIGH_MEASURE_START);
	break;
	
	default:
		return 0;
	break;
	}
	
	while (isDataReady == 0) {
		waitPressureI2CStandbyState();
		readPressureRegister(&conversionStatus, BMP180_CTRL_MEAS_REG);
		if (!(conversionStatus & 0x20)) {
			isDataReady = 1;
			waitPressureI2CStandbyState();
			readPressureRegister(&data[0], BMP180_OUT_MSB_REG);
			waitPressureI2CStandbyState();
			readPressureRegister(&data[1], BMP180_OUT_LSB_REG);
			waitPressureI2CStandbyState();
			readPressureRegister(&data[2], BMP180_OUT_XLSB_REG);
			
			*uP = ((((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2]) >> (8 - oss);
		}
	}
	
	return 1;
}

uint8_t getTempAndPressure(int16_t* temp, int32_t* pressure, uint8_t oss)
{
	int32_t x1, x2, x3, b3, b5, b6;
	uint32_t b4, b7;
	CalibrationParam calib_param;
	uint16_t uT;
	uint32_t uP;
	int32_t result;
	
	getCalibParam(&calib_param);
	getUT(&uT);

#ifdef ENABLE_WATCHDOG
	WWDG_Enable(0x7F);
#endif
	if (!getUP(&uP, oss)) {
		return 0;
	}
#ifdef ENABLE_WATCHDOG
	WWDG_DeInit();
#endif
	
	x1 = (((int32_t)uT - (int32_t)calib_param.ac6) * (int32_t)calib_param.ac5) >> 15;
	x2 = (((int32_t)calib_param.mc) << 11) / (x1 + calib_param.md);
	b5 = x1 + x2;

	*temp = (b5 + 8) >> 4; 
	
	b6 = b5 - 4000;

	x1 = (b6 * b6) >> 12;
	x1 *= calib_param.b2;
	x1 >>= 11;

	x2 = calib_param.ac2 * b6;
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((int32_t)calib_param.ac1) * 4 + x3) << oss) + 2) >> 2;

	x1 = (((int32_t)calib_param.ac3) * b6) >> 13;
	x2 = (((int32_t)calib_param.b1) * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (calib_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t)(uP - b3)) * (50000 >> oss);

	if (b7 < 0x80000000) {
		result = (b7 << 1) / b4;
	} else {
		result = (b7 / b4) << 1;
	}

	x1 = result >> 8;
	x1 *= x1;
	x1 = (x1 * BMP180_PARAM_MG) >> 16;
	x2 = (result * BMP180_PARAM_MH) >> 16;
	result = result + ((x1 + x2 + BMP180_PARAM_MI) >> 4);

	*pressure = result;
	
	return 1;
}
