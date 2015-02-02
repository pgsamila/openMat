/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsBmp180.h"

CalibrationParam calib_param;

typedef enum  {GETTEMP, GETTEMPBUSY, GETPRESSURE, GETPRESSUREBUSY} BMP180States;
BMP180States BMP180_CurrentState = GETTEMP;

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
	//I2C_Cmd(PRESSURE_I2C_PORT, ENABLE);
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
	
	getCalibParam(&calib_param);
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

uint8_t isBMP180ConversionComplete(void)
{
    uint8_t conversionStatus;
    waitPressureI2CStandbyState();
	readPressureRegister(&conversionStatus, BMP180_CTRL_MEAS_REG);
	return !(conversionStatus & 0x20);
}

void sendUTStartCmd(void)
{  
    writePressureRegister(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE_START);       
}

uint8_t getUT(uint16_t* uT)
{
    uint8_t data[2];
    readPressureRegister(&data[0], BMP180_OUT_MSB_REG);
    // waitPressureI2CStandbyState();
    readPressureRegister(&data[1], BMP180_OUT_LSB_REG);

    *uT = (((uint16_t)data[0]) << 8) | (uint16_t)data[1];
	return 1;	
}

void sendUPStartCmd(uint8_t oss)
{       
	uint8_t cmd;
    switch(oss) {
    case 0:
        cmd = BMP180_P_LOW_MEASURE_START;
        //delay   = 6;
        break;
    case 1:
        cmd = BMP180_P_STD_MEASURE_START;
        //delay   = 9;
        break;
    case 2:
        cmd = BMP180_P_HIGH_MEASURE_START;
        //delay   = 15;
        break;
    case 3:
        cmd = BMP180_P_ULTRA_HIGH_MEASURE_START;
       // delay   = 27;
        break;
    }
    writePressureRegister(BMP180_CTRL_MEAS_REG,cmd);  
}

uint8_t getUP(uint32_t* uP, uint8_t oss)
{
	uint8_t data[3];
    readPressureRegister(&data[0], BMP180_OUT_MSB_REG);
    // waitPressureI2CStandbyState();
    readPressureRegister(&data[1], BMP180_OUT_LSB_REG);
    // waitPressureI2CStandbyState();
    readPressureRegister(&data[2], BMP180_OUT_XLSB_REG);

    *uP = ((((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2]) >> (8 - oss);
    return 1;
}


int16_t getTrueTemperature(uint16_t UT) {
	calib_param.b5  = (((int32_t)UT - (int32_t)calib_param.ac6) * (int32_t)calib_param.ac5) >> 15;
	calib_param.b5 += ((int32_t)calib_param.mc << 11) / (calib_param.b5 + calib_param.md);
	return (calib_param.b5 + 8) >> 4;
}

int32_t getTruePressure(uint32_t UP, uint8_t oss) {
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = calib_param.b5 - 4000;
	X3 = ((calib_param.b2 * ((B6 * B6) >> 12)) >> 11) + ((calib_param.ac2 * B6) >> 11);
	B3 = (((((int32_t)calib_param.ac1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((calib_param.ac3 * B6) >> 13) + ((calib_param.b1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (calib_param.ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}

uint8_t getTempAndPressure(int16_t* temp, int32_t* pressure, uint8_t oss)
{
    if (BMP180_CurrentState == GETTEMP)
    {
        sendUTStartCmd();
        BMP180_CurrentState = GETTEMPBUSY;
    } else if (BMP180_CurrentState == GETTEMPBUSY && isBMP180ConversionComplete())
    {           
        uint16_t uT;
        getUT(&uT); 
        *temp = getTrueTemperature(uT);        
        BMP180_CurrentState = GETPRESSURE;         
    } 
    else if (BMP180_CurrentState == GETPRESSURE)
    {
        sendUPStartCmd(oss);
        BMP180_CurrentState = GETPRESSUREBUSY;
    } else if (BMP180_CurrentState == GETPRESSUREBUSY && isBMP180ConversionComplete())
    {
        uint32_t uP;
        getUP(&uP, oss);
        *pressure = getTruePressure(uP,oss);
        BMP180_CurrentState = GETTEMP;
        return 1;
    }
    return 0;
}
