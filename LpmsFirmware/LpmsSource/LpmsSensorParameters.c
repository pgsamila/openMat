/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsSensorParameters.h"

LpmsReg gReg;
uint8_t isSelfTestOn = 0;

extern LpFilterParameters lpFilterParam;
extern LpmsCalibrationData calibrationData;
extern LpVector3f a;
extern LpVector3f b;
extern LpVector3f g;
extern LpVector3f w;
extern LpVector3f gRaw;
extern LpVector4f q;
extern LpVector4f qOffset;
extern LpVector4f qAfterOffset;
extern LpVector3f r;
extern LpVector3f rAfterOffset;
extern LpVector3f linAcc;
extern float magNoise;
extern uint32_t lpmsStatus;
extern uint32_t lpmsStatus;
extern float pressure;
extern float temperature;
extern float heaveY;
extern float measurementTime;

uint8_t writeRegisters(void)
{
	writeCompleteRegisterSet();

	return 1;
}

uint16_t getImuID(void)
{
	return ((uint16_t) gReg.data[LPMS_IMU_ID]);
}

void setImuID(uint16_t id)
{
	gReg.data[LPMS_IMU_ID] = (uint32_t) id;
}

void updateConfigRegToSensorManager(void)
{	
	if ((gReg.data[LPMS_CONFIG] & LPMS_GYR_THRES_ENABLED) != 0) {
		lpFilterParam.useGyrThreshold = 1;
	} else {
		lpFilterParam.useGyrThreshold = 0;
	}
}

uint8_t setBaudrate(uint8_t* data)
{
	/* uint32_t baudrate = 0;
	
	for (uint8_t i = 0; i < 4; i++) {
		baudrate = baudrate | (((uint32_t) data[i]) << (i * 8));
	}
	
	if (baudrate == LPMS_BT_BAUDRATE_9600 || baudrate == LPMS_CANBUS_BAUDRATE_10K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_9600_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_19200 || baudrate == LPMS_CANBUS_BAUDRATE_10K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_19200_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_38400 || baudrate == LPMS_CANBUS_BAUDRATE_50K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_38400_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_57600 || baudrate == LPMS_CANBUS_BAUDRATE_125K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_57600_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_115200 || baudrate == LPMS_CANBUS_BAUDRATE_250K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_115200_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_230400 || baudrate == LPMS_CANBUS_BAUDRATE_500K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_230400_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_460800 || baudrate == LPMS_CANBUS_BAUDRATE_800K) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) | LPMS_BT_BAUDRATE_460800_ENABLED;
	} else if (baudrate == LPMS_BT_BAUDRATE_921600 || baudrate == LPMS_CANBUS_BAUDRATE_1M) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_BAUDRATE_MASK)) |  LPMS_BT_BAUDRATE_921600_ENABLED;
	} else {
		return 0;
	} */
		
	return 1;
}

uint8_t setCanBaudrate(uint8_t* data)
{
	uint32_t baudrate = getUi32t(data);
	
	switch (baudrate) {
	case LPMS_CAN_BAUDRATE_125K:
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_CAN_BAUDRATE_MASK)) | LPMS_CAN_BAUDRATE_125K_ENABLED;		
	break;

	case LPMS_CAN_BAUDRATE_250K:
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_CAN_BAUDRATE_MASK)) | LPMS_CAN_BAUDRATE_250K_ENABLED;		
	break;

	case LPMS_CAN_BAUDRATE_500K:
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_CAN_BAUDRATE_MASK)) | LPMS_CAN_BAUDRATE_500K_ENABLED;		
	break;

	case LPMS_CAN_BAUDRATE_1M:
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_CAN_BAUDRATE_MASK)) | LPMS_CAN_BAUDRATE_1M_ENABLED;		
	break;
	}

	return 1;
}

uint8_t setCanHeartbeat(uint8_t* data)
{
  	uint32_t v = getUi32t(data);
	gReg.data[LPMS_CAN_HEARTBEAT] = v;
	updateCanHeartbeat();

	return 0;
}

uint8_t getCanHeartbeat(uint8_t* data, uint16_t *l)
{
	setUi32t(&(data[0]), gReg.data[LPMS_CAN_HEARTBEAT]);

	*l = 4;

	return 0;
}

void updateCanHeartbeat(void)
{
	switch (gReg.data[LPMS_CAN_HEARTBEAT]) {
	case LPMS_CAN_HEARTBEAT_005:
		calibrationData.canHeartbeatTiming = 0.5;
	break;

	case LPMS_CAN_HEARTBEAT_010:
		calibrationData.canHeartbeatTiming = 1.0;
	break;

	case LPMS_CAN_HEARTBEAT_020:
		calibrationData.canHeartbeatTiming = 2.0;
	break;

	case LPMS_CAN_HEARTBEAT_050:
		calibrationData.canHeartbeatTiming = 5.0;
	break;

	case LPMS_CAN_HEARTBEAT_100:
		calibrationData.canHeartbeatTiming = 10.0;
	break;
	}
}

uint8_t setCanMapping(uint8_t* data)
{   
	uint32_t v[16];
	int i;

	getMultiUi32t(data, 16, v);

	for (i=0; i<16; ++i) {
		gReg.data[LPMS_CAN_MAPPING+i] = v[i];
	}

	updateCanMapping();

	return 0;
}

uint8_t getCanMapping(uint8_t* data, uint16_t *l)
{       
	int i;
	uint32_t v[16];

	for (i=0; i<16; ++i) {
		v[i] = gReg.data[LPMS_CAN_MAPPING+i];
	}

	setMultiUi32t(&(data[0]), v, 16);

	*l = 64;

	return 0;
}

void updateCanMapping(void)
{                                
	int i;

	for (i=0; i<16; ++i) {
		calibrationData.canMapping[i] = gReg.data[LPMS_CAN_MAPPING+i];
	}
}

uint8_t getCanConfiguration(uint8_t* data, uint16_t *l)
{    
	setUi32t(&(data[0]), gReg.data[LPMS_CAN_CONFIGURATION]);
	*l = 4;

	return 1;
}

uint8_t setCanChannelMode(uint8_t* data)
{                
	uint32_t v = getUi32t(data);

	switch(v) {
	case LPMS_CAN_SEQUENTIAL_MODE:     
		 gReg.data[LPMS_CAN_CONFIGURATION] |= LPMS_CAN_SEQUENTIAL_MODE;
	break;

	default:
		gReg.data[LPMS_CAN_CONFIGURATION] &= ~LPMS_CAN_SEQUENTIAL_MODE;
	break;
	}

	return 1;
}

uint8_t setCanPointMode(uint8_t* data)
{
	uint32_t v = getUi32t(data);

	switch(v) {
	case LPMS_CAN_FIXEDPOINT_MODE:     
		 gReg.data[LPMS_CAN_CONFIGURATION] |= LPMS_CAN_FIXEDPOINT_MODE;
	break;

	default:
		gReg.data[LPMS_CAN_CONFIGURATION] &= ~LPMS_CAN_FIXEDPOINT_MODE;
	break;
	}

	return 1;
}

uint8_t setCanStartId(uint8_t* data)
{
	uint32_t v = getUi32t(data);
	
	gReg.data[LPMS_CAN_CONFIGURATION] &= ~0xffff0000;
	gReg.data[LPMS_CAN_CONFIGURATION] |= (v & 0x0000ffff) << 16;

	return 1;
}

uint8_t setStreamFreq(uint8_t* data)
{
	uint32_t freq = 0;
	
	for (uint8_t i = 0; i < 4; i++) {
		freq = freq | (((uint32_t) data[i]) << (i * 8));
	}
	
	if (freq == (uint32_t)LPMS_STREAM_FREQ_5HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_5HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_10HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_10HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_30HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_30HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_50HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_50HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_100HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_100HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_200HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_200HZ_ENABLED;
	} else if (freq == (uint32_t)LPMS_STREAM_FREQ_500HZ) {
		gReg.data[LPMS_CONFIG] = (gReg.data[LPMS_CONFIG] & (~LPMS_STREAM_FREQ_MASK)) | LPMS_STREAM_FREQ_500HZ_ENABLED;
	} else {
		return 0;
	}

	initTimebase();
		
	return 1;
}

uint8_t setOrientationOffset(uint8_t* data)
{
	qOffset.data[0] = 1.0f;
	qOffset.data[1] = 0.0f;
	qOffset.data[2] = 0.0f;
	qOffset.data[3] = 0.0f;

  	quaternionInv(&q, &qOffset);

	setRegVector4f(LPMS_OFFSET_QUAT_0, qOffset);
  
  	return 1;
}

uint8_t resetOrientationOffset(uint8_t* data)
{
	qOffset.data[0] = 1.0f;
	qOffset.data[1] = 0.0f;
	qOffset.data[2] = 0.0f;
	qOffset.data[3] = 0.0f;

	setRegVector4f(LPMS_OFFSET_QUAT_0, qOffset);
  
  	return 1;
}

uint8_t setGyrRange(uint8_t* data)
{
	switch (getUi32t(data)) {
	case GYR_RANGE_250DPS:
		calibrationData.gyrRange = GYR_RANGE_250DPS;
		vect3x1SetScalar(&calibrationData.gyrGain, GYR_GAIN_250DPS);
		break;

	case GYR_RANGE_500DPS:
		calibrationData.gyrRange = GYR_RANGE_500DPS;
		vect3x1SetScalar(&calibrationData.gyrGain, GYR_GAIN_500DPS);
	break;

	case GYR_RANGE_2000DPS:
		calibrationData.gyrRange = GYR_RANGE_2000DPS;
		vect3x1SetScalar(&calibrationData.gyrGain, GYR_GAIN_2000DPS);
	break;
	
	default:
		return 0;
	break;
	}
	
	setGyrFullScale(calibrationData.gyrRange);
	setRegUInt32(LPMS_GYR_RANGE, calibrationData.gyrRange);
	setRegVector3f(LPMS_GYR_GAIN_X, calibrationData.gyrGain);
	
	return 1;
}

uint8_t setGyrOutputRate(uint8_t* data)
{
	switch (getUi32t(data)) {
	case GYR_OUTPUT_DATA_RATE_95HZ:
	case GYR_OUTPUT_DATA_RATE_190HZ:
	case GYR_OUTPUT_DATA_RATE_380HZ:
	case GYR_OUTPUT_DATA_RATE_760HZ:
		calibrationData.gyrOutputRate = getUi32t(data);
	break;
	
	default:
		return 0;
	break;
	}
	
	setRegUInt32(LPMS_GYR_OUTPUT_RATE, calibrationData.gyrOutputRate);
	
	return 1;
}

uint8_t setGyrBias(uint8_t* data)
{
	calibrationData.gyrOffset = getVector3f(data);
	
	if (setRegVector3f(LPMS_GYR_BIAS_X, calibrationData.gyrOffset) == 0) return 0;
	
	return 1;
}

uint8_t getGyrAlignMatrix(uint8_t* data, uint16_t *l)
{
	setFloatMatrix3x3f(data, 0, calibrationData.gyrAlignment);

	*l = 36;

	return 1;
}

uint8_t getGyrAlignBias(uint8_t* data, uint16_t *l)
{
	setFloatVector3f(data, 0, calibrationData.gyrAlignOffset);
	*l = 12;

	return 1;
}

uint8_t setGyrAlignMatrix(uint8_t* data)
{
	calibrationData.gyrAlignment = getMatrix3x3f(data);
	setRegMatrix3x3f(LPMS_GYR_ALIG_00, calibrationData.gyrAlignment);

	return 1;
}

uint8_t setGyrAlignBias(uint8_t* data)
{
	calibrationData.gyrAlignOffset = getVector3f(data);
	setRegVector3f(LPMS_GYR_ALIG_BIAS_X, calibrationData.gyrAlignOffset);

	return 1;
}

uint8_t setEnableGyrThresh(uint8_t* data) 
{
  	uint8_t f = 1;
	
	if (getUi32t(data) == LPMS_ENABLE_GYR_THRESHOLD) {
		gReg.data[LPMS_CONFIG] |= LPMS_GYR_THRES_ENABLED;
		lpFilterParam.useGyrThreshold = 1;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_GYR_THRES_ENABLED;
		lpFilterParam.useGyrThreshold = 0;	
	}
	  
	return f;
}

uint8_t setEnableGyrAutoCal(uint8_t* data) 
{
  	uint8_t f = 1;
	
	if (getUi32t(data) == LPMS_ENABLE_GYR_AUTOCAL) {
		gReg.data[LPMS_CONFIG] |= LPMS_GYR_AUTOCAL_ENABLED;
		lpFilterParam.useGyrAutoCal = 1;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_GYR_AUTOCAL_ENABLED;
		lpFilterParam.useGyrAutoCal = 0;	
	}
	  
	return f;
}

uint8_t setAccRange(uint8_t* data)
{	
	switch (getUi32t(data)) {
	case ACC_RANGE_2G:
		calibrationData.accRange = ACC_RANGE_2G;
		vect3x1SetScalar(&calibrationData.accGain, ACC_GAIN_2G);
	break;

	case ACC_RANGE_4G:
		calibrationData.accRange = ACC_RANGE_4G;
		vect3x1SetScalar(&calibrationData.accGain, ACC_GAIN_4G);
	break;

	case ACC_RANGE_8G:
		calibrationData.accRange = ACC_RANGE_8G;
		vect3x1SetScalar(&calibrationData.accGain, ACC_GAIN_8G);
	break;

	case ACC_RANGE_16G:
		calibrationData.accRange = ACC_RANGE_16G;
		vect3x1SetScalar(&calibrationData.accGain, ACC_GAIN_16G);
	break;
	
	default:
		return 0;
	break;
	}
	
	setAccFullScale(calibrationData.accRange);
	setRegUInt32(LPMS_ACC_RANGE, calibrationData.accRange);
	setRegVector3f(LPMS_ACC_GAIN_X, calibrationData.accGain);
	
	return 1;
}

uint8_t setAccOutputRate(uint8_t* data)
{
	switch (getUi32t(data)) {
	case ACC_OUTPUT_DATA_RATE_0HZ:	
	case ACC_OUTPUT_DATA_RATE_1HZ:
	case ACC_OUTPUT_DATA_RATE_10HZ:
	case ACC_OUTPUT_DATA_RATE_25HZ:
	case ACC_OUTPUT_DATA_RATE_50HZ:	
	case ACC_OUTPUT_DATA_RATE_100HZ:
	case ACC_OUTPUT_DATA_RATE_200HZ:
	case ACC_OUTPUT_DATA_RATE_400HZ:
	case ACC_OUTPUT_DATA_RATE_1344HZ:
		calibrationData.accOutputRate = getUi32t(data);
	break;
	
	default:
		return 0;
	break;
	}
	
	setRegUInt32(LPMS_ACC_OUTPUT_RATE, calibrationData.accOutputRate); 

	return 1;
}

uint8_t setAccBias(uint8_t* data)
{
	calibrationData.accOffset = getVector3f(data);
	if (setRegVector3f(LPMS_ACC_BIAS_X, calibrationData.accOffset) == 0) return 0;
	
	return 1;
}

uint8_t setAccOffset(uint8_t* data) 
{
	calibrationData.accOffset = getVector3f(data);
	setRegVector3f(LPMS_ACC_BIAS_X, calibrationData.accOffset);

	return 1;
}

uint8_t getAccOffset(uint8_t* data, uint16_t *l) 
{
	setFloatVector3f(data, 0, calibrationData.accOffset);

	*l = 12;

	return 1;
}

uint8_t setAccAlignMatrix(uint8_t* data)
{	
	calibrationData.accAlignment = getMatrix3x3f(data);
	setRegMatrix3x3f(LPMS_ACC_ALIG_00, calibrationData.accAlignment);

	return 1;
}

uint8_t getAccAlignMatrix(uint8_t* data, uint16_t *l)
{
	setFloatMatrix3x3f(data, 0, calibrationData.accAlignment);

	*l = 36;

	return 1;
}

uint8_t setAccCovar(uint8_t* data) 
{
  	uint8_t f = 1;  
  
  	lpFilterParam.accCovar = getFloat(data);
	f = setRegFloat(LPMS_ACC_COVAR_USER, lpFilterParam.accCovar);	
  
	setCovariances();
	setReferences();

	return f;
}

uint8_t setMagRange(uint8_t* data)
{
	switch (getUi32t(data)) {
	case MAG_RANGE_130UT:
		calibrationData.magRange = MAG_RANGE_130UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_130UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_130UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_130UT;
	break;

	case MAG_RANGE_190UT:
		calibrationData.magRange = MAG_RANGE_190UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_190UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_190UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_190UT;
	break;

	case MAG_RANGE_250UT:
		calibrationData.magRange = MAG_RANGE_250UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_250UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_250UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_250UT;
	break;

	case MAG_RANGE_400UT:
		calibrationData.magRange = MAG_RANGE_400UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_400UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_400UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_400UT;
	break;
	
	case MAG_RANGE_560UT:
		calibrationData.magRange = MAG_RANGE_560UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_560UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_560UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_560UT;
	break;

	case MAG_RANGE_810UT:
		calibrationData.magRange = MAG_RANGE_810UT;
		calibrationData.magGain.data[0] = MAG_GAIN_XY_810UT;
		calibrationData.magGain.data[1] = MAG_GAIN_XY_810UT;
		calibrationData.magGain.data[2] = MAG_GAIN_Z_810UT;
	break;	
	
	default:
		return 0;
	break;
	}
	
	setMagFullScale(calibrationData.magRange);
	setRegUInt32(LPMS_MAG_RANGE, calibrationData.magRange);
	setRegVector3f(LPMS_MAG_GAIN_X, calibrationData.magGain);
	
	return 1;
}

uint8_t setMagOutputRate(uint8_t* data)
{
	switch (getUi32t(data)) {
	case MAG_OUTPUT_DATA_RATE_1HZ:	
	case MAG_OUTPUT_DATA_RATE_2HZ:
	case MAG_OUTPUT_DATA_RATE_3HZ:
	case MAG_OUTPUT_DATA_RATE_8HZ:
	case MAG_OUTPUT_DATA_RATE_15HZ:
	case MAG_OUTPUT_DATA_RATE_30HZ:
	case MAG_OUTPUT_DATA_RATE_75HZ:
	case MAG_OUTPUT_DATA_RATE_220HZ:
		calibrationData.magOutputRate = getUi32t(data);
	break;	
	
	default:
		return 0;
	break;
	}
	
	setRegUInt32(LPMS_MAG_OUTPUT_RATE, calibrationData.magOutputRate);
	
	return 1;
}

uint8_t setHardIronOffsetData(uint8_t* data) 
{
	LpVector3f f = getVector3f(data);
	setHardIronOffset(f);
	setRegVector3f(LPMS_MAG_BIAS_X, f);

	return 1;
}

uint8_t getHardIronOffsetData(uint8_t* data, uint16_t *l) 
{
	LpVector3f f;

	f = getHardIronOffset();
	setFloatVector3f(data, 0, f);

	*l = 12;

	return 1;
}

uint8_t setSoftIronMatrixData(uint8_t* data) 
{
	LpMatrix3x3f M = getMatrix3x3f(data);
	setSoftIronMatrix(M);
	setRegMatrix3x3f(LPMS_MAG_SOFT_00, M);

	return 1;
}

uint8_t getSoftIronMatrixData(uint8_t* data, uint16_t *l) 
{
	LpMatrix3x3f M;

	M = getSoftIronMatrix();
	setFloatMatrix3x3f(data, 0, M);

	*l = 36;

	return 1;
}

void updateMagAlignMatrix(void)
{
	getRegMatrix3x3f(LPMS_MAG_ALIG_00, &(calibrationData.magAlignmentMatrix));
}

void setDefaultAlignMatrix(void)
{
	gReg.data[LPMS_MAG_ALIG_00] = conFtoI(LPMS_FACTORY_MAG_ALIG_00);
	gReg.data[LPMS_MAG_ALIG_01] = conFtoI(LPMS_FACTORY_MAG_ALIG_01);
	gReg.data[LPMS_MAG_ALIG_02] = conFtoI(LPMS_FACTORY_MAG_ALIG_02);
	gReg.data[LPMS_MAG_ALIG_10] = conFtoI(LPMS_FACTORY_MAG_ALIG_10);
	gReg.data[LPMS_MAG_ALIG_11] = conFtoI(LPMS_FACTORY_MAG_ALIG_11);
	gReg.data[LPMS_MAG_ALIG_12] = conFtoI(LPMS_FACTORY_MAG_ALIG_12);
	gReg.data[LPMS_MAG_ALIG_20] = conFtoI(LPMS_FACTORY_MAG_ALIG_20);
	gReg.data[LPMS_MAG_ALIG_21] = conFtoI(LPMS_FACTORY_MAG_ALIG_21);
	gReg.data[LPMS_MAG_ALIG_22] = conFtoI(LPMS_FACTORY_MAG_ALIG_22);
}

void setDefaultAlignBias(void)
{
	gReg.data[LPMS_MAG_ALIG_BIAS_X] = conFtoI(LPMS_FACTORY_MAG_ALIG_BIAS_X);
	gReg.data[LPMS_MAG_ALIG_BIAS_Y] = conFtoI(LPMS_FACTORY_MAG_ALIG_BIAS_Y);
	gReg.data[LPMS_MAG_ALIG_BIAS_Z] = conFtoI(LPMS_FACTORY_MAG_ALIG_BIAS_Z);
}

void updateMagAlignBias(void)
{
	getRegVector3f(LPMS_MAG_ALIG_BIAS_X, &(calibrationData.magAlignmentOffset));
}

void updateMagReference(void)
{
	getRegVector3f(LPMS_MAG_REF_X, &(lpFilterParam.magRef));
}

uint8_t setMagAlignMatrix(uint8_t* data)
{
	calibrationData.magAlignmentMatrix = getMatrix3x3f(data);
	setRegMatrix3x3f(LPMS_MAG_ALIG_00, calibrationData.magAlignmentMatrix);

	return 1;
}

uint8_t setMagAlignBias(uint8_t* data)
{
	calibrationData.magAlignmentOffset = getVector3f(data);
	setRegVector3f(LPMS_MAG_ALIG_BIAS_X, calibrationData.magAlignmentOffset);

	return 1;
}

void setDefaultMagReference(void)
{
  	gReg.data[LPMS_MAG_REF_X] = conFtoI(LPMS_FACTORY_MAG_REF_X);
	gReg.data[LPMS_MAG_REF_Y] = conFtoI(LPMS_FACTORY_MAG_REF_Y);
	gReg.data[LPMS_MAG_REF_Z] = conFtoI(LPMS_FACTORY_MAG_REF_Z);
}

uint8_t setMagReference(uint8_t* data)
{
	lpFilterParam.magRef = getVector3f(data);
	setRegVector3f(LPMS_MAG_REF_X, lpFilterParam.magRef);

	return 1;
}

uint8_t getMagAlignMatrix(uint8_t* data, uint16_t *l)
{
	setFloatMatrix3x3f(data, 0, calibrationData.magAlignmentMatrix);

	*l = 36;

	return 1;
}

uint8_t getMagAlignBias(uint8_t* data, uint16_t *l)
{
	setFloatVector3f(data, 0, calibrationData.magAlignmentOffset);

	*l = 12;

	return 1;
}

uint8_t getMagReference(uint8_t* data, uint16_t *l)
{
	setFloatVector3f(data, 0, lpFilterParam.magRef);

	*l = 12;

	return 1;
}

uint8_t setFieldEstimateData(uint8_t* data)
{	
	lpFilterParam.magFieldEstimate = getFloat(data);
	gReg.data[LPMS_MAG_FIELD_EST] = conFtoI(lpFilterParam.magFieldEstimate);
	
	return 1;
}

uint8_t getFieldEstimateData(uint8_t* data, uint16_t *l)
{
	setFloat(data, lpFilterParam.magFieldEstimate, FLOAT_FULL_PRECISION);

	*l = 4;

	return 1;
}

uint8_t setMagCovar(uint8_t* data) 
{
   	uint8_t f = 1; 
  
  	lpFilterParam.magCovar = getFloat(data);  
	f = setRegFloat(LPMS_MAG_COVAR_USER, lpFilterParam.magCovar);
  
	setCovariances();
	setReferences();

	return f;
}

uint8_t setTransmitData(uint8_t* data) 
{
	uint32_t v = getUi32t(data);
	
	if ((v & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_GYR_RAW_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_GYR_RAW_OUTPUT_ENABLED;
	}
	
	if ((v & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_ACC_RAW_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_ACC_RAW_OUTPUT_ENABLED;
	}

	if ((v & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_MAG_RAW_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_MAG_RAW_OUTPUT_ENABLED;
	}

	if ((v & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_QUAT_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_QUAT_OUTPUT_ENABLED;
	}
	
	if ((v & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
		gReg.data[LPMS_CONFIG] |= LPMS_EULER_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_EULER_OUTPUT_ENABLED;
	}

	if ((v & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_LINACC_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_LINACC_OUTPUT_ENABLED;
	}

	if ((v & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
		gReg.data[LPMS_CONFIG] |= LPMS_TEMPERATURE_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_TEMPERATURE_OUTPUT_ENABLED;
	}

	if ((v & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
		gReg.data[LPMS_CONFIG] |= LPMS_PRESSURE_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_PRESSURE_OUTPUT_ENABLED;
	}

	if ((v & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
		gReg.data[LPMS_CONFIG] |= LPMS_ALTITUDE_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_ALTITUDE_OUTPUT_ENABLED;
	}

	if ((v & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0)  {
		gReg.data[LPMS_CONFIG] |= LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	}

	if ((v & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0)  {
		gReg.data[LPMS_CONFIG] |= LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	} else {
		gReg.data[LPMS_CONFIG] &= ~LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	}

	return 1;
}

uint8_t getFirmwareVersion(uint8_t* data, uint16_t *l)
{
	setUi32t(&(data[0]), FIRMWARE_VERSION_DIGIT0);
	setUi32t(&(data[4]), FIRMWARE_VERSION_DIGIT1);
	setUi32t(&(data[8]), FIRMWARE_VERSION_DIGIT2);
	
	*l = 12;

	return 1;
}

uint8_t getStatus(uint8_t* data) 
{
  	uint8_t f = 1;  
  
	setUi32t(data, lpmsStatus);
	
	return f;
}

uint32_t getConfigReg(void) 
{
  	return gReg.data[LPMS_CONFIG];
}

void setTimestamp(uint8_t* data)
{
	measurementTime = getFloat(data);
}

uint8_t setFilterPreset(uint8_t* data) 
{
  	uint8_t f = 1;
	uint32_t v = getUi32t(data);
	
	switch (v) {
	case LPMS_FILTER_PRM_SET_1:
		lpFilterParam.accCovar = LPMS_FACTORY_ACC_COVAR_1;
		lpFilterParam.magCovar = LPMS_FACTORY_MAG_COVAR_1;
		gReg.data[LPMS_CONFIG] &= ~LPMS_DYNAMIC_COVAR_ENABLED;
		lpFilterParam.dynamicCovar = 0;
	break;

	case LPMS_FILTER_PRM_SET_2:
		lpFilterParam.accCovar = LPMS_FACTORY_ACC_COVAR_2;
		lpFilterParam.magCovar = LPMS_FACTORY_MAG_COVAR_2;
		gReg.data[LPMS_CONFIG] &= ~LPMS_DYNAMIC_COVAR_ENABLED;
		lpFilterParam.dynamicCovar = 0;
	break;

	case LPMS_FILTER_PRM_SET_3:
		lpFilterParam.accCovar = LPMS_FACTORY_ACC_COVAR_3;
		lpFilterParam.magCovar = LPMS_FACTORY_MAG_COVAR_3;
		gReg.data[LPMS_CONFIG] &= ~LPMS_DYNAMIC_COVAR_ENABLED;
		lpFilterParam.dynamicCovar = 0;
	break;

	case LPMS_FILTER_PRM_SET_4:
		lpFilterParam.accCovar = LPMS_FACTORY_ACC_COVAR_4;
		lpFilterParam.magCovar = LPMS_FACTORY_MAG_COVAR_4;
		gReg.data[LPMS_CONFIG] |= LPMS_DYNAMIC_COVAR_ENABLED;
		lpFilterParam.dynamicCovar = 1;
	break;
	
	default:
		lpFilterParam.accCovar = LPMS_FACTORY_ACC_COVAR_1;
		lpFilterParam.magCovar = LPMS_FACTORY_MAG_COVAR_1;
		gReg.data[LPMS_CONFIG] &= ~LPMS_DYNAMIC_COVAR_ENABLED;
		lpFilterParam.dynamicCovar = 0;
	break;
	}
	
	f = setRegFloat(LPMS_ACC_COVAR_USER, lpFilterParam.accCovar);
	f = setRegFloat(LPMS_MAG_COVAR_USER, lpFilterParam.magCovar);
	f = setRegUInt32(LPMS_FILTER_PRESET, v);

  	return f;
}

uint8_t setFilterMode(uint8_t* data) 
{
   	uint8_t f = 1; 
  	uint32_t v = getUi32t(data);
	
	switch (v) {
	case LPMS_FILTER_GYR:
		f = setRegUInt32(LPMS_FILTER_MODE, LPMS_FILTER_GYR);
		lpFilterParam.filterMode = LPMS_FILTER_GYR;
	break;
	  
	case LPMS_FILTER_GYR_ACC:	
		f = setRegUInt32(LPMS_FILTER_MODE, LPMS_FILTER_GYR_ACC);
		lpFilterParam.filterMode = LPMS_FILTER_GYR_ACC;
	break;
	  
	case LPMS_FILTER_GYR_ACC_MAG:
		f = setRegUInt32(LPMS_FILTER_MODE, LPMS_FILTER_GYR_ACC_MAG);
		lpFilterParam.filterMode = LPMS_FILTER_GYR_ACC_MAG;
	break;
	
	case LPMS_FILTER_ACC_MAG:
		f = setRegUInt32(LPMS_FILTER_MODE, LPMS_FILTER_ACC_MAG);
		lpFilterParam.filterMode = LPMS_FILTER_ACC_MAG;
	break;

	case LPMS_FILTER_GYR_ACC_EULER:
		f = setRegUInt32(LPMS_FILTER_MODE, LPMS_FILTER_GYR_ACC_EULER);
		lpFilterParam.filterMode = LPMS_FILTER_GYR_ACC_EULER;
	break;
	
	default:
	break;
	}
		
	return f;
}

uint8_t setLinAccCompMode(uint8_t* data) 
{
	uint8_t f = 1; 
	
	f = setRegUInt32(LPMS_LIN_ACC_COMP_MODE, getUi32t(data));

	updateLinAccCompMode();

	return f;
}

void updateLinAccCompMode(void)
{
	lpFilterParam.linAccCompMode = gReg.data[LPMS_LIN_ACC_COMP_MODE];
}

uint8_t getLinAccCompMode(uint8_t* data, uint16_t *l)
{
	setUi32t(&(data[0]), gReg.data[LPMS_LIN_ACC_COMP_MODE]);
	*l = 4;

	return 1;
}

uint8_t setCentriCompMode(uint8_t* data) 
{
	uint8_t f = 1; 

	f = setRegUInt32(LPMS_CENTRI_COMP_MODE, getUi32t(data));

	updateCentriCompMode();

	return f;
}

void updateCentriCompMode(void)
{
	lpFilterParam.centriCompMode = gReg.data[LPMS_CENTRI_COMP_MODE];
}

uint8_t getCentriCompMode(uint8_t* data, uint16_t *l)
{
	setUi32t(&(data[0]), gReg.data[LPMS_CENTRI_COMP_MODE]);
	*l = 4;

	return 1;
}

uint8_t setSelfTest(uint8_t* data)
{
	uint32_t v = getUi32t(data);

	switch (v) {
	case LPMS_SELF_TEST_OFF:
		lpmsStatus &= ~LPMS_SELF_TEST_RUNNING;
		isSelfTestOn = v;
		return 1;
	break;

	case LPMS_SELF_TEST_ON:
		lpmsStatus |= LPMS_SELF_TEST_RUNNING;
		isSelfTestOn = v;
		return 1;
	break;
	}

	return 0;
}

uint8_t getRawDataLp(uint8_t* data, uint16_t *l)
{
	setUi32t(&(data[0]), gReg.data[LPMS_RAW_DATA_LP]);

	*l = 4;

	return 0;
}

uint8_t setRawDataLp(uint8_t* data) 
{
  	uint32_t v = getUi32t(data);

	switch(v) {
	case LPMS_LP_OFF:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 1.0f;
	break;
	
	case LPMS_LP_01:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 0.1f;
	break;

	case LPMS_LP_005:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 0.05f;
	break;

	case LPMS_LP_001:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 0.01f;
	break;

	case LPMS_LP_0005:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 0.005f;
	break;

	case LPMS_LP_0001:
		gReg.data[LPMS_RAW_DATA_LP] = v;
		calibrationData.lpAlpha = 0.001f;
	break;
	}

	return 0;
}

void updateRawDataLp(void)
{
	switch(gReg.data[LPMS_RAW_DATA_LP]) {
	case LPMS_LP_OFF:
		calibrationData.lpAlpha = 1.0f;
	break;
	
	case LPMS_LP_01:
		calibrationData.lpAlpha = 0.1f;
	break;

	case LPMS_LP_005:
		calibrationData.lpAlpha = 0.05f;
	break;

	case LPMS_LP_001:
		calibrationData.lpAlpha = 0.01f;
	break;

	case LPMS_LP_0005:
		calibrationData.lpAlpha = 0.005f;
	break;

	case LPMS_LP_0001:
		calibrationData.lpAlpha = 0.001f;
	break;
	}
}

uint8_t getRollData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, rAfterOffset.data[0] * r2d, prec);
	} else {
		setFloat(data, rAfterOffset.data[0], prec);
	}	

	return 1;
}

uint8_t getPitchData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, rAfterOffset.data[1] * r2d, prec);
	} else {
		setFloat(data, rAfterOffset.data[1], prec);
	}

	return 1;
}

uint8_t getYawData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, rAfterOffset.data[2] * r2d, prec);
	} else {
		setFloat(data, rAfterOffset.data[2], prec);
	}

	return 1;
}

uint8_t getWXData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, w.data[0] * r2d, prec);
	} else {
		setFloat(data, w.data[0], prec);
	}

	return 1;
}

uint8_t getWYData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, w.data[1] * r2d, prec);
	} else {
		setFloat(data, w.data[1], prec);
	}

	return 1;
}

uint8_t getWZData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, w.data[2] * r2d, prec);
	} else {
		setFloat(data, w.data[2], prec);
	}

	return 1;
}

uint8_t getQ0Data(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, q.data[0], prec);

	return 1;
}

uint8_t getQ1Data(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, q.data[1], prec);

	return 1;
}

uint8_t getQ2Data(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, q.data[2], prec);

	return 1;
}

uint8_t getQ3Data(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, q.data[3], prec);

	return 1;
}

uint8_t getLinAccXData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, linAcc.data[0] * 10.0f, prec);

	return 1;
}

uint8_t getLinAccYData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, linAcc.data[1] * 10.0f, prec);

	return 1;
}

uint8_t getLinAccZData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, linAcc.data[2] * 10.0f, prec);

	return 1;
}

uint8_t getAccXData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, a.data[0], prec);

	return 1;
}

uint8_t getAccYData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, a.data[1], prec);

	return 1;
}

uint8_t getAccZData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, a.data[2], prec);

	return 1;
}

uint8_t getMagXData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, b.data[0], prec);

	return 1;
}

uint8_t getMagYData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, b.data[1], prec);

	return 1;
}

uint8_t getMagZData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, b.data[2], prec);

	return 1;
}

uint8_t getHeaveData(uint8_t* data, uint16_t *l, uint8_t prec) 
{
	setFloat(data, heaveY, prec);

	return 1;
}

uint8_t getGyroXData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, gRaw.data[0] * r2d, prec);
	} else {
		setFloat(data, gRaw.data[0], prec);
	}

	return 1;
}

uint8_t getGyroYData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, gRaw.data[1] * r2d, prec);
	} else {
		setFloat(data, gRaw.data[1], prec);
	}

	return 1;
}

uint8_t getGyroZData(uint8_t* data, uint16_t *l, uint8_t prec, int degRad) 
{
	const float r2d = 57.2958f;

	if (degRad == USE_DEGREE) {
		setFloat(data, gRaw.data[2] * r2d, prec);
	} else {
		setFloat(data, gRaw.data[2], prec);
	}

	return 1;
}

uint8_t setLpBusDataMode(uint8_t* data)
{
	uint32_t v = getUi32t(data);

	switch(v) {
	case LPMS_LPBUS_DATA_MODE_16:     
		 gReg.data[LPMS_CONFIG] |= LPMS_LPBUS_DATA_MODE_16BIT_ENABLED;
	break;

	default:
		gReg.data[LPMS_CONFIG] &= ~LPMS_LPBUS_DATA_MODE_16BIT_ENABLED;
	break;
	}

	return 1;
}