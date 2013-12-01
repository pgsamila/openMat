/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsIoInterface.h"

INIT_LOGGING

LpmsIoInterface::LpmsIoInterface(CalibrationData *configData) :
	configData(configData)
{
}

bool LpmsIoInterface::connect(std::string deviceId) 
{ 
	currentState = IDLE_STATE;
	
	waitForAck = false;
	waitForData = false;
	ackReceived = false;
	dataReceived = false;
	ackTimeout = 0;	
	dataTimeout = 0;
	
	lpmsStatus = 0;
	configReg = 0;
	imuId = 1;
	currentMode = LPMS_STREAM_MODE;
	
	latestLatency = 0.0f;
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;
	
	resendI = 0;
	cLength = 0;
	
	zeroImuData(&imuData);
	
	return true;
}

void LpmsIoInterface::zeroImuData(ImuData* id)
{
	id->q[0] = 1.0f;
	id->q[1] = 0.0f;
	id->q[2] = 0.0f;
	id->q[3] = 0.0f;
	
	for (int i=0; i<3; i++) id->r[i] = 0.0f;
	for (int i=0; i<3; i++) id->a[i] = 0.0f;
	for (int i=0; i<3; i++) id->g[i] = 0.0f;
	for (int i=0; i<3; i++) id->b[i] = 0.0f;
	for (int i=0; i<3; i++) id->aRaw[i] = 0.0f;
	for (int i=0; i<3; i++) id->gRaw[i] = 0.0f;
	for (int i=0; i<3; i++) id->bRaw[i] = 0.0f;
	for (int i=0; i<3; i++) id->linAcc[i] = 0.0f;
	
	id->pressure = 0.0f;
	id->altitude = 0.0f;
	id->temperature = 0.0f;
	id->hm.yHeave = 0.0f;
	
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			if (i != j) {
				id->rotationM[i*3+j] = 0.0f;
				id->rotOffsetM[i*3+j] = 0.0f;
			} else {
				id->rotationM[i*3+j] = 1.0f;
				id->rotOffsetM[i*3+j] = 1.0f;
			}
		}
	}
	
	id->openMatId = 1;
	id->frameCount = 0;
	id->timeStamp = 0.0f;
	
	id->hm.yHeave = 0.0f;
	
	id->gm.zGait = 0.0f;
	id->gm.yGait = 0.0f;
	id->gm.zAmplitude = 0.0f;
	id->gm.yAmplitude = 0.0f;
	id->gm.frequency = 0.0f;
	id->gm.velocity = 0.0f;
	id->gm.symmetry = 0.0f;
	id->gm.zDirection = 0;
	id->gm.yDirection = 0;
}

void LpmsIoInterface::setTxRxImuId(int id)
{
	imuId = id;
}
	
bool LpmsIoInterface::deviceStarted(void) 
{ 
	return true;
}
	
void LpmsIoInterface::loadData(ImuData *data) 
{ 
	*data = imuData;
}

bool LpmsIoInterface::pollData(void) 
{ 
	return true; 
}

void LpmsIoInterface::close(void) 
{ 
}

void LpmsIoInterface::startStreaming(void) 
{ 
}

void LpmsIoInterface::stopStreaming(void) 
{ 
}

long long LpmsIoInterface::getConnectWait(void) 
{ 
	return 0; 
}

float LpmsIoInterface::getSamplingTime(void) 
{
	return 0.005f;
}

int LpmsIoInterface::getGyroCalCycles(void)
{
	return 1000;
}

bool LpmsIoInterface::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{	
	return true;
}

bool LpmsIoInterface::parseModbusByte(void)
{
	return true;
}

bool LpmsIoInterface::isAck(void) 
{
	if (currentFunction == REPLY_ACK) {
		return true;
	}
	
	return false;
}

bool LpmsIoInterface::isNack(void) 
{
	if (currentFunction == REPLY_NACK) {
		return true;
	}
	
	return false;
}	

void LpmsIoInterface::receiveReset(void) 
{
	currentState = IDLE_STATE;
	waitForData = false;
	dataReceived = false;
	waitForAck = false;

	ackTimeout = 0;
	dataTimeout = 0;
	resendI = 0;
}	

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, long *v)
{	
	if (currentLength < 2) return false;
		
	*v = 0;
	for (int i=3; i>=0; --i) {
		*v = *v * 256;
		*v += (long) data[i];
	}

	return true;
}

bool LpmsIoInterface::fromBufferInt16(unsigned char *data, int *v)
{	
	boost::int16_t i;

	i = (boost::int16_t) data[1] << 8 | data[0];
	*v = i;

	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, long *v, int length)
{
	if ((int) currentLength < length*4) return false;
	
	for (int j=0; j<length; ++j) {
		v[j] = 0;
		for (int i=3; i>=0; --i) {
			v[j] = v[j] * 256;
			v[j] += (long) data[j*4+i];
		}
	}

	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, float *v)
{	
	boost::uint32_t w;

	if (currentLength < 2) return false;
		
	w = 0;
	for (int i=3; i>=0; --i) {
		w = w * 256;
		w += (long) data[i];
	}

	*v = conItoF(w);
	
	return true;
}

bool LpmsIoInterface::fromBufferBigEndian(unsigned char *data, float *v)
{	
	boost::uint32_t w;

	if (currentLength < 4) return false;
		
	w = 0;
	for (int i=0; i<4; ++i) {
		w = w * 256;
		w += (long) data[i];
	}

	*v = conItoF(w);
	
	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, 
	long *x, long *y, long *z)
{	
	if (currentLength < 12) return false;
	
	long v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j];
		}
	}

	*x = v[0];
	*y = v[1];
	*z = v[2];
	
	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, 
	float *x, float *y, float *z)
{	
	if (currentLength < 12) return false;
	
	boost::uint32_t v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j];
		}
	}

	*x = conItoF(v[0]);
	*y = conItoF(v[1]);
	*z = conItoF(v[2]);
	
	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, unsigned start,
	float *x, float *y, float *z)
{	
	if (currentLength < (start+12)) return false;
	
	boost::uint32_t v[3];
	
	for (int i=0; i<3; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*x = conItoF(v[0]);
	*y = conItoF(v[1]);
	*z = conItoF(v[2]);
	
	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, unsigned start,
	float *q0, float *q1, float *q2, float *q3)
{	
	if (currentLength < (start+16)) return false;
	
	boost::uint32_t v[4];
	
	for (int i=0; i<4; i++) {
		v[i] = 0;
		for (int j=3; j>=0; --j) {
			v[i] = v[i] * 256;
			v[i] += (long) data[i*4+j+start];
		}
	}

	*q0 = conItoF(v[0]);
	*q1 = conItoF(v[1]);
	*q2 = conItoF(v[2]);
	*q3 = conItoF(v[3]);
	
	return true;
}

bool LpmsIoInterface::fromBuffer(std::vector<unsigned char> data, unsigned start, float *v)
{	
	boost::uint32_t i;

	if (currentLength < (2+start)) return false;
		
	i = 0;
	for (int j=3; j>=0; --j) {
		i = i * 256;
		i += (long) data[j+start];
	}

	*v = conItoF(i);
	
	return true;
}

bool LpmsIoInterface::fromBuffer(unsigned char *data, float *v)
{	
	boost::uint32_t i;

	i = 0;
	for (int j=3; j>=0; --j) {
		i = i * 256;
		i += (long) data[j];
	}

	*v = conItoF(i);
	
	return true;
}

bool LpmsIoInterface::parseFieldMapData(void)
{
	unsigned p, r, y;

	p = currentFieldMapPitch; 
	r = currentFieldMapRoll; 
	y = currentFieldMapYaw; 
	
	fromBuffer(oneTx, 0, &(configData->fieldMap[p][r][y].data[0]), 
		&(configData->fieldMap[p][r][y].data[1]), 
		&(configData->fieldMap[p][r][y].data[2]));
	
	return true;
}

void LpmsIoInterface::resetTimestamp(void)
{
	timestampOffset = currentTimestamp;
}

bool LpmsIoInterface::parseSensorData(void)
{
	unsigned o=0;
	float r0, r1, r2;
	const float r2d = 57.2958f;

	zeroImuData(&imuData);
	
	fromBuffer(oneTx, o, &currentTimestamp);
	o = o + 4;
	
	if (timestampOffset > currentTimestamp) timestampOffset = currentTimestamp;
	imuData.timeStamp = currentTimestamp - timestampOffset;
	
	if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
		fromBuffer(oneTx, o, &r0, &r1, &r2);
		imuData.gRaw[0] = r0 * r2d;
		imuData.gRaw[1] = r1 * r2d;
		imuData.gRaw[2] = r2 * r2d;	
		o = o + 12;
	}
	
	if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
		fromBuffer(oneTx, o, &(imuData.aRaw[0]), &(imuData.aRaw[1]), &(imuData.aRaw[2]));	
		o = o + 12;
	}	

	if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
		fromBuffer(oneTx, o, &(imuData.bRaw[0]), &(imuData.bRaw[1]), &(imuData.bRaw[2]));	
		o = o + 12;
	}
	
	if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
		fromBuffer(oneTx, o, &(imuData.w[0]), &(imuData.w[1]), &(imuData.w[2]));	
		o = o + 12;
	}
	
	if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
		fromBuffer(oneTx, o, &(imuData.q[0]), &(imuData.q[1]), &(imuData.q[2]), &(imuData.q[3]));	
		o = o + 16;
	}	
	  	
	if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &r0, &r1, &r2);
		imuData.r[0] = r0 * r2d;
		imuData.r[1] = r1 * r2d;
		imuData.r[2] = r2 * r2d;	
		o = o + 12;
	}	

	if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &(imuData.linAcc[0]), &(imuData.linAcc[1]), &(imuData.linAcc[2]));
		o = o + 12;
	}	
 
	if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &imuData.pressure);
		o = o + 4;
	}
	
	if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &imuData.altitude);
		o = o + 4;
	}

	if ((configReg & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &imuData.temperature);
		o = o + 4;
	}
	
	if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0)  {
		fromBuffer(oneTx, o, &imuData.hm.yHeave);
		o = o + 4;
	}
	
	if (imuDataQueue.size() < 64) {
		imuDataQueue.push(imuData);
	}

	return true;
}

bool LpmsIoInterface::getLatestImuData(ImuData *id) 
{
	if (imuDataQueue.empty() == true) return false;
	
	*id = imuDataQueue.front();
	imuDataQueue.pop();
	
	return true;
}

void LpmsIoInterface::clearDataQueue(void)
{
	while (imuDataQueue.empty() == false) imuDataQueue.pop();
}

bool LpmsIoInterface::parseFunction(void) 
{
	long l;
	long i0, i1, i2;
	int selectedData = 0;
	long a[64];
	
	if (waitForAck == true) {
		if (isAck() == true) {
			ackReceived = true;
						
			return true;
		}

		if (isNack() == true) {
			receiveReset();
			
			return false;
		}
	}
	
	switch (currentFunction) {
	case GET_CAN_CONFIGURATION:
		fromBuffer(oneTx, &l);		
		
		if ((l & LPMS_CAN_FIXEDPOINT_MODE) != 0) {
			configData->setParameter(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FIXED);	
		} else {
			configData->setParameter(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FLOATING);
		}

		if ((l & LPMS_CAN_SEQUENTIAL_MODE) != 0) {
			configData->setParameter(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_SEQUENTIAL);
		} else {
			configData->setParameter(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_CANOPEN);		
		}
		
		selectedData = (l & 0xffff0000) >> 16;
		configData->setParameter(PRM_CAN_START_ID, selectedData);
	break;
	
	case GET_CONFIG:
		latestLatency = latencyTimer.measure() / 1000.0f;	
	
		if (fromBuffer(oneTx, &l)) configReg = l;		
		
		if ((configReg & LPMS_GYR_THRES_ENABLED) != 0) {
			configData->setParameter(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_ENABLED);
		} else {
			configData->setParameter(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
		}
		
		if ((configReg & LPMS_GYR_AUTOCAL_ENABLED) != 0) {
			configData->setParameter(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
		} else {
			configData->setParameter(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_DISABLED);
		}
		
		if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_30HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_30HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_50HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_200HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_500HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_500HZ);	
		} else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1000HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_1000HZ);	
		} else {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);	
		}	

		if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_125K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS);
		} else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_250K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_250KBPS);
		} else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_500K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_500KBPS);
		} else /* ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_1M_ENABLED) */ {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
		}
		
		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;	
		}		
		
		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_GYRO_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_QUAT_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_EULER_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_LINACC_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;	
		}

		if ((configReg & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;	
		}		
		
		if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;	
		}
		
		if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
		}
		
		if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
		} else {
			selectedData &= ~SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
		}
		
		configData->setParameter(PRM_SELECT_DATA, selectedData);
		
		if ((configReg & LPMS_HEAVEMOTION_ENABLED) != 0) {
			configData->setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_ENABLED);
		} else {
			configData->setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_DISABLED);
		}
		
		if ((configReg & LPMS_GAIT_TRACKING_ENABLED) != 0) {
			configData->setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_ENABLED);
		} else {
			configData->setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_DISABLED);
		}
	break;	
	
	case GET_SENSOR_DATA:
		parseSensorData();
		
		return true; // <- Careful, in case of GET_DATA don't reset anything to prevent overflow in stream mode.
		// if ((lpmsStatus & LPMS_STREAM_MODE) != 0) return true;
	break;

	case GET_IMU_ID:
		if (fromBuffer(oneTx, &l)) {
			configData->setParameter(PRM_OPENMAT_ID, (int) l);
			imuId = l;
		}
	break;

	case GET_STATUS:
		if (fromBuffer(oneTx, &l)) lpmsStatus = l;
	break;

	case GET_GYR_RANGE:
		if (fromBuffer(oneTx, &l)) configData->setParameter(PRM_GYR_RANGE, (int) l);					
	break;

	case GET_ACC_RANGE:
		if (fromBuffer(oneTx, &l)) configData->setParameter(PRM_ACC_RANGE, (int) l);	
	break;

	case GET_MAG_RANGE:
		if (fromBuffer(oneTx, &l)) configData->setParameter(PRM_MAG_RANGE, (int) l);			
	break;
	
	case GET_FILTER_MODE:
		fromBuffer(oneTx, &l);
		switch(l) {
		case LPMS_FILTER_GYR:
			configData->setParameter(PRM_FILTER_MODE, SELECT_FM_GYRO_ONLY);
		break;
		
		case LPMS_FILTER_GYR_ACC:
			configData->setParameter(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC);
		break;

		case LPMS_FILTER_GYR_ACC_MAG:
			configData->setParameter(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC_MAG);
		break;

		case LPMS_FILTER_ACC_MAG:
			configData->setParameter(PRM_FILTER_MODE, SELECT_FM_ACC_MAG);
		break;
		
		case LPMS_FILTER_GYR_ACC_EULER:
			configData->setParameter(PRM_FILTER_MODE, SELECT_FM_GYR_ACC_EULER);
		break;
		}
	break;
	
	case GET_FILTER_PRESET:
		fromBuffer(oneTx, &l);
		switch(l) {
		case LPMS_FILTER_PRM_SET_1:
			configData->setParameter(PRM_PARAMETER_SET, SELECT_IMU_SLOW);				
		break;

		case LPMS_FILTER_PRM_SET_2:
			configData->setParameter(PRM_PARAMETER_SET, SELECT_IMU_MEDIUM);				
		break;
		
		case LPMS_FILTER_PRM_SET_3:
			configData->setParameter(PRM_PARAMETER_SET, SELECT_IMU_FAST);				
		break;	
		
		case LPMS_FILTER_PRM_SET_4:
			configData->setParameter(PRM_PARAMETER_SET, SELECT_IMU_DYNAMIC);				
		break;	
		}
	break;
	
	case GET_HARD_IRON_OFFSET:
		fromBuffer(oneTx, 0, &(configData->hardIronOffset.data[0]), &(configData->hardIronOffset.data[1]), &(configData->hardIronOffset.data[2]));
	break;
	
	case GET_ACC_BIAS:
		fromBuffer(oneTx, 0, &(configData->accBias.data[0]), &(configData->accBias.data[1]), &(configData->accBias.data[2]));
	break;
	
	case GET_SOFT_IRON_MATRIX:
		for (int i=0; i<3; i++) {
			fromBuffer(oneTx, i*12, &(configData->softIronMatrix.data[i][0]), &(configData->softIronMatrix.data[i][1]), &(configData->softIronMatrix.data[i][2]));
		}
	break;
	
	case GET_ACC_ALIGN_MATRIX:
		for (int i=0; i<3; i++) {
			fromBuffer(oneTx, i*12, &(configData->misalignMatrix.data[i][0]), &(configData->misalignMatrix.data[i][1]), &(configData->misalignMatrix.data[i][2]));
		}
	break;

	case GET_FIELD_ESTIMATE:
		fromBuffer(oneTx, 0, &(configData->fieldRadius));
	break;
	
	case GET_FIRMWARE_VERSION:
		fromBuffer(oneTx, &i0, &i1, &i2);
		configData->firmwareVersion = static_cast<ostringstream*>(&(ostringstream() << i2))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i1))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i0))->str();
	break;

	case GET_GYR_ALIGN_BIAS:
		fromBuffer(oneTx, 0, &(configData->gyrAlignmentBias.data[0]), &(configData->gyrAlignmentBias.data[1]), &(configData->gyrAlignmentBias.data[2]));
	break;
	
	case GET_GYR_ALIGN_MATRIX:
		for (int i=0; i<3; i++) {
			fromBuffer(oneTx, i*12, &(configData->gyrMisalignMatrix.data[i][0]), &(configData->gyrMisalignMatrix.data[i][1]), &(configData->gyrMisalignMatrix.data[i][2]));
		}		
	break;	
	
	case GET_RAW_DATA_LP:
		fromBuffer(oneTx, &l);
		switch(l) {
		case LPMS_LP_OFF:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_OFF);				
		break;

		case LPMS_LP_01:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_01);				
		break;
		
		case LPMS_LP_005:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_005);				
		break;	
		
		case LPMS_LP_001:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_001);				
		break;	
		
		case LPMS_LP_0005:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_0005);				
		break;

		case LPMS_LP_0001:
			configData->setParameter(PRM_LOW_PASS, SELECT_LPMS_LP_0001);				
		break;
		}
	break;
	
	case GET_CAN_MAPPING:
		fromBuffer(oneTx, a, 8);	
		configData->setParameter(PRM_CAN_MAPPING, (int *) a);
	break;
	
	case GET_CAN_HEARTBEAT:
		fromBuffer(oneTx, &l);	
		configData->setParameter(PRM_CAN_HEARTBEAT, (int) l);
	break;
	
	case GET_LIN_ACC_COMP_MODE:
		fromBuffer(oneTx, &l);	
		switch(l) {
		case LPMS_LIN_ACC_COMP_MODE_OFF:
			configData->setParameter(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_OFF);				
		break;

		case LPMS_LIN_ACC_COMP_MODE_WEAK:
			configData->setParameter(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK);				
		break;
		
		case LPMS_LIN_ACC_COMP_MODE_STRONG:
			configData->setParameter(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG);				
		break;	
		}
	break;
	
	case GET_CENTRI_COMP_MODE:
		fromBuffer(oneTx, &l);	
		switch(l) {
		case LPMS_CENTRI_COMP_MODE_OFF:
			configData->setParameter(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_OFF);				
		break;

		case LPMS_CENTRI_COMP_MODE_ON:
			configData->setParameter(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_ON);				
		break;	
		}
	break;

	default:
		return false;
	break;
	}
	
	receiveReset();

	return true;
}

void LpmsIoInterface::clearRxBuffer(void)
{
	while (dataQueue.size() > 0) {
		dataQueue.pop();
	}
	
	oneTx.clear();
	receiveReset();
}

bool LpmsIoInterface::checkState(void)
{
	parseModbusByte();

	if (waitForAck == true && ackReceived == false) {	
		if (ackTimer.measure() > ACK_MAX_TIME) {
			if (resendI < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendI;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpmsIoInterface] ACK timeout error. Resending command: %d\n", resendI);
				
				return true;
			} else {			
				currentState = IDLE_STATE;
				waitForAck = false;
				ackReceived = false;
				ackTimeout = 0;
			
				LOGV("[LpmsIoInterface] ACK timeout error. Resetting send queue.\n");
			
				if (ifs.is_open() == true) ifs.close();
			
				return false;
			}
		}
	} 
	
	if (waitForData == true && dataReceived == false) {
		if (ackTimer.measure() > ACK_MAX_TIME) {
			if (resendI < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendI;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpmsIoInterface] ACK timeout error. Resending command: %d\n", resendI);
				
				return true;
			} else {			
				currentState = IDLE_STATE;
				waitForAck = false;
				ackReceived = false;
				ackTimeout = 0;
			
				LOGV("[LpmsIoInterface] ACK timeout error. Resetting send queue.\n");
			
				return false;
			}
		}
	}

	if (waitForAck == true && ackReceived == true) {
		switch (currentState) {
			case UPDATE_FIRMWARE:
				handleFirmwareFrame();
			break;
			
			case UPDATE_IAP:
				handleIAPFrame();
			break;
			
			case SET_CAN_STREAM_FORMAT:
				receiveReset();
				zeroImuData(&imuData);
			break;
			
			default:
				receiveReset();
			break;
		}
	}
	
	return true;
}

bool LpmsIoInterface::isWaitForData(void)
{
	return waitForData;
}

bool LpmsIoInterface::isWaitForAck(void)
{
	return waitForAck;
}

bool LpmsIoInterface::startUploadFirmware(std::string fn)
{
	bool f = false;
	long long l;
	unsigned long r;
	
	if (ifs.is_open() == true) ifs.close();

	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpmsIoInterface] Firmware file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open firmware file %s\n", fn.c_str());
		f = false;
	
		return f;
	}
	
	ifs.seekg(0, ios::end);
	l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] Firmware filesize: %d\n", l);
	
	firmwarePages = l / FIRMWARE_PACKET_LENGTH;
	r = l % FIRMWARE_PACKET_LENGTH;
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] Firmware pages: %d\n", firmwarePages);	
	LOGV("[LpmsIoInterface] Firmware remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] Firmware packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_FIRMWARE, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;	
	uploadTimer.reset();
	
	return f;
}

int LpmsIoInterface::getCurrentState(void)
{
	return currentState;
}

bool LpmsIoInterface::checkUploadTimeout(void)
{
	if (currentState == UPDATE_FIRMWARE) {
		if (uploadTimer.measure() > MAX_UPLOAD_TIME) {	
			currentState = IDLE_STATE;
		
			waitForAck = false;
			ackReceived = false;	

			ifs.close();
			
			LOGV("[LpmsIoInterface] Firmware upload failed. Please reconnect sensor and retry.\n");

			return false;
		}
	}
	
	return true;
}

bool LpmsIoInterface::handleFirmwareFrame(void)
{
	uploadTimer.reset();
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
		
		ifs.close();		
		
		return false;
	}

	if (ifs.eof() == true || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;	

		ifs.close();
		
		LOGV("[LpmsIoInterface] Firmware upload finished. Now writing to flash. Please DO NOT detach the power from the device for 15s.\n");
		
		return true;
	}

	LOGV("[LpmsIoInterface] Firmware sending packet %d\n", pCount);
	++pCount;

	for (unsigned i=0; i < FIRMWARE_PACKET_LENGTH; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, FIRMWARE_PACKET_LENGTH);
	cLength = FIRMWARE_PACKET_LENGTH;
	sendModbusData(imuId, UPDATE_FIRMWARE, FIRMWARE_PACKET_LENGTH, (unsigned char *)cBuffer);

	ackTimeout = 0;
	ackTimer.reset();
	dataTimeout = 0;
	resendI = 0;

	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;	
	
	return true;
}

bool LpmsIoInterface::startUploadIap(std::string fn)
{
	bool f = false;

	if (ifs.is_open() == true) ifs.close();
	
	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpmsIoInterface] IAP file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open IAP file %s\n", fn.c_str());	
		f = false;
	}
	
	ifs.seekg(0, ios::end);
	long long l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] IAP filesize: %d\n", l);
	
	firmwarePages = l / 256;
	unsigned long r = (long) (l % 256);
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] IAP pages: %d\n", firmwarePages);
	LOGV("[LpmsIoInterface] IAP remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] IAP packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_IAP, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;
	uploadTimer.reset();
	
	return f;
}	

bool LpmsIoInterface::handleIAPFrame(void)
{
	uploadTimer.reset();	
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
			
		ifs.close();	
		
		return false;
	}

	if (ifs.eof() == true) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;

		ifs.close();
		
		LOGV("[LpmsIoInterface] IAP upload finished\n");
		
		return true;		
	}
	
	LOGV("[LpmsIoInterface] Sending IAP packet %d\n", pCount);
	++pCount;		
	
	for (unsigned i=0; i < FIRMWARE_PACKET_LENGTH; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, FIRMWARE_PACKET_LENGTH);
	cLength = FIRMWARE_PACKET_LENGTH;
	sendModbusData(imuId, UPDATE_IAP, FIRMWARE_PACKET_LENGTH, (unsigned char *)cBuffer);
	
	ackTimeout = 0;
	dataTimeout = 0;
	resendI = 0;
	ackTimer.reset();
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;		
		
	return true;
}

boost::uint32_t LpmsIoInterface::conFtoI(float f)
{
	float2uint f2int;
	f2int.fp = f;
	return f2int.up;
}

float LpmsIoInterface::conItoF(boost::uint32_t v)
{
	float2uint f2int;
	f2int.up = v;
	return f2int.fp;
}	

bool LpmsIoInterface::modbusSetNone(unsigned command) 
{
	bool r;

	receiveReset();

	cLength = 0;
	r = sendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	waitForAck = true;
	waitForData = false;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpmsIoInterface::modbusGet(unsigned command) 
{
	bool r;

	receiveReset();

	cLength = 0;
	r = sendModbusData(imuId, command, 0, cBuffer);
	
	currentState = command;
	
	waitForData = true;
	waitForAck = false;	
	dataReceived = false;
	dataTimeout = 0;
	ackTimer.reset();	
	
	return r;
}

bool LpmsIoInterface::modbusGetMultiUint32(unsigned command, boost::uint32_t *v, int n) 
{
	bool r;
	boost::uint32_t t;
	
	receiveReset();

	for (int j=0; j<n; ++j) {
		t = v[j];
		for (int i=0; i<4; ++i) {
			cBuffer[j*4+i] = t & 0xff;
			t = t >> 8;
		}
	}
	
	cLength = n*4;
	r = sendModbusData(imuId, command, n*4, cBuffer);
	
	currentState = command;
	waitForData = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpmsIoInterface::modbusSetInt32(unsigned command, long v)
{
	bool r;	
	
	receiveReset();

	for (int i=0; i<4; ++i) {
		cBuffer[i] = (unsigned char) (v & 0xff);
		v = v >> 8;
	}
	
	cLength = 4;
	r = sendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpmsIoInterface::modbusSetInt32Array(unsigned command, long *v, int length)
{
	bool r;
	
	receiveReset();

	for (int j=0; j<length; ++j) {
		for (int i=0; i<4; ++i) {
			cBuffer[j*4+i] = (unsigned char) (v[j] & 0xff);
			v[j] = v[j] >> 8;
		}
	}
	
	cLength = 4*length;
	r = sendModbusData(imuId, command, 4*length, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpmsIoInterface::modbusSetVector3Int32(unsigned command, long x, long y, long z)
{
	bool r;
	long v[3] = { x, y, z };

	receiveReset();
	
	for (int i=0; i<3; ++i) {	
		for (int j=3; j>=0; --j) {
			cBuffer[i*4+j] = (unsigned char) (v[i] & 0xff);
			v[i] = v[i] / 256;
		}
	}
	
	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;
}

bool LpmsIoInterface::modbusSetFloat(unsigned command, float v)
{
	boost::uint32_t i;
	boost::uint32_t m = 0xff;
	bool r;		

	receiveReset();

	i = conFtoI(v);

	for (int j=0; j<4; j++) {
		cBuffer[j] = (unsigned char) (i & 0xff);
		i = i / 256;
	}

	cLength = 4;
	r = sendModbusData(imuId, command, 4, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();
	
	return r;		
}

bool LpmsIoInterface::modbusSetVector3Float(unsigned command, float x, float y, float z)
{
	boost::uint32_t i;
	boost::uint32_t m = 0xff;
	float v[3] = { x, y, z };
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; ++j) {	
		i = conFtoI(v[j]);
		for (int k=3; k>=0; --k) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}

bool LpmsIoInterface::modbusSetVector3f(unsigned command, LpVector3f v)
{
	boost::uint32_t i;	
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; j++) {	
		i = conFtoI(v.data[j]);
		for (int k=0; k<4; k++) {
			cBuffer[j*4+k] = (unsigned char) (i & 0xff);
			i = i / 256;
		}
	}

	cLength = 12;
	r = sendModbusData(imuId, command, 12, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}

bool LpmsIoInterface::modbusSetMatrix3x3f(unsigned command, LpMatrix3x3f m)
{
	boost::uint32_t i;
	bool r;	
	
	receiveReset();

	for (int j=0; j<3; j++) {	
		for (int l=0; l<3; l++) {
			i = conFtoI(m.data[j][l]);
			for (int k=0; k<4; k++) {
				cBuffer[j*3*4+l*4+k] = (unsigned char) (i & 0xff);
				i = i / 256;
			}
		}
	}

	cLength = 36;
	r = sendModbusData(imuId, command, 36, cBuffer);
	
	currentState = command;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;	
	ackTimer.reset();
	
	return r;		
}

bool LpmsIoInterface::selectData(long p) 
{	
	boost::uint32_t v = 0;
	
	if ((p & SELECT_LPMS_ACC_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ACC_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ACC_RAW_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_MAG_OUTPUT_ENABLED) != 0) {
		v |= LPMS_MAG_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_MAG_RAW_OUTPUT_ENABLED;
	}	
	
	if ((p & SELECT_LPMS_GYRO_OUTPUT_ENABLED) != 0) {
		v |= LPMS_GYR_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_GYR_RAW_OUTPUT_ENABLED;
	}	
	
	if ((p & SELECT_LPMS_QUAT_OUTPUT_ENABLED) != 0) {
		v |= LPMS_QUAT_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_QUAT_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_EULER_OUTPUT_ENABLED) != 0) {
		v |= LPMS_EULER_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_EULER_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_PRESSURE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_PRESSURE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ALTITUDE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ALTITUDE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
		v |= LPMS_TEMPERATURE_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_TEMPERATURE_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_LINACC_OUTPUT_ENABLED) != 0) {
		v |= LPMS_LINACC_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_LINACC_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	}
	
	if ((p & SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
		v |= LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	}
	
	modbusSetInt32(SET_TRANSMIT_DATA, v);
	
	return true;
}

int LpmsIoInterface::getMode(void)
{
	if ((lpmsStatus & LPMS_COMMAND_MODE) != 0) {
			return SELECT_LPMS_MODE_COMMAND;
		} else if ((lpmsStatus & LPMS_STREAM_MODE) != 0) {
			return SELECT_LPMS_MODE_STREAM;
		} else if ((lpmsStatus & LPMS_SLEEP_MODE) != 0) {
			return SELECT_LPMS_MODE_SLEEP;
		}

	return currentMode;
}

bool LpmsIoInterface::setCommandMode(void) 
{	
	bool r;

	lpmsStatus |= LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	lpmsStatus &= ~LPMS_SLEEP_MODE;
	
	r = modbusSetNone(GOTO_COMMAND_MODE);
	
	return r;
}

bool LpmsIoInterface::setStreamMode(void) 
{	
	bool r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus |= LPMS_STREAM_MODE;
	lpmsStatus &= ~LPMS_SLEEP_MODE;	
	
	r = modbusSetNone(GOTO_STREAM_MODE);

	return r;
}
	
bool LpmsIoInterface::setSleepMode(void) 
{	
	bool r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	lpmsStatus |= LPMS_SLEEP_MODE;	
	
	r = modbusSetNone(GOTO_SLEEP_MODE);

	return r;
}

bool LpmsIoInterface::restoreFactoryValue(void)
{
	return modbusSetNone(RESTORE_FACTORY_VALUE);
}

bool LpmsIoInterface::setSelfTest(long v)
{
	return modbusSetInt32(SELF_TEST, v);
}

bool LpmsIoInterface::setImuId(long v)
{
	bool f;
	
	f = modbusSetInt32(SET_IMU_ID, v);
	imuId = v;

	return f;
}

bool LpmsIoInterface::setStreamFrequency(long v)
{
	return modbusSetInt32(SET_STREAM_FREQ, v);
}

bool LpmsIoInterface::startGyrCalibration(void)
{
	return modbusSetNone(START_GYR_CALIBRA);
}

bool LpmsIoInterface::setGyrRange(long v)
{
	return modbusSetInt32(SET_GYR_RANGE, v);
}

bool LpmsIoInterface::setMagRange(long v)
{
	return modbusSetInt32(SET_MAG_RANGE, v);
}

bool LpmsIoInterface::setAccRange(long v)
{
	return modbusSetInt32(SET_ACC_RANGE, v);
}

bool LpmsIoInterface::setAccBias(LpVector3f v)
{
	return modbusSetVector3f(SET_ACC_BIAS, v);
}

bool LpmsIoInterface::enableGyrThres(long v)
{
	return modbusSetInt32(ENABLE_GYR_THRES, v);
}

bool LpmsIoInterface::enableGyrAutocalibration(long v)
{
	return modbusSetInt32(ENABLE_GYR_AUTOCAL, v);
}

bool LpmsIoInterface::resetOrientation(void) 
{
	return modbusSetNone(SET_OFFSET);
}

bool LpmsIoInterface::resetReference(void) 
{
	return modbusSetNone(RESET_REFERENCE);
}

bool LpmsIoInterface::setFilterMode(long v)
{
	return modbusSetInt32(SET_FILTER_MODE, v);
}

bool LpmsIoInterface::setFilterPreset(long v)
{
	return modbusSetInt32(SET_FILTER_PRESET, v);
}

bool LpmsIoInterface::setCanStreamFormat(long v)
{
	return modbusSetInt32(SET_CAN_STREAM_FORMAT, v);
}

bool LpmsIoInterface::setCanBaudrate(long v)
{
	return modbusSetInt32(SET_CAN_BAUDRATE, v);
}

bool LpmsIoInterface::setFieldEstimate(float v)
{
	return modbusSetFloat(SET_FIELD_ESTIMATE, v);
}

bool LpmsIoInterface::getConfig(void)
{
	latencyTimer.reset();

	return modbusGet(GET_CONFIG);
}

bool LpmsIoInterface::getImuId(void)
{
	return modbusGet(GET_IMU_ID);
}

bool LpmsIoInterface::getStatus(void)
{
	return modbusGet(GET_STATUS);
}

bool LpmsIoInterface::getGyrRange(void)
{
	return modbusGet(GET_GYR_RANGE);
}

bool LpmsIoInterface::getAccRange(void)
{
	return modbusGet(GET_ACC_RANGE);
}

bool LpmsIoInterface::getMagRange(void)
{
	return modbusGet(GET_MAG_RANGE);
}	

bool LpmsIoInterface::getAccBias(void)
{
	return modbusGet(GET_ACC_BIAS);
}

bool LpmsIoInterface::getSensorData(void)
{
	return modbusGet(GET_SENSOR_DATA);
}

bool LpmsIoInterface::getFilterMode(void)
{
	return modbusGet(GET_FILTER_MODE);
}

bool LpmsIoInterface::getFilterPreset(void)
{
	return modbusGet(GET_FILTER_PRESET);
}

bool LpmsIoInterface::getFieldEstimate(void)
{
	return modbusGet(GET_FIELD_ESTIMATE);
}

bool LpmsIoInterface::isCalibrating(void)
{
	if ((lpmsStatus & (LPMS_GYR_CALIBRATION_RUNNING | 
		LPMS_MAG_CALIBRATION_RUNNING | LPMS_REF_CALIBRATION_RUNNING)) != 0) {
		return true;
	}
	
	return false;
}

bool LpmsIoInterface::isError(void)
{
	if (((lpmsStatus & (LPMS_GYR_INIT_FAILED | LPMS_ACC_INIT_FAILED | 
		LPMS_MAG_INIT_FAILED | LPMS_PRESSURE_INIT_FAILED | LPMS_GYR_UNRESPONSIVE |
		LPMS_ACC_UNRESPONSIVE | LPMS_MAG_UNRESPONSIVE | LPMS_FLASH_WRITE_FAILED |LPMS_SET_BAUDRATE_FAILED | LPMS_SET_BROADCAST_FREQ_FAILED)) != 0)) {
		return true;
	}
	
	return false;
}

bool LpmsIoInterface::writeRegisters(void)
{
	return modbusSetNone(WRITE_REGISTERS);
}

bool LpmsIoInterface::getHardIronOffset(void)
{
	return modbusGet(GET_HARD_IRON_OFFSET);
}

bool LpmsIoInterface::getSoftIronMatrix(void)
{
	return modbusGet(GET_SOFT_IRON_MATRIX);
}

bool LpmsIoInterface::setHardIronOffset(LpVector3f v)
{
	return modbusSetVector3f(SET_HARD_IRON_OFFSET, v);
}

bool LpmsIoInterface::setSoftIronMatrix(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_SOFT_IRON_MATRIX, m);
}

bool LpmsIoInterface::setAccAlignment(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_ACC_ALIGN_MATRIX, m);
}

bool LpmsIoInterface::getAccAlignment(void)
{
	return modbusGet(GET_ACC_ALIGN_MATRIX);
}

bool LpmsIoInterface::getFirmwareVersion(void)
{
	return modbusGet(GET_FIRMWARE_VERSION);
}

CalibrationData *LpmsIoInterface::getConfigData(void)
{
	return configData;
}

bool LpmsIoInterface::getUploadProgress(int *p)
{
	if (firmwarePages > 0) {
		*p = pCount * 100 / (int) firmwarePages;
	} else {
		*p = 0;
	}

	if (checkUploadTimeout() == false) return false;
	
	return true;
}

float LpmsIoInterface::getLatestLatency(void)
{
	return latestLatency;
}

bool LpmsIoInterface::setGyrAlignment(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_GYR_ALIGN_MATRIX, m);
}

bool LpmsIoInterface::setGyrAlignmentBias(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_ALIGN_BIAS, v);
}

bool LpmsIoInterface::getGyrAlignment(void)
{
	return modbusGet(GET_GYR_ALIGN_MATRIX);
}

bool LpmsIoInterface::getGyrAlignmentBias(void)
{
	return modbusGet(GET_GYR_ALIGN_BIAS);
}

bool LpmsIoInterface::setGyrTempCalPrmA(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_TEMP_CAL_PRM_A, v);
}

bool LpmsIoInterface::setGyrTempCalPrmB(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_TEMP_CAL_PRM_B, v);
}

bool LpmsIoInterface::setGyrTempCalBaseV(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_TEMP_CAL_BASE_V, v);
}

bool LpmsIoInterface::setGyrTempCalBaseT(float v)
{
	return modbusSetFloat(SET_GYR_TEMP_CAL_BASE_T, v);
}

bool LpmsIoInterface::getGyrTempCalPrmA(void)
{
	return modbusGet(GET_GYR_TEMP_CAL_PRM_A);
}

bool LpmsIoInterface::getGyrTempCalPrmB(void)
{
	return modbusGet(GET_GYR_TEMP_CAL_PRM_B);
}

bool LpmsIoInterface::getGyrTempCalBaseV(void)
{
	return modbusGet(GET_GYR_TEMP_CAL_BASE_V);
}

bool LpmsIoInterface::getGyrTempCalBaseT(void)
{
	return modbusGet(GET_GYR_TEMP_CAL_BASE_T);
}

long LpmsIoInterface::getConfigReg(void) {
	return configReg;
}

bool LpmsIoInterface::getRawDataLpFilter(void)
{
	return modbusGet(GET_RAW_DATA_LP);	
}

bool LpmsIoInterface::setRawDataLpFilter(int v)
{
	return modbusSetInt32(SET_RAW_DATA_LP, v);
}

bool LpmsIoInterface::getCanMapping(void)
{
	return modbusGet(GET_CAN_MAPPING);	
}

bool LpmsIoInterface::setCanMapping(int *a)
{
	return modbusSetInt32Array(SET_CAN_MAPPING, (long *) a, 8);
}

bool LpmsIoInterface::getCanHeartbeat(void)
{
	return modbusGet(GET_CAN_HEARTBEAT);	
}

bool LpmsIoInterface::setCanHeartbeat(int v)
{
	return modbusSetInt32(SET_CAN_HEARTBEAT, v);
}

bool LpmsIoInterface::resetSensorTimestamp(void)
{
	return modbusSetNone(RESET_TIMESTAMP);
}

bool LpmsIoInterface::getLinAccCompMode(void)
{
	return modbusGet(GET_LIN_ACC_COMP_MODE);
}

bool LpmsIoInterface::setLinAccCompMode(int v)
{
	return modbusSetInt32(SET_LIN_ACC_COMP_MODE, v);
}

bool LpmsIoInterface::getCentriCompMode(void)
{
	return modbusGet(GET_CENTRI_COMP_MODE);
}

bool LpmsIoInterface::setCentriCompMode(int v)
{
	return modbusSetInt32(SET_CENTRI_COMP_MODE, v);
}

bool LpmsIoInterface::getCanConfiguration(void)
{
	return modbusGet(GET_CAN_CONFIGURATION);
}

bool LpmsIoInterface::setCanChannelMode(int v)
{
	return modbusSetInt32(SET_CAN_CHANNEL_MODE, v);
}

bool LpmsIoInterface::setCanPointMode(int v)
{
	return modbusSetInt32(SET_CAN_POINT_MODE, v);
}

bool LpmsIoInterface::setCanStartId(int v)
{
	return modbusSetInt32(SET_CAN_START_ID, v);
}
