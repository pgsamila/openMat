/***********************************************************************
** Copyright (C) LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "CalibrationDataC.h"

void lpmsInitCalibrationData(CalibrationData* cd)
{ 
	LpMatrix3x3f m;
	LpVector3f v;
	int i, j, k, l;
	
	vectZero3x1(&v);
	createIdentity3x3(&m);
	
	for (i=0; i<ABSMAXPITCH; i++) {
		for (j=0; j<ABSMAXROLL; j++) {
			for (k=0; k<ABSMAXYAW; k++) {
				for (l=0; l<3; l++) {
					cd->fieldMap[i][j][k].data[l] = 0.0f;
				}
			}
		}
	}
	
	cd->firmwareVersion[0] = 0;
	cd->name[0] = 0;
	cd->deviceId[0] = 0;
	cd->openMatId = 1;
	cd->deviceType = 0;
	cd->parameterSet = 0;	
	cd->filterMode = 0;
	cd->gyrThresEnable = 0;
	cd->quaternionCalcLocal = 0;
	cd->samplingRate = 0;	
	cd->gyrRange = 0;
	cd->magRange = 0;
	cd->accRange = 0;
	cd->magAutocalibration = 0;
	cd->canStreamFormat = 0;
	cd->canBaudrate = 0;
	cd->selfTestOn = 0;	
	cd->fieldRadius = 0;
	cd->magThreshold = 0;
	cd->magOutOfRange = 0;
	cd->gyrAutocalibration = 0;	
	cd->hardIronOffset = v;
	cd->softIronMatrix = m;
	cd->misalignMatrix = m;
	cd->accBias = v;
	cd->gyrMisalignMatrix = m;
	cd->gyrAlignmentBias = v;
	cd->lowPassFilter = 0;
	cd->canHeartbeat = 0;
	cd->heavemotionEnabled = 0;
	cd->gaitTrackingEnabled = 0;
	cd->linAccCompMode = 0;
	cd->centriCompMode = 0;
	cd->canPointMode = 0;
	cd->canChannelMode = 0;
	cd->canStartId = 0;	
	cd->selectedData = 0xffff;
}

void lpmsSetParameterString(CalibrationData* cd, int parameterIndex, char* parameter)
{
	switch (parameterIndex) {
	case PRM_NAME:
		strcpy(cd->name, parameter);
	break;
	
	case PRM_DEVICE_ID:
		strcpy(cd->deviceId, parameter);
	break;
	}
}

void lpmsSetParameterInt(CalibrationData* cd, int parameterIndex, int parameter)
{
	switch (parameterIndex) {
	case PRM_OPENMAT_ID:	
		cd->openMatId = parameter;
	break;

	case PRM_DEVICE_TYPE:
		cd->deviceType = parameter;
	break;

	case PRM_GYR_THRESHOLD_ENABLED:
		cd->gyrThresEnable = parameter;
	break;	
		
	case PRM_PARAMETER_SET:
		cd->parameterSet = parameter;
	break;

	case PRM_FILTER_MODE:
		cd->filterMode = parameter;
	break;
	
	case PRM_GYR_RANGE:
		cd->gyrRange = parameter;
	break;
	
	case PRM_MAG_RANGE:
		cd->magRange = parameter;	
	break;
	
	case PRM_ACC_RANGE:
		cd->accRange = parameter;		
	break;
	
	case PRM_LOCAL_Q:	
		cd->quaternionCalcLocal = parameter;
	break;
	
	case PRM_MAG_AUTOCALIBRATION:
		cd->magAutocalibration = parameter;
	break;
	
	case PRM_CAN_STREAM_FORMAT:
		cd->canStreamFormat = parameter;
	break;	
	
	case PRM_CAN_BAUDRATE:
		cd->canBaudrate = parameter;
	break;
	
	case PRM_SAMPLING_RATE:
		cd->samplingRate = parameter;
	break;	
	
	case PRM_SELF_TEST:
		cd->selfTestOn = parameter;
	break;	
	
	case PRM_GYR_AUTOCALIBRATION:		
		cd->gyrAutocalibration = parameter;
	break;
	
	case PRM_SELECT_DATA:
		cd->selectedData = parameter;
	break;
	
	case PRM_LOW_PASS:
		cd->lowPassFilter = parameter;
	break;
	
	case PRM_CAN_HEARTBEAT:
		cd->canHeartbeat = parameter;
	break;
	
	case PRM_HEAVEMOTION_ENABLED:
		cd->heavemotionEnabled = parameter;
	break;
	
	case PRM_GAIT_TRACKING_ENABLED:
		cd->gaitTrackingEnabled = parameter;
	break;

	case PRM_LIN_ACC_COMP_MODE:
		cd->linAccCompMode = parameter;
	break;
	
	case PRM_CENTRI_COMP_MODE:
		cd->centriCompMode = parameter;
	break;
	
	case PRM_CAN_CHANNEL_MODE:
		cd->canChannelMode = parameter;
	break;
	
	case PRM_CAN_POINT_MODE:
		cd->canPointMode = parameter;
	break;
	
	case PRM_CAN_START_ID:
		cd->canStartId = parameter;
	break;
	
	case PRM_LPBUS_DATA_MODE:
		cd->lpBusDataMode = parameter;
	break;	
	}
}

void lpmsSetParameterFloat(CalibrationData* cd, int parameterIndex, float parameter)
{
}

void lpmsSetParameterIntArray(CalibrationData* cd, int parameterIndex, int *parameter)
{	
	int i;

	switch (parameterIndex) {
	case PRM_CAN_MAPPING:
		for (i=0; i<16; ++i) {
			cd->canMapping[i] = parameter[i];
		}
	break;
	}
}

void lpmsGetParameterString(CalibrationData* cd, int parameterIndex, char *parameter)
{
	switch (parameterIndex) {
	case PRM_NAME:
		strcpy(parameter, cd->name);
	break;
	
	case PRM_DEVICE_ID:
		strcpy(parameter, cd->deviceId);
	break;
	
	case PRM_FIRMWARE_VERSION:
		strcpy(parameter, cd->firmwareVersion);
	break;
	}
}

void lpmsGetParameterInt(CalibrationData* cd, int parameterIndex, int *parameter)
{	
	int i;

	switch (parameterIndex) {
	case PRM_OPENMAT_ID:	
		*parameter = cd->openMatId;
	break;

	case PRM_DEVICE_TYPE:	
		*parameter = cd->deviceType;
	break;	

	case PRM_GYR_THRESHOLD_ENABLED:
		*parameter = cd->gyrThresEnable;
	break;	
		
	case PRM_PARAMETER_SET:
		*parameter = cd->parameterSet;
	break;

	case PRM_FILTER_MODE:
		*parameter = cd->filterMode;
	break;
	
	case PRM_GYR_RANGE:
		*parameter = cd->gyrRange;
	break;
	
	case PRM_MAG_RANGE:
		*parameter = cd->magRange;
	break;
	
	case PRM_ACC_RANGE:
		*parameter = cd->accRange;
	break;
	
	case PRM_LOCAL_Q:	
		*parameter = cd->quaternionCalcLocal;
	break;
	
	case PRM_MAG_AUTOCALIBRATION:
		*parameter = cd->magAutocalibration;
	break;

	case PRM_CAN_STREAM_FORMAT:
		*parameter = cd->canStreamFormat;
	break;	
	
	case PRM_CAN_BAUDRATE:
		*parameter = cd->canBaudrate;
	break;	

	case PRM_SAMPLING_RATE:
		*parameter = cd->samplingRate;
	break;

	case PRM_SELF_TEST:
		*parameter = cd->selfTestOn;
	break;	
	
	case PRM_GYR_AUTOCALIBRATION:		
		*parameter = cd->gyrAutocalibration;
	break;
	
	case PRM_SELECT_DATA:
		*parameter = cd->selectedData;
	break;
	
	case PRM_LOW_PASS:
		*parameter = cd->lowPassFilter;
	break;
	
	case PRM_CAN_HEARTBEAT:
		*parameter = cd->canHeartbeat;
	break;
	
	case PRM_HEAVEMOTION_ENABLED:
		*parameter = cd->heavemotionEnabled;
	break;
	
	case PRM_GAIT_TRACKING_ENABLED:
		*parameter = cd->gaitTrackingEnabled;
	break;
	
	case PRM_CAN_MAPPING:
		for (i=0; i<16; ++i) {
			parameter[i] = cd->canMapping[i];
		}
	break;
	
	case PRM_LIN_ACC_COMP_MODE:
		*parameter = cd->linAccCompMode;
	break;
	
	case PRM_CENTRI_COMP_MODE:
		*parameter = cd->centriCompMode;
	break;
	
	case PRM_CAN_CHANNEL_MODE:
		*parameter = cd->canChannelMode;
	break;
	
	case PRM_CAN_POINT_MODE:
		*parameter = cd->canPointMode;
	break;
	
	case PRM_CAN_START_ID:
		*parameter = cd->canStartId;
	break;
	
	case PRM_LPBUS_DATA_MODE:
		*parameter = cd->lpBusDataMode;
	break;
	}
}

void lpmsGetParameterFloat(CalibrationData* cd, int parameterIndex, float *parameter)
{
}

void printMatrix(LpMatrix3x3f m) 
{
	int i, j;

	for (i=0; i<3; i++) {
		printf("[CalibrationData] ");
		for (j=0; j<3; j++) {
			printf("%f ", m.data[i][j]);
		}
		printf("\n");
	}
}

void printVector(LpVector3f v) 
{
	int i;

	printf("[CalibrationData] ");
	for (i=0; i<3; i++) {
		printf("%f ", v.data[i]);
	}
	printf("\n");
}

void lpmsPrintParameters(CalibrationData* cd)
{
	printf("[CalibrationData] DeviceID: %s\n", cd->deviceId);
	printf("[CalibrationData] OpenMAT ID: %d\n", cd->openMatId);
	printf("[CalibrationData] DeviceType: %d\n", cd->deviceType);
	printf("[CalibrationData] Parameter set: %d\n", cd->parameterSet);	
	printf("[CalibrationData] Filter mode: %d\n", cd->filterMode);
	printf("[CalibrationData] Gyroscope threshold: %d\n", cd->gyrThresEnable);
	printf("[CalibrationData] Sampling rate: %d\n", cd->samplingRate);	
	printf("[CalibrationData] Gyro range: %d\n", cd->gyrRange);
	printf("[CalibrationData] Mag. range: %d\n", cd->magRange);
	printf("[CalibrationData] Acc. range: %d\n", cd->accRange);
	printf("[CalibrationData] CAN Baudrate: %d\n", cd->canBaudrate);
	printf("[CalibrationData] Field estimate: %f\n", cd->fieldRadius);
	printf("[CalibrationData] Gyr. auto-calibration on / off: %d\n", cd->gyrAutocalibration);
	printf("[CalibrationData] Hard iron offset:\n");
	printVector(cd->hardIronOffset);
	printf("[CalibrationData] Soft iron matrix:\n");
	printMatrix(cd->softIronMatrix);
	printf("[CalibrationData] Misalignment matrix:\n");
	printMatrix(cd->misalignMatrix);
	printf("[CalibrationData] Accelerometer bias:\n");
	printVector(cd->accBias);
	printf("[CalibrationData] Gyroscope misalignment matrix:\n");
	printMatrix(cd->gyrMisalignMatrix);
	printf("[CalibrationData] Gyroscope alignment bias:\n");
	printVector(cd->gyrAlignmentBias);
	printf("[CalibrationData] Magnetometer misalignment matrix:\n");
	printMatrix(cd->magMAlignmentMatrix);
	printf("[CalibrationData] Magnetometer alignment bias:\n");
	printVector(cd->magMAlignmentBias);
	printf("[CalibrationData] Magnetometer reference:\n");
	printVector(cd->magReference);
	printf("[CalibrationData] Selected data: 0x%d\n", cd->selectedData);
	printf("[CalibrationData] Firmware version %s\n", cd->firmwareVersion);
}
