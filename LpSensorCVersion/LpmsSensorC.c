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

#include "LpmsSensorC.h"

const float pi = 3.141592f;

#define WAIT_AFTER_CONNECT 500000
#define STATUS_PERIOD 500000
#define WAIT_IAP_WRITE_TIME 3000000
#define STREAM_N_PREPARE 100
#define FIRMWARE_BACKUP_FILE "LpmsFirmwareBackupFile.txt"

char deviceId[32];
char iapFilename[64];
char firmwareFilename[64];

int state;
int deviceType;
int getConfigState;
int sensorStatus;
int connectionStatus;
int stopped;
int paused;
int latencyCounter;
int newFieldMap;
int isMagCalibrationEnabled;
int isGetMisalign;
int isGetGyrMisalign;
int misalignSetIndex;
int isSelectQuaternion;
int isSelectEuler;
int isSelectLinAcc;
int retrialsCommandMode;
int retrialsConnect;
int misalignSamples;
int isGetGyrTempCal;
int prevDataSelection;
int prepareStream;
int isFirmwareUpdated;
int isSaveData;
int callbackSet;
int isMagMisalignCalEnabled;
int isAutoMagMisalignCalEnabled;
int cumulatedRefCounter;
int isRefCalibrationEnabled;
int isPlanarMagCalibrationEnabled;
int saveDataPreroll;
int frameCounterOffset;
int lpmsSensorReceivedNewData;

CalibrationData lpmsSensorConfigData;

/* MicroMeasure lpmsTimer;
MicroMeasure statusTimer; */

float frameTime;
float currentFps;
float q0;
float q1;
float q2;
float q3;
float accLatency;
float avgLatency;
float magCalibrationDuration;
float misalignTime;
float gyrAvgTemp[N_GYR_TEMPCAL_SETS];
float gTemp;
float cumulatedRefData[3];
float refCalibrationDuration;
float lpmsSensorTimestampOffset;
float currentSyncOffset;

long lpmsSensorConfigReg;

unsigned long frameNo;

ImuData currentData;

LpVector3f aRaw;
LpVector3f bRaw;
LpVector3f gRaw;
LpVector3f a;
LpVector3f b;
LpVector3f g;
LpVector3f misalignAData[N_ALIGNMENT_SETS];
LpVector3f misalignBData[N_ALIGNMENT_SETS];
LpVector3f gyrMisalignAData[N_ALIGNMENT_SETS];
LpVector3f gyrMisalignBData[N_ALIGNMENT_SETS];
LpVector3f magMisalignAData[N_MAG_ALIGNMENT_SETS];
LpVector3f magmisalignBData[N_MAG_ALIGNMENT_SETS];
LpVector3f misalignADataAcc;
LpVector3f gyrMisalignADataAcc;
LpVector3f gyrTempCalData[N_GYR_TEMPCAL_SETS];
LpVector3f bMax;
LpVector3f bMin;

LpVector4f qOffset;
LpVector4f currentQ;
LpVector4f qAfterOffset;

LpmsCallback lpmsCallback;

// GaitTracking gm;
// stdofstream *saveDataHandle;



/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
void lpmsSensorInit(int dt, const char *did)
{
	strcpy(deviceId, did);
	deviceType = dt;

	lpmsInitCalibrationData(&lpmsSensorConfigData);
	
	lpmsSetParameterString(&lpmsSensorConfigData, PRM_DEVICE_ID, deviceId);
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_DEVICE_TYPE, deviceType);
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_OPENMAT_ID, 1);

	lpmsInitIoInterface(&lpmsSensorConfigData);
	
	paused = 0;
	stopped = 0;
	
	lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
	lpmsSensorSetConnectionStatus(SENSOR_CONNECTION_CONNECTING);
	
	frameNo = 0;
	lpmsSensorConfigReg = 0;
	
	state = STATE_CONNECT;
	retrialsConnect = 0;	

	isGetMisalign = 0;
	isGetGyrMisalign = 0;
	misalignSetIndex = 0;
	isFirmwareUpdated = 0;
	isSaveData = 0;
	callbackSet = 0;
	isMagCalibrationEnabled = 0;
	isGetGyrTempCal = 0;
	isPlanarMagCalibrationEnabled = 0;
	isRefCalibrationEnabled	= 0;
	lpmsSensorTimestampOffset = 0.0f;
	frameCounterOffset = 0;
	currentSyncOffset = 0.0f;
	lpmsSensorReceivedNewData = 0;
	
	lpmsZeroImuData(&currentData);
	
	LOGV("[LpmsSensor] Initialized LpmsSensor.\n");
}

void lpmsDeinitSensor(void)
{	
	lpmsClose();
}



/***********************************************************************
** POLL / UPDATE DATA FROM SENSORS
***********************************************************************/
void lpmsSensorPollData(void)
{
	if (lpmsDeviceStarted() == 1) {		
		lpmsPollData();
		lpmsCheckState();
	}
}

void lpmsSensorAssertConnected(void)
{
	if (lpmsDeviceStarted() == 0) {	
		lpmsClose();
		state = STATE_CONNECT;
		LOGV("[LpmsSensor] Re-connecting..\n");	
	}
}

void lpmsSensorUpdate(void)
{
	ImuData imuData;
	int p;
	int pa[64];
	LpVector4f q;
	LpMatrix3x3f m;
		
	switch (state) {
	case STATE_CONNECT:
		lpmsSensorSetConnectionStatus(SENSOR_CONNECTION_CONNECTING);
		lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
		
		// lpmsTimer.reset();		
		
		lpmsConnect(deviceId);
		
		LOGV("[LpmsSensor] Trying to connect..\n");
		
		retrialsCommandMode = 0;
		prepareStream = 0;
		
		state = STATE_WAIT_CONNECT;		
	break;
		
	case STATE_WAIT_CONNECT:
		if (/* lpmsTimer.measure() < lpmsgetConnectWait() && */ lpmsDeviceStarted() == 0) {
			state = STATE_WAIT_CONNECT;
		} else {
			if (lpmsDeviceStarted() == 0) {		
				lpmsSensorSetConnectionStatus(SENSOR_CONNECTION_FAILED);
				state = STATE_CONNECT;		
				LOGV("[LpmsSensor] Connection failed (timeout)..\n");			
			} else {			
				state = STATE_WAIT_AFTER_CONNECT;
			}
		}
	break;
	
	// Waits for a certains period after the connect 
	case STATE_WAIT_AFTER_CONNECT:
		/* if (lpmsTimer.measure() > WAIT_AFTER_CONNECT) {
			lpmsTimer.reset(); */
			LOGV("[LpmsSensor] Waiting after connect..\n");
			
			/* if (deviceType == DEVICE_LPMS_BLE) {
				state = STATE_MEASURE;
				setSensorStatus(SENSOR_STATUS_RUNNING);			
			} else { */
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GOTO_COMMAND_MODE;
			/* }
		} */
	break;

	// Retrieves the current parameter settings of the sensor
	case STATE_GET_SETTINGS:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			// lpmsTimer.reset();
			
			switch (getConfigState) {
			// Switches to command mode.
			case C_STATE_GOTO_COMMAND_MODE:
				LOGV("[LpmsSensor] Switch to command mode\n");
				lpmsSetCommandMode();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_FIRMWARE_VERSION;
			break;
			
			// Retrieves firmware version
			case C_STATE_GET_FIRMWARE_VERSION:
				LOGV("[LpmsSensor] Get firmware version\n");
				lpmsGetFirmwareVersion();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_CONFIG;			
			break;			
			
			// Gets the current configuration word. 
			case C_STATE_GET_CONFIG:
				LOGV("[LpmsSensor] Get configuration data\n");
				lpmsGetConfig();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_FILTER_MODE;
			break;
			
			// Retrieves the current filter mode.
			case C_STATE_FILTER_MODE:
				LOGV("[LpmsSensor] Get filter mode\n");			
				lpmsGetFilterMode();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_FILTER_PRESET;			
			break;

			// Retrieves the current filter parameter preset.
			case C_STATE_FILTER_PRESET:
				LOGV("[LpmsSensor] Get filter preset\n");			
				lpmsGetFilterPreset();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GYR_RANGE;			
			break;			
			
			// Retrieves the current gyroscope range.
			case C_STATE_GYR_RANGE:
				LOGV("[LpmsSensor] Get gyr. range parameters\n");
				lpmsGetGyrRange();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_ACC_RANGE;			
			break;
			
			// Retrieves the current accelerometer range.
			case C_STATE_ACC_RANGE:
				LOGV("[LpmsSensor] Get acc. range parameters\n");
				lpmsGetAccRange();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_MAG_RANGE;
			break;

			// Retrieves the current magnetometer range.
			case C_STATE_MAG_RANGE:
				LOGV("[LpmsSensor] Get mag. range parameters\n");
				lpmsGetMagRange();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_IMU_ID;			
			break;

			// Retrieves the current IMU ID.
			case C_STATE_IMU_ID:
				LOGV("[LpmsSensor] Get IMU ID\n");
				lpmsGetImuId();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_HARD;			
			break;	

			// Retrieves hard iron matrix.
			case C_STATE_GET_HARD:
				LOGV("[LpmsSensor] Get hard iron offset\n");
				lpmsGetHardIronOffset();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_SOFT;			
			break;

			// Retrieves hard soft matrix.
			case C_STATE_GET_SOFT:
				LOGV("[LpmsSensor] Get soft iron matrix\n");
				lpmsGetSoftIronMatrix();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_ESTIMATE;			
			break;
			
			// Retrieves field estimate.
			case C_STATE_GET_ESTIMATE:
				LOGV("[LpmsSensor] Get field estimate\n");
				lpmsGetFieldEstimate();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_MAG_ALIGNMENT_MATRIX;
			break;
			
			// Retrieves magnetometer alignment matrix.
			case C_STATE_GET_MAG_ALIGNMENT_MATRIX:
				LOGV("[LpmsSensor] Get magnetometer alignment matrix\n");
				lpmsGetMagAlignmentMatrix();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_MAG_ALIGNMENT_BIAS;
			break;			

			// Retrieves magnetometer alignment matrix.
			case C_STATE_GET_MAG_ALIGNMENT_BIAS:
				LOGV("[LpmsSensor] Get magnetometer alignment bias\n");
				lpmsGetMagAlignmentBias();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_MAG_REFERENCE;
			break;			

			// Retrieves magnetometer alignment matrix.
			case C_STATE_GET_MAG_REFERENCE:
				LOGV("[LpmsSensor] Get magnetometer reference\n");
				lpmsGetMagReference();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_ACC_ALIGNMENT;
			break;			
			
			// Retrieves accelerometer alignment matrix.
			case C_STATE_GET_ACC_ALIGNMENT:
				LOGV("[LpmsSensor] Get acc. alignment matrix\n");
				lpmsGetAccAlignment();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_ACC_BIAS;			
			break;	

			// Retrieves accelerometer bias.
			case C_STATE_GET_ACC_BIAS:
				LOGV("[LpmsSensor] Get acc. bias\n");
				lpmsGetAccBias();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_GYR_ALIGNMENT;			
			break;
			
			// Retrieves gyroscope alignment matrix.
			case C_STATE_GET_GYR_ALIGNMENT:
				LOGV("[LpmsSensor] Get gyr. alignment matrix\n");
				lpmsGetGyrAlignment();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_GYR_ALIGNMENT_BIAS;			
			break;

			// Retrieves gyroscope alignment bias
			case C_STATE_GET_GYR_ALIGNMENT_BIAS:
				LOGV("[LpmsSensor] Get gyr. alignment bias\n");
				lpmsGetGyrAlignmentBias();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_LOW_PASS;
			break;
			
			// Retrieves low-pass filter settings
			case C_STATE_GET_LOW_PASS:
				LOGV("[LpmsSensor] Get low-pass filter settings\n");
				lpmsGetRawDataLpFilter();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_CAN_MAPPING;			
			break;
			
			// Retrieves CAN bus mapping
			case C_STATE_GET_CAN_MAPPING:
				LOGV("[LpmsSensor] Get CANopen mapping\n");
				lpmsGetCanMapping();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_CAN_HEARTBEAT;			
			break;
			
			// Retrieves CANopen heartbeat timing
			case C_STATE_GET_CAN_HEARTBEAT:
				LOGV("[LpmsSensor] Get CANopen heartbeat timing\n");
				lpmsGetCanHeartbeat();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_LIN_ACC_COMP_MODE;			
			break;
			
			// Retrieves CANopen heartbeat timing
			case C_STATE_GET_LIN_ACC_COMP_MODE:
				LOGV("[LpmsSensor] Get linear acceleration compensation mode\n");
				lpmsGetLinAccCompMode();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_CENTRI_COMP_MODE;			
			break;
			
			// Retrieves CANopen heartbeat timing
			case C_STATE_CENTRI_COMP_MODE:
				LOGV("[LpmsSensor] Get centripetal acceleration compensation mode\n");
				lpmsGetCentriCompMode();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_CAN_CONFIGURATION;			
			break;
			
			// Retrieves CAN configuration
			case C_STATE_GET_CAN_CONFIGURATION:
				LOGV("[LpmsSensor] Get CAN configuration\n");
				lpmsGetCanConfiguration();
				state = STATE_GET_SETTINGS;
				getConfigState = C_RESET_SENSOR_TIMESTAMP;		
			break;

			// Resets sensor timestamp
			case C_RESET_SENSOR_TIMESTAMP:
				LOGV("[LpmsSensor] Resetting timestamp\n");
				lpmsSetTimestamp(0.0f);
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_SETTINGS_DONE;		
			break;				
		
			// Resets the timer and retrieves the field map (soft/hard iron calibration parameters)
			case C_STATE_SETTINGS_DONE:	
				LOGV("[LpmsSensor] Done reading configuration\n");			
			
				/* lpmsTimer.reset();
				statusTimer.reset(); */
				
				newFieldMap = 1;		
					
				// lpmsSensorStartStreaming();
				
				lpmsSensorSetSensorStatus(SENSOR_STATUS_RUNNING);	
				
				retrialsConnect = 0;
				retrialsCommandMode = 0;
				
				state = STATE_MEASURE;
				
				lpmsPrintParameters(&lpmsSensorConfigData);
				
				if (isFirmwareUpdated == 1) {
					lpmsSensorLoadCalibrationData(FIRMWARE_BACKUP_FILE);
					isFirmwareUpdated = 0;
				}
			break;
			} 
		}
		
		// lpmsSensorAssertConnected();
	break;	
		
	// Main measurement state
	case STATE_MEASURE:
		lpmsSensorAssertConnected();		
		
		// LOGV("[LpmsSensor] STATE MEASURE");	
		// Start next measurement step only if program is not waiting for data or ACK.
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			if (lpmsGetMode() != SELECT_LPMS_MODE_STREAM) {
				lpmsSetStreamMode();
				prepareStream = 0;
			}
		}
		
		// TODO: Insert error handling for sensor.
		/* if (lpmsisError() == 1) {
			setSensorStatus(SENSOR_STATUS_ERROR);
		} */

		if (paused == 1) {
			break;
		}
		
		if (prepareStream < STREAM_N_PREPARE) {
			++prepareStream;
			break;
		}
				
		// Load current data from hardware and calculate rotation matrix and Euler angle.		
		if (lpmsGetLatestImuData(&imuData) == 0) break;
	
		lpmsSensorReceivedNewData = 1;
		
		/* frameTime = lpmsTimer.measure() / 1000.0f;	
		lpmsTimer.reset(); */
		lpmsSensorSetFps(frameTime);		
		
		convertArrayToLpVector4f(imuData.q, &q);
		quaternionToMatrix(&q, &m);
		convertLpMatrixToArray(&m, imuData.rotationM);		

		// Add frame number timestamp and IMU ID to current ImuData.
		++frameNo;
		imuData.frameCount = frameNo;	
		imuData.openMatId = lpmsSensorConfigData.openMatId;				

		lpmsSensorSetConnectionStatus(SENSOR_CONNECTION_CONNECTED);
		if (isMagCalibrationEnabled == 1) {
			lpmsSensorSetSensorStatus(SENSOR_STATUS_CALIBRATING);
		} else {
			if (paused == 0) {
				lpmsSensorSetSensorStatus(SENSOR_STATUS_RUNNING);
			} else {
				lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
			}
		}
		
		convertArrayToLpVector3f(imuData.aRaw, &aRaw);
		convertArrayToLpVector3f(imuData.bRaw, &bRaw);
		convertArrayToLpVector3f(imuData.gRaw, &gRaw);

		// Corrects magnetometer measurement
		if ((lpmsGetConfigReg() & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			vectSub3x1(&lpmsSensorConfigData.hardIronOffset, &bRaw, &b);		
			matVectMult3(&lpmsSensorConfigData.softIronMatrix, &b, &b);
		} else {
			vectZero3x1(&b);
		}

		// Corrects accelerometer measurement
		if ((lpmsGetConfigReg() & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {		
			matVectMult3(&lpmsSensorConfigData.misalignMatrix, &aRaw, &a);
			vectAdd3x1(&lpmsSensorConfigData.accBias, &a, &a);
		} else {
			vectZero3x1(&a);
		}

		g = gRaw;
	
		convertLpVector3fToArray(&a, imuData.a);
		convertLpVector3fToArray(&b, imuData.b);
		convertLpVector3fToArray(&g, imuData.g);
	
		// Checks, if calibration is active. 
		lpmsSensorCheckMagCal(frameTime);
		lpmsSensorCheckPlanarMagCal(frameTime);
		lpmsSensorCheckMisalignCal(frameTime);
		lpmsSensorCheckGyrMisalignCal(frameTime);
		lpmsSensorCheckMagMisalignCal(frameTime);
		lpmsSensorCheckMagReferenceCal(frameTime);
		lpmsSensorCheckAutoMagMisalignCal(frameTime);
		
		/* if ((lpmsGetConfigReg() & LPMS_GAIT_TRACKING_ENABLED) != 0) {
			gm.update(&imuData);
		} */
		
		// Sets current data.
		lpmsSensorSetCurrentData(imuData);
		
		// Checks, if data saving is active.
		lpmsSensorCheckSaveData();
	break;
	
	// Prepares parameter adjustment by switching to command mode.
	case PREPARE_PARAMETER_ADJUSTMENT:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsSetCommandMode();
			state = getConfigState;
		}
	break;
	
	// Enables / disables gyroscope threshold.
	case STATE_SET_CONFIG:
	case STATE_ENABLE_THRESHOLD:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_GYR_THRESHOLD_ENABLED, &p);
			switch(p) {
			case SELECT_IMU_GYR_THRESH_ENABLED:
				lpmsEnableGyrThres(LPMS_ENABLE_GYR_THRESHOLD);
			break;
			
			default:
				lpmsEnableGyrThres(LPMS_DISABLE_GYR_THRESHOLD);
			break;
			}
			LOGV("[LpmsSensor] Enable / disable threshold\n");
			state = STATE_GYR_AUTOCALIBRATION;
		}	
	break;
	
	// Enables / disables gyroscope autocalibration.
	case STATE_GYR_AUTOCALIBRATION:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_GYR_AUTOCALIBRATION, &p);
			switch(p) {
			case SELECT_GYR_AUTOCALIBRATION_ENABLED:
				lpmsEnableGyrAutocalibration(LPMS_ENABLE_GYR_AUTOCAL);
			break;
			
			default:
				lpmsEnableGyrAutocalibration(LPMS_DISABLE_GYR_AUTOCAL);
			break;
			}
			LOGV("[LpmsSensor] Gyroscope autocalibration on / off\n");
			state = STATE_SET_GYR_RANGE;
		}	
	break;	
	
	// Sets current gyroscope range.
	case STATE_SET_GYR_RANGE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_GYR_RANGE, &p);
			lpmsSetGyrRange(p);
			LOGV("[LpmsSensor] Set gyroscope range\n");
			state = STATE_SET_ACC_RANGE;
		}		
	break;

	// Sets accelerometer range.
	case STATE_SET_ACC_RANGE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_ACC_RANGE, &p);
			lpmsSetAccRange(p);
			LOGV("[LpmsSensor] Set accelerometer range\n");
			state = STATE_SET_MAG_RANGE;
		}	
	break;
	
	// Sets magnetometer range.
	case STATE_SET_MAG_RANGE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_MAG_RANGE, &p);
			lpmsSetMagRange(p);
			LOGV("[LpmsSensor] Set magnetometer range\n");
			state = STATE_SET_FILTER_MODE;
		}
	break;
	
	// Sets current filter mode.
	case STATE_SET_FILTER_MODE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_FILTER_MODE, &p);
			switch(p) {
			case SELECT_FM_GYRO_ONLY:
				lpmsSetFilterMode(LPMS_FILTER_GYR);
			break;
			
			case SELECT_FM_GYRO_ACC:
				lpmsSetFilterMode(LPMS_FILTER_GYR_ACC);			
			break;
			
			case SELECT_FM_GYRO_ACC_MAG:
				lpmsSetFilterMode(LPMS_FILTER_GYR_ACC_MAG);			
			break;
			
			case SELECT_FM_ACC_MAG:
				lpmsSetFilterMode(LPMS_FILTER_ACC_MAG);			
			break;
			
			case SELECT_FM_GYR_ACC_EULER:
				lpmsSetFilterMode(LPMS_FILTER_GYR_ACC_EULER);
			break;
			}
			LOGV("[LpmsSensor] Set filter mode\n");
			state = STATE_SET_PARAMETER_SET;	
		}
	break;	

	// Sets current filter parameter set.
	case STATE_SET_PARAMETER_SET:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_PARAMETER_SET, &p);
			switch(p) {
			case SELECT_IMU_SLOW:
				lpmsSetFilterPreset(LPMS_FILTER_PRM_SET_1);	
			break;
			
			case SELECT_IMU_MEDIUM:
				lpmsSetFilterPreset(LPMS_FILTER_PRM_SET_2);	
			break;
			
			case SELECT_IMU_FAST:
				lpmsSetFilterPreset(LPMS_FILTER_PRM_SET_3);	
			break;	

			case SELECT_IMU_DYNAMIC:
				lpmsSetFilterPreset(LPMS_FILTER_PRM_SET_4);	
			break;				
			}
			LOGV("[LpmsSensor] Set parameter set\n");
			state = STATE_SET_LP_FILTER;
		}
	break;
	
	// Sets current low-pass filter setting.
	case STATE_SET_LP_FILTER:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_LOW_PASS, &p);
			lpmsSetRawDataLpFilter(p);
			LOGV("[LpmsSensor] Set low-pass filter\n");
			state = STATE_SET_OPENMAT_ID;
		}
	break;	

	// Sets OpenMAT ID.
	case STATE_SET_OPENMAT_ID:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_OPENMAT_ID, &p);
			lpmsSetImuId(p);
			LOGV("[LpmsSensor] Set OpenMAT ID\n");
			state = STATE_SET_CAN_BAUDRATE;
		}
	break;
	
	// Sets CAN protocol.
	case STATE_SET_CAN_PROTOCOL:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_CAN_STREAM_FORMAT, &p);
			lpmsSetCanStreamFormat(p);
			LOGV("[LpmsSensor] Set CAN protocol\n");
			state = STATE_SET_CAN_BAUDRATE;
		}
	break;
	
	// Sets CAN baudrate.
	case STATE_SET_CAN_BAUDRATE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_CAN_BAUDRATE, &p);
			lpmsSetCanBaudrate(p);
			LOGV("[LpmsSensor] Set sampling rate\n");
			state = STATE_SET_SAMPLING_RATE;
		}
	break;

	// Sets sampling rate.
	case STATE_SET_SAMPLING_RATE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SAMPLING_RATE, &p);
			lpmsSetStreamFrequency(p);
			LOGV("[LpmsSensor] Set sampling rate\n");
			state = STATE_SET_HARD_IRON_OFFSET;
		}
	break;	
	
	// Sets hard iron offset.
	case STATE_SET_HARD_IRON_OFFSET:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetHardIronOffset(lpmsSensorConfigData.hardIronOffset);
			LOGV("[LpmsSensor] Set hard iron offset\n");
			state = STATE_SET_SOFT_IRON_MATRIX;
		}
	break;

	// Sets soft iron matrix.
	case STATE_SET_SOFT_IRON_MATRIX:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetSoftIronMatrix(lpmsSensorConfigData.softIronMatrix);
			LOGV("[LpmsSensor] Set soft iron matrix\n");
			state = STATE_SET_FIELD_ESTIMATE;
		}
	break;
	
	// Sets field estimate.
	case STATE_SET_FIELD_ESTIMATE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetFieldEstimate(lpmsSensorConfigData.fieldRadius);
			LOGV("[LpmsSensor] Set field estimate\n");
			state = STATE_SET_MAG_ALIGNMENT_MATRIX;
		}
	break;
	
	// Sets magnetometer alignment matrix.
	case STATE_SET_MAG_ALIGNMENT_MATRIX:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetMagAlignmentMatrix(lpmsSensorConfigData.magMAlignmentMatrix);
			LOGV("[LpmsSensor] Set magnetometer alignment matrix\n");
			state = STATE_SET_MAG_ALIGNMENT_BIAS;
		}
	break;	
	
	// Sets magnetometer alignment bias.
	case STATE_SET_MAG_ALIGNMENT_BIAS:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetMagAlignmentBias(lpmsSensorConfigData.magMAlignmentBias);
			LOGV("[LpmsSensor] Set magnetometer alignment bias\n");
			state = STATE_SET_MAG_REFERENCE;
		}
	break;	

	// Sets magnetometer reference.
	case STATE_SET_MAG_REFERENCE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetMagReference(lpmsSensorConfigData.magReference);
			LOGV("[LpmsSensor] Set magnetometer reference\n");
			state = STATE_SET_GYR_ALIGNMENT;
		}
	break;		
	
	// Sets gyroscope alignment.
	case STATE_SET_GYR_ALIGNMENT:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetGyrAlignment(lpmsSensorConfigData.gyrMisalignMatrix);
			LOGV("[LpmsSensor] Set gyroscope alignment\n");
			state = STATE_SET_GYR_ALIGNMENT_BIAS;
		}
	break;
	
	// Sets gyroscope alignment matrix.
	case STATE_SET_GYR_ALIGNMENT_BIAS:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetGyrAlignmentBias(lpmsSensorConfigData.gyrAlignmentBias);
			LOGV("[LpmsSensor] Set gyroscope alignment bias\n");
			state = STATE_SET_ACC_ALIGNMENT;
		}
	break;
	
	// Sets accelerometer alignment.
	case STATE_SET_ACC_ALIGNMENT:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetAccAlignment(lpmsSensorConfigData.misalignMatrix);
			LOGV("[LpmsSensor] Set acclerometer alignment\n");
			state = STATE_SET_ACC_BIAS;
		}
	break;
	
	// Sets accelerometer bias
	case STATE_SET_ACC_BIAS:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetAccBias(lpmsSensorConfigData.accBias);
			LOGV("[LpmsSensor] Set accelerometer bias\n");
			state = STATE_SET_CAN_MAPPING;
		}
	break;
	
	// Sets CANopen mapping
	case STATE_SET_CAN_MAPPING:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_CAN_MAPPING, pa);
			lpmsSetCanMapping(pa);
			state = STATE_SET_CAN_HEARTBEAT;
		}
	break;
	
	// Sets CANopen heartbeat timing
	case STATE_SET_CAN_HEARTBEAT:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetCanHeartbeat(lpmsSensorConfigData.canHeartbeat);
			LOGV("[LpmsSensor] Set CAN bus heartbeat timing\n");
			state = STATE_SET_LIN_ACC_COMP_MODE;
		}
	break;

	// Sets linear acceleration compensation mode
	case STATE_SET_LIN_ACC_COMP_MODE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_LIN_ACC_COMP_MODE, &p);
			switch(p) {
			case SELECT_LPMS_LIN_ACC_COMP_MODE_OFF:
				lpmsSetLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_OFF);	
			break;
			
			case SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK:
				lpmsSetLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_WEAK);	
			break;
			
			case SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM:
				lpmsSetLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_MEDIUM);	
			break;	

			case SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG:
				lpmsSetLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_STRONG);	
			break;				
			
			case SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA:
				lpmsSetLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_ULTRA);	
			break;			
			}
		
			LOGV("[LpmsSensor] Set linear acceleration compensation mode\n");
			state = STATE_SET_CENTRI_COMP_MODE;
		}
	break;	

	// Sets centripetal acceleration compensation mode
	case STATE_SET_CENTRI_COMP_MODE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetCentriCompMode(lpmsSensorConfigData.centriCompMode);
			LOGV("[LpmsSensor] Set centripetal acceleration compensation mode\n");
			state = STATE_SET_CAN_CHANNEL_MODE;
		}
	break;
	
	// Sets CAN channel mode
	case STATE_SET_CAN_CHANNEL_MODE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_CAN_CHANNEL_MODE, &p);
			switch(p) {
			case SELECT_CAN_CHANNEL_MODE_CANOPEN:
				lpmsSetCanChannelMode(0);
			break;
			
			case SELECT_CAN_CHANNEL_MODE_SEQUENTIAL:
				lpmsSetCanChannelMode(LPMS_CAN_SEQUENTIAL_MODE);
			break;
			}
			LOGV("[LpmsSensor] Set CAN channel mode\n");
			state = STATE_SET_CAN_POINT_MODE;
		}
	break;	
	
	// Sets CAN point mode
	case STATE_SET_CAN_POINT_MODE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_CAN_POINT_MODE, &p);
			switch(p) {
			case SELECT_CAN_POINT_MODE_FLOATING:
				lpmsSetCanPointMode(0);
			break;
			
			case SELECT_CAN_POINT_MODE_FIXED:
				lpmsSetCanPointMode(LPMS_CAN_FIXEDPOINT_MODE);
			break;
			}		
			LOGV("[LpmsSensor] Set CAN point mode\n");
			state = STATE_SET_CAN_START_ID;
		}
	break;
	
	// Sets CAN point mode
	case STATE_SET_CAN_START_ID:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetCanStartId(lpmsSensorConfigData.canStartId);
			LOGV("[LpmsSensor] Set CAN start ID\n");
			state = STATE_SET_LPBUS_DATA_MODE;
		}
	break;
	
	// Sets CAN point mode
	case STATE_SET_LPBUS_DATA_MODE:
		/* if (assertFwVersion(1, 3, 0) == 0) {
			state = STATE_SELECT_DATA;
			break;
		} */
			
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_LPBUS_DATA_MODE, &p);
			switch(p) {
			case SELECT_LPMS_LPBUS_DATA_MODE_32:
				lpmsSetLpBusDataMode(LPMS_LPBUS_DATA_MODE_32);
			break;
			
			case SELECT_LPMS_LPBUS_DATA_MODE_16:
				lpmsSetLpBusDataMode(LPMS_LPBUS_DATA_MODE_16);
			break;
			}
			LOGV("[LpmsSensor] Set LP-BUS data mode\n");
			state = STATE_SELECT_DATA;
		}
	break;	
		
	// Selects transmission data
	case STATE_SELECT_DATA:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &p);
			lpmsSelectData(p);
			printf("[LpmsSensor] Select data 0x%x\n", p);
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
		
	// Writes parameters to the sensor flash memory.
	case STATE_WRITE_PARAMETERS:	
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsWriteRegisters();
			lpmsSensorSetSensorStatus(SENSOR_STATUS_CALIBRATING);
			
			state = STATE_MEASURE;
		}
	break;
	
	// Starts uploading firmware.
	case STATE_UPLOAD_FIRMWARE:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsStartUploadFirmware(firmwareFilename);		
			state = STATE_CHECK_FIRMWARE_UPLOAD;
			lpmsSensorSetSensorStatus(SENSOR_STATUS_UPLOADING);
		}
	break;
	
	// Starts checking state of firmware upload.
	case STATE_CHECK_FIRMWARE_UPLOAD:
		if (lpmsGetCurrentState() == UPDATE_FIRMWARE) {
		} else {
			state = STATE_WAIT_FIRMWARE_WRITE;
			// lpmsTimer.reset();
		}
	break;
	
	// Waits for firmware writing to be finished (10s).
	case STATE_WAIT_FIRMWARE_WRITE:
		// if (lpmsTimer.measure() > WAIT_FIRMWARE_WRITE_TIME) {
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GOTO_COMMAND_MODE;
			if (paused == 0) {
				isFirmwareUpdated = 1;
				lpmsSensorSetSensorStatus(SENSOR_STATUS_RUNNING);
			} else {
				lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
			}
		// }
	break;	

	// Starts uploading the IAP.
	case STATE_UPLOAD_IAP:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsStartUploadIap(iapFilename);		
			state = STATE_CHECK_IAP_UPLOAD;
			lpmsSensorSetSensorStatus(SENSOR_STATUS_UPLOADING);
		}
	break;
	
	// Starts checking the status of the IAP upload.
	case STATE_CHECK_IAP_UPLOAD:
		if (lpmsGetCurrentState() == UPDATE_IAP) {
		} else {
			state = STATE_WAIT_IAP_WRITE;
			// lpmsTimer.reset();
		}
	break;
	
	// Waits for IAP writing to be finished.
	case STATE_WAIT_IAP_WRITE:
		// if (lpmsTimer.measure() > WAIT_IAP_WRITE_TIME) {
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GOTO_COMMAND_MODE;
			if (paused == 0) {
				lpmsSensorSetSensorStatus(SENSOR_STATUS_RUNNING);
			} else {
				lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
			}
		// }
	break;
	
	// Initiates self-test.
	case STATE_SET_SELF_TEST:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELF_TEST, &p);
			lpmsSetSelfTest(p);
			state = STATE_MEASURE;			
		}
	break;
	
	// Retrieves communication latency.
	case STATE_GET_LATENCY:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {		
			lpmsSensorSetSensorStatus(SENSOR_STATUS_CALIBRATING);
		
			lpmsGetConfig();
			
			if (latencyCounter == 0) {
				accLatency = 0.0f;
			} else if (latencyCounter > 0 && latencyCounter < 50) {
				// accLatency += lpmsGetLatestLatency();
			} else {
				avgLatency = accLatency / (float)(latencyCounter-1) / 2.0f;
				// stdcout << "[LpmsSensor] Average latency: " << avgLatency << "ms" << stdendl;
				// setParameter(&lpmsSensorConfigData, PRM_SELF_TEST, &p);
				state = STATE_MEASURE;	
			}
			
			latencyCounter++;
		}
	break;
	
	// Starts gyroscope calibration
	case STATE_CALIBRATE_GYRO:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {	
			lpmsStartGyrCalibration();

			state = STATE_CALIBRATING;
			getConfigState = CAL_STATE_GET_STATUS;
		}
	break;
	
	// Waits for calibration to finish.
	case STATE_CALIBRATING:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			switch (getConfigState) {
			case CAL_STATE_GET_STATUS:
				lpmsGetStatus();

				lpmsSensorSetSensorStatus(SENSOR_STATUS_CALIBRATING);

				getConfigState = CAL_STATE_WAIT_FINISH;
			break;

			case CAL_STATE_WAIT_FINISH:
				if (lpmsIsCalibrating() == 0) {
					state = STATE_MEASURE;
				} // else if (statusTimer.measure() > STATUS_PERIOD) {
				//	lpmsGetStatus();
				//	statusTimer.reset();
				//}
			}
		}
	break;
	
	// Resets offset.
	case STATE_RESET_ORIENTATION_OFFSET:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSensorResetOrientationOffset();
			state = STATE_SET_CONFIG;
		}	
	break;
	
	// Sets offset.
	case STATE_SET_ORIENTATION_OFFSET:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSensorSetOrientationOffset();
			state = STATE_SET_CONFIG;
		}	
	break;	
	
	// Restores factory defaults.
	case STATE_RESET_TO_FACTORY_DEFAULTS:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsRestoreFactoryValues();
			LOGV("[LpmsSensor] Reset factory defaults\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
	
	// Resets sensor timestamp.
	case STATE_RESET_TIMESTAMP:
		if (lpmsIsWaitForData() == 0 && lpmsIsWaitForAck() == 0) {
			lpmsSetTimestamp(0.0f);
			LOGV("[LpmsSensor] Reset sensor timestamp\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;	
	
	// Error state.
	case STATE_NONE:
		state = STATE_NONE;
	break;
	
	default:
	break;
	}
}



/***********************************************************************
** DIRECT GET / SET DEVICE PARAMETERS
***********************************************************************/
void lpmsSensorGetDeviceId(char *str)
{
	char deviceId[64];

	lpmsGetParameterString(&lpmsSensorConfigData, PRM_DEVICE_ID, deviceId);
	
	strcpy(str, deviceId);
}

CalibrationData *lpmsSensorGetConfigurationData(void)
{
	return &lpmsSensorConfigData;
}

int lpmsSensorAssertFwVersion(int d0, int d1, int d2)
{
	if (lpmsSensorConfigData.firmwareVersionDig0 > d0) return 1;
	if (lpmsSensorConfigData.firmwareVersionDig0 == d0 && lpmsSensorConfigData.firmwareVersionDig1 > d1) return 1;
	if (lpmsSensorConfigData.firmwareVersionDig0 == d0 && lpmsSensorConfigData.firmwareVersionDig1 == d1 && lpmsSensorConfigData.firmwareVersionDig2 >= d2) return 1;
	
	return 0;
}

int lpmsSensorHasNewFieldMap(void)
{
	int f = newFieldMap;
	newFieldMap = 0;
	
	return f;
}

void lpmsSensorSetFps(float f) 
{	
	currentFps = f;
}

float lpmsSensorGetFps(void) 
{	
	return currentFps;
}

void lpmsSensorSetSensorStatus(int s) 
{	
	sensorStatus = s;
}

int lpmsSensorGetSensorStatus(void) 
{	
	return sensorStatus;
}

void lpmsSensorSetConnectionStatus(int s)
{
	connectionStatus = s;
}

int lpmsSensorGetConnectionStatus(void)
{
	return connectionStatus;
}

void lpmsSensorSetCurrentData(ImuData d)
{
	currentData = d;
	if (callbackSet == 1) {
		lpmsCallback(d, deviceId);
	}
}

void lpmsSensorSetCallback(LpmsCallback cb)
{
	lpmsCallback = cb;
	callbackSet = 1;
}

int lpmsSensorGetCurrentData(ImuData *d)
{	
	int r = lpmsSensorReceivedNewData;

	lpmsSensorReceivedNewData = 0;
	*d = currentData;

	return r;
}

void lpmsSensorGetCalibratedSensorData(float g[3], float a[3], float b[3])
{
	int i;

	for (i=0; i<3; i++) g[i] = currentData.g[i];
	for (i=0; i<3; i++) a[i] = currentData.a[i];
	for (i=0; i<3; i++) b[i] = currentData.b[i];
}

void lpmsSensorGetQuaternion(float q[4]) 
{
	int i;

	for (i=0; i<4; i++) q[i] = currentData.q[i];
}

void lpmsSensorGetEulerAngle(float r[3]) 
{
	int i;

	for (i=0; i<3; i++) r[i] = currentData.r[i];
}

void lpmsSensorGetRotationMatrix(float M[3][3]) 
{
	int i, j;

	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			M[i][j] = currentData.rotationM[i*3+j];
		}
	}
}

int lpmsSensorIsRunning(void)
{
	return !paused;
}

void lpmsSensorPause(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	lpmsSensorSetSensorStatus(SENSOR_STATUS_PAUSED);
	paused = 1;
}

void lpmsSensorRun(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	lpmsSensorSetSensorStatus(SENSOR_STATUS_RUNNING);	
	paused = 0;
}

void lpmsSensorClose(void)
{
	lpmsClose();
	
	stopped = 1;
}

void lpmsSensorStartCalibrateGyro(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	if (state != STATE_MEASURE) return;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_CALIBRATE_GYRO;
}

void lpmsSensorSetOrientationOffset(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	if (state != STATE_MEASURE) return;	
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_SET_ORIENTATION_OFFSET;
}

void lpmsSensorResetOrientationOffset(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	if (state != STATE_MEASURE) return;	
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_RESET_ORIENTATION_OFFSET;
}
	
void lpmsSensorSetOpenMatId(int id)
{
	lpmsSensorConfigData.openMatId = id;
}

int lpmsSensorGetOpenMatId(void)
{
	return lpmsSensorConfigData.openMatId;
}

int lpmsSensorUpdateParameters(void)
{
	int r = 1;

	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return 0;
	
	if (state != STATE_MEASURE) {
		return 0;
	}
	
	// if (deviceType == DEVICE_LPMS_BLE) return 0;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;
	getConfigState = STATE_SET_CONFIG;
	
	return r;
}

int lpmsSensorSetConfigurationPrmInt(int parameterIndex, int parameter)
{	
	int f = 1;

	// if (deviceType == DEVICE_LPMS_BLE) return f;	
	
	lpmsSetParameterInt(&lpmsSensorConfigData, parameterIndex, parameter);

	switch (parameterIndex) {
	case PRM_SELF_TEST:
		state = PREPARE_PARAMETER_ADJUSTMENT;	
		getConfigState = STATE_SET_SELF_TEST;
	break;

	default:
		f = lpmsSensorUpdateParameters();
	break;
	} 

	return f;
}

int lpmsSensorSetConfigurationPrmIntArray(int parameterIndex, int *parameter)
{	
	int f = 1;
	
	// if (deviceType == DEVICE_LPMS_BLE) return f;
	
	lpmsSetParameterIntArray(&lpmsSensorConfigData, parameterIndex, parameter);	
	
	f = lpmsSensorUpdateParameters();
	
	return f;
}

int lpmsSensorGetConfigurationPrmInt(int parameterIndex, int* parameter)
{
	lpmsGetParameterInt(&lpmsSensorConfigData, parameterIndex, parameter);

	return 1; 
}

int lpmsSensorGetConfigurationPrmString(int parameterIndex, char* parameter)
{
	char cppStr[64];
	
	lpmsGetParameterString(&lpmsSensorConfigData, parameterIndex, cppStr);
	
	strcpy(parameter, cppStr);
	
	return 1;
}



/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
int lpmsSensorUploadFirmware(const char *fn)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return 0;
	
	lpmsSensorsaveCalibrationData(FIRMWARE_BACKUP_FILE);
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_UPLOAD_FIRMWARE;
	strcpy(firmwareFilename, fn);
	
	return 1;
}

int lpmsSensorUploadIap(const char *fn)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return 0;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_UPLOAD_IAP;
	strcpy(iapFilename, fn);
	
	return 1;
}

int lpmsSensorGetUploadProgress(int *p) 
{
	if (state != STATE_CHECK_IAP_UPLOAD &&
		state != STATE_WAIT_IAP_WRITE &&
		state != STATE_UPLOAD_IAP &&
		state != STATE_CHECK_FIRMWARE_UPLOAD &&
		state != STATE_WAIT_FIRMWARE_WRITE &&
		state != STATE_UPLOAD_FIRMWARE) {
		return 0;
	}
	
	if (lpmsGetUploadProgress(p) == 0) return 2;
	
	/* if (state == STATE_WAIT_FIRMWARE_WRITE) {
		*p = *p + (int) (lpmsTimer.measure() * 100 / WAIT_FIRMWARE_WRITE_TIME);
	} 

	if (state == STATE_WAIT_IAP_WRITE) {
		*p = *p + (int) (lpmsTimer.measure() * 100 / WAIT_IAP_WRITE_TIME);
	} */

	return 1;
}

void lpmsSensorSaveParametersToSensor(void)	
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_WRITE_PARAMETERS;	
}

/* LpmsIoInterface *lpmsSensorGetIoInterface(void)
{
	return (LpmsIoInterface *)bt;
} */

void lpmsSensorMeasureAvgLatency(void) 
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	latencyCounter = 0;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_GET_LATENCY;
}

void lpmsSensorAcquireFieldMap(void) 
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_START_GET_FIELD_MAP;
}

int lpmsSensorGetPressure(float *p)
{
	// sensorMutex.lock();
	*p = currentData.pressure;
	// sensorMutex.unlock();

	return 1;
}

void lpmsSensorgetHardIronOffset(float v[3]) 
{
	int i;

	for (i=0; i<3; i++) {
		v[i] = lpmsSensorConfigData.hardIronOffset.data[i];
	}
}
	
void lpmsSensorgetSoftIronMatrix(float M[3][3], float *fieldRadius) 
{
	int i, j;

	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			M[i][j] = lpmsSensorConfigData.softIronMatrix.data[i][j];
		}
	}
	
	*fieldRadius = lpmsSensorConfigData.fieldRadius;
}

float lpmsSensorgetFieldNoise(void)
{
	return fabs(lpmsSensorConfigData.fieldRadius - sqrtf(b.data[0]*b.data[0] + b.data[1]*b.data[1] + b.data[2]*b.data[2]));
}

void lpmsSensorgetFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]) 
{
	int i, j, k, l;

	for (i=0; i<ABSMAXPITCH; i++) {
		for (j=0; j<ABSMAXROLL; j++) {
			for (k=0; k<ABSMAXYAW; k++) {
				for (l=0; l<3; l++) {
					fieldMap[i][j][k][l] = lpmsSensorConfigData.fieldMap[i][j][k].data[l];
				}
			}
		}
	}
}

void lpmsSensorZeroFieldMap(void)
{
	int i, j, k, l;

	for (i=0; i<ABSMAXPITCH; i++) {
		for (j=0; j<ABSMAXROLL; j++) {
			for (k=0; k<ABSMAXYAW; k++) {
				for (l=0; l<3; l++) {
					lpmsSensorConfigData.fieldMap[i][j][k].data[l] = 0.0f;
				}
			}
		}
	}
}

void lpmsSensorResetToFactorySettings(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_RESET_TO_FACTORY_DEFAULTS;
}

long lpmsSensorGetStreamFrequency(void)
{
	int i;
	int dataSavePeriod;
	
	lpmsSensorGetConfigurationPrmInt(PRM_SAMPLING_RATE, &i);	
	
	switch (i) {
	case SELECT_STREAM_FREQ_5HZ:
		dataSavePeriod = 200;
	break;	
	
	case SELECT_STREAM_FREQ_10HZ:
		dataSavePeriod = 100;
	break;	
	
	case SELECT_STREAM_FREQ_50HZ:		
		dataSavePeriod = 20;				
	break;
		
	case SELECT_STREAM_FREQ_100HZ:
		dataSavePeriod = 10;
	break;
		
	case SELECT_STREAM_FREQ_200HZ:
		dataSavePeriod = 5;
	break;
	
	case SELECT_STREAM_FREQ_500HZ:
		dataSavePeriod = 2;
	break;	
	
	default:
		dataSavePeriod = 2;
	break;
	}
	
	return dataSavePeriod;
}



/***********************************************************************
** CALIBRATION
***********************************************************************/
void lpmsSensorStartPlanarMagCalibration(void)
{
	int p = 0, i;

  	if (isPlanarMagCalibrationEnabled == 1) return;
  	
	isPlanarMagCalibrationEnabled = 1;	
	magCalibrationDuration = 0.0f;

	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
		
	p = prevDataSelection;
	p |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
	p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;

	lpmsSensorSetConfigurationPrmInt(PRM_SELECT_DATA, p);
	
	for (i=0; i<3; i++) {
		bMax.data[i] = -1.0e4f;
		bMin.data[i] = 1.0e4f;
	}
}

#define LPMS_MAG_CALIBRATION_DURATION_20S 20000

void lpmsSensorCheckPlanarMagCal(float T)
{
	int i;

	if (isPlanarMagCalibrationEnabled == 1) {
		magCalibrationDuration += T;
	
		for (i=0; i<3; i++) {
			if (currentData.bRaw[i] > bMax.data[i]) {
				bMax.data[i] = currentData.bRaw[i];
				// printf("[LpmsSensor] New maximum detected: Axis=%d, field=%f\n", i, currentData.bRaw[i]);
			}
			if (currentData.bRaw[i] < bMin.data[i]) {
				bMin.data[i] = currentData.bRaw[i];
				// printf("[LpmsSensor] New minimum detected: Axis=%d, field=%f\n", i, currentData.bRaw[i]);
			}
		}	
		
		if (magCalibrationDuration >= LPMS_MAG_CALIBRATION_DURATION_20S) {
			lpmsSensorStopPlanarMagCalibration();
		}
	} 
}

void lpmsSensorStopPlanarMagCalibration(void)
{
	int i;
	float sqSum = 0;
	LpVector3f bBias;
	LpVector3f bRadius;

  	if (isPlanarMagCalibrationEnabled == 0) return;

	for (i=0; i<3; i++) {
		bBias.data[i] = (bMax.data[i] + bMin.data[i]) / 2.0f;
		// printf("[LpmsSensor] Calculated bias: Axis=%d, field=%f\n", i, bBias.data[i]);
	}
	
	for (i=0; i<3; i++) {
		bRadius.data[i] = bMax.data[i] - bBias.data[i];
		sqSum += bRadius.data[i]; // * bRadius.data[i];
	}
	
	lpmsSensorConfigData.fieldRadius = sqSum / 3; // sqrt(sqSum);
	
	matZero3x3(&lpmsSensorConfigData.softIronMatrix);
	for (i=0; i<3; i++) {
		lpmsSensorConfigData.softIronMatrix.data[i][i] = lpmsSensorConfigData.fieldRadius / bRadius.data[i];
		// printf("[LpmsSensor] Calculated radius: Axis=%d, field=%f\n", i, lpmsSensorConfigData.softIronMatrix.data[i][i]);
		lpmsSensorConfigData.hardIronOffset.data[i] = bBias.data[i];
	}
		
	newFieldMap = 1;
	isPlanarMagCalibrationEnabled = 0;
	magCalibrationDuration = 0.0f;

	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);	

	lpmsSensorUpdateParameters();
}

void lpmsSensorstartMagCalibration(void)
{
	int p;

  	if (isMagCalibrationEnabled == 1) return;
  	
	isMagCalibrationEnabled = 1;	
	magCalibrationDuration = 0.0f;
	
	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
	
	p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
	p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;	
	p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);	
	lpmsSensorUpdateParameters();
	
	bCalInitEllipsoidFit();
}

#define LPMS_MAG_CALIBRATION_DURATION_20S 20000

void lpmsSensorCheckMagCal(float T)
{
	float bInc;
	LpVector3f tV, tV2;
	
	convertArrayToLpVector3f(currentData.bRaw, &tV);
	bCalOrientationFromAccMag(tV, a, &tV2, &bInc);
	
	if (isMagCalibrationEnabled == 1) {
		magCalibrationDuration += T;
		
		bCalUpdateBMap(tV2, tV);
		
		if (magCalibrationDuration >= LPMS_MAG_CALIBRATION_DURATION_20S) {
			lpmsSensorStopMagCalibration();
		}
	} 
}

void lpmsSensorStopMagCalibration(void)
{
	int i, j, k, l;

  	if (isMagCalibrationEnabled == 0) return;
	
	if (bCalFitEllipsoid() == 1) {
		lpmsSensorConfigData.softIronMatrix = bCalGetSoftIronMatrix();
		lpmsSensorConfigData.hardIronOffset = bCalGetHardIronOffset();
		lpmsSensorConfigData.fieldRadius = bCalGetFieldRadius();
	}
	
	for (i=0; i<ABSMAXPITCH; i++) {
		for (j=0; j<ABSMAXROLL; j++) {
			for (k=0; k<ABSMAXYAW; k++) {
				for(l=0; l<3; l++) {
					lpmsSensorConfigData.fieldMap[i][j][k].data[l] = bCalGetFieldMapElement(i, j, k, l);
				}
			}
		}
	}
	
	newFieldMap = 1;	
	isMagCalibrationEnabled = 0;
	magCalibrationDuration = 0.0f;

	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);
	
	lpmsSensorUpdateParameters();
}

#define N_ALIGNMENT_SETS 6

void lpmsSensorinitMisalignCal(void)
{
	int p, i;

  	if (isGetMisalign == 1) return;
  	
	isGetMisalign = 0;
	misalignSetIndex = 0;
	misalignSamples = 0;
	misalignTime = 0.0f;
	
	for(i=0; i<N_ALIGNMENT_SETS; i++) {
		vectZero3x1(&misalignAData[i]);
		vectZero3x1(&misalignBData[i]);
		vectZero3x1(&misalignADataAcc);
	}
	
	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
	
	p = SELECT_LPMS_QUAT_OUTPUT_ENABLED;	
	p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;

	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);
	lpmsSensorUpdateParameters();
}

void lpmsSensorstartGetMisalign(int i) 
{
	if (i < N_ALIGNMENT_SETS) {
		isGetMisalign = 1;
		misalignSetIndex = i;
		misalignSamples = 0;
		misalignTime = 0.0f;
		
		vectZero3x1(&misalignADataAcc);
	}
}

void lpmsSensorCheckMisalignCal(float T)
{
	int i;
	
	if (isGetMisalign == 1) {		
		for(i=0; i<3; i++) {
			misalignADataAcc.data[i] += aRaw.data[i];
		}
		
		++misalignSamples;
				
		for(i=0; i<3; i++) {
			if (aRaw.data[i] > 0.5) {
				misalignBData[misalignSetIndex].data[i] = 1.0f;
			} else if (aRaw.data[i] < -0.5) {
				misalignBData[misalignSetIndex].data[i] = -1.0f;
			} else {
				misalignBData[misalignSetIndex].data[i] = 0.0f;
			}
		}

		misalignTime += T;
		if (misalignTime > 2000.0f) {
			isGetMisalign = 0;

			printf("[LpmsSensor] Average acc. vector %d: ", misalignSetIndex);
			
			for(i=0; i<3; i++) {
				if (misalignSamples == 0) break;
				
				misalignAData[misalignSetIndex].data[i] = misalignADataAcc.data[i] / misalignSamples;
				
				printf("%f ", misalignAData[misalignSetIndex].data[i]);
			}
			printf("\n");
			
			vectZero3x1(&misalignADataAcc);
			
			misalignSamples = 0;
			misalignTime = 0.0f;
		}
	}
}

void lpmsSensorcalcMisalignMatrix(void)
{
	int i;

	float *aTest[N_ALIGNMENT_SETS];
	float *bTest[N_ALIGNMENT_SETS];
	
	for(i=0; i<N_ALIGNMENT_SETS; i++) {
		aTest[i] = misalignAData[i].data;
		bTest[i] = misalignBData[i].data;
	}

	maCalCalcMisalignment(aTest, bTest, &lpmsSensorConfigData.misalignMatrix, &lpmsSensorConfigData.accBias, 6);
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);
	lpmsSensorUpdateParameters();
}	

void lpmsSensorsaveCalibrationData(const char* fn)
{
	printf("[LpmsSensor] Saving calibration data to %s\n", fn);
	// lpmsSensorConfigData.save(fn);
}

void lpmsSensorLoadCalibrationData(const char* fn)
{
	printf("[LpmsSensor] Loading calibration data from %s\n", fn);
	// lpmsSensorConfigData.load(fn);
	lpmsSensorUpdateParameters();
}

void lpmsSensorinitGyrMisalignCal(void)
{
	int p, i;

  	if (isGetGyrMisalign == 1) return;
  	
	isGetGyrMisalign = 0;
	misalignSetIndex = 0;
	misalignSamples = 0;
	misalignTime = 0.0f;

	for(i=0; i<N_ALIGNMENT_SETS; i++) {
		vectZero3x1(&gyrMisalignAData[i]);
		vectZero3x1(&gyrMisalignBData[i]);
		vectZero3x1(&gyrMisalignADataAcc);
	}
		
	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
	
	p = SELECT_LPMS_GYRO_OUTPUT_ENABLED;
	p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);	
	lpmsSensorUpdateParameters();		
}

void lpmsSensorstartGetGyrMisalign(int i) 
{
	if (i < N_ALIGNMENT_SETS) {
		isGetGyrMisalign = 1;
		misalignSetIndex = i;
		misalignSamples = 0;
		misalignTime = 0.0f;
		
		vectZero3x1(&gyrMisalignADataAcc);
	}
}

#define CALC_GYR_MA_DURATION 5000.0f

void lpmsSensorCheckGyrMisalignCal(float T)
{
	int i;

	if (isGetGyrMisalign == 1) {
		for(i=0; i<3; i++) {
			gyrMisalignADataAcc.data[i] += gRaw.data[i];
		}
				
		++misalignSamples;
		misalignTime += T;
		
		if (misalignTime >= CALC_GYR_MA_DURATION) {
			isGetGyrMisalign = 0;
			
			printf("Accumulated gyr. vector %d: ", misalignSetIndex);			

			for(i=0; i<3; i++) {
				if (misalignSamples == 0) break;
				
				gyrMisalignAData[misalignSetIndex].data[i] = gyrMisalignADataAcc.data[i] / misalignSamples;
			}

			for(i=0; i<3; i++) {
				if (gyrMisalignAData[misalignSetIndex].data[i] > 50.0f) {
					gyrMisalignBData[misalignSetIndex].data[i] = 270.0f;
				} else if (gyrMisalignAData[misalignSetIndex].data[i] < -50.0f) {
					gyrMisalignBData[misalignSetIndex].data[i] = -270.0f;
				} else {
					gyrMisalignBData[misalignSetIndex].data[i] = 0.0f;
				}
				printf("%f ", gyrMisalignAData[misalignSetIndex].data[i]);
			}
			printf("\n");
			
			vectZero3x1(&gyrMisalignADataAcc);
			
			misalignSamples = 0;
			misalignTime = 0.0f;
		}
	}
}

void lpmsSensorcalcGyrMisalignMatrix(void)
{
	int i, j;

	const float r2d = 57.2958f;	
	const float d2r = 0.01745f;	

	float *aTest[N_ALIGNMENT_SETS];
	float *bTest[N_ALIGNMENT_SETS];

	for(i=0; i<3; i++) {
		for(j=0; j<3; j++) {	
			gyrMisalignAData[3+i].data[j] = -gyrMisalignAData[i].data[j];
			gyrMisalignBData[3+i].data[j] = -gyrMisalignBData[i].data[j];
		}
	}
	
	for(i=0; i<N_ALIGNMENT_SETS; i++) {
		aTest[i] = gyrMisalignAData[i].data;
		bTest[i] = gyrMisalignBData[i].data;
	}

	maCalCalcGyrMisalignment(aTest, bTest, &lpmsSensorConfigData.gyrMisalignMatrix, &lpmsSensorConfigData.gyrAlignmentBias, 6);
	
	for(i=0; i<3; i++) {
		lpmsSensorConfigData.gyrAlignmentBias.data[i] = lpmsSensorConfigData.gyrAlignmentBias.data[i] * d2r;
	}

	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);	
	lpmsSensorUpdateParameters();	
}

void lpmsSensorresetTimestamp(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_RESET_TIMESTAMP;
	
	frameNo = 0;
}

void lpmsSensorsyncTimestamp(float t)
{
	if ((lpmsGetMode() == SELECT_LPMS_MODE_STREAM) && (lpmsDeviceStarted() == 1) && (state == STATE_MEASURE)) {
		lpmsSetTimestamp(t);
	}
}

void lpmsSensorsetCurrentSyncOffset(float t)
{
	currentSyncOffset = t;
}

float lpmsSensorgetCurrentSyncOffset(void)
{
	return currentSyncOffset;
}

void lpmsSensorstartAutoMagMisalignCal(void)
{
	int p;

  	if (isAutoMagMisalignCalEnabled == 1) return;
  	
	isAutoMagMisalignCalEnabled = 1;	
	misalignTime = 0.0f;
	
	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
	
	p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
	p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;	
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);	
	lpmsSensorUpdateParameters();
	
	printf("[LpmsSensor] Starting magnetometer misalignment calibration.\n");
	
	bMACalInitEllipsoidFit();
}

#define LPMS_REF_CALIBRATION_DURATION_1S	(1000.0f)
#define LPMS_REF_CALIBRATION_DURATION_20S	(20000.0f)

void lpmsSensorCheckAutoMagMisalignCal(float T)
{
	LpVector3f tV, bR;
	LpVector4f tV2;
	
	bR.data[0] = 0.0f;
	bR.data[1] = -1.0f;
	bR.data[2] = -1.0f;
	
	convertArrayToLpVector3f(currentData.b, &tV);
	convertArrayToLpVector4f(currentData.q, &tV2);
	
	if (isAutoMagMisalignCalEnabled == 1) {
		misalignTime += T;
		
		bMAUpdateMap(tV2, tV, lpmsSensorConfigData.magReference);
		
		if (misalignTime >= LPMS_REF_CALIBRATION_DURATION_20S) {
			lpmsSensorCalcMagMisalignCal();
		}
		
		printf("[LpmsSensor] b: %f, %f, %f\n", tV.data[0], tV.data[1], tV.data[2]);
		printf("[LpmsSensor] q: %f, %f, %f, %f\n", tV2.data[0], tV2.data[1], tV2.data[2], tV2.data[3]);
		printf("[LpmsSensor] T: %f\n", misalignTime);		
	} 
}

void lpmsSensorcalcAutoMagMisalignCal(void)
{
	LpMatrix3x3f R;
	LpVector3f t;

 	if (isAutoMagMisalignCalEnabled == 0) return;

	if (bMACalFitEllipsoid(&R, &t) == 1) {
		lpmsSensorConfigData.magMAlignmentMatrix = R;
		lpmsSensorConfigData.magMAlignmentBias = t;
	}
	
	isAutoMagMisalignCalEnabled = 0;
	misalignTime = 0.0f;

	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);
	
	lpmsSensorUpdateParameters();
}

void lpmsSensorstartMagReferenceCal(void)
{
	int p, i;

  	if (isRefCalibrationEnabled == 1) return;
  	
	isRefCalibrationEnabled = 1;
	
	cumulatedRefCounter = 0;
	refCalibrationDuration = 0.0f;
	
	for(i=0; i<3; i++) cumulatedRefData[i] = 0;

	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);
		
	p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
	p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;	
	p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;

	printf("[LpmsSensor] Starting magnetometer reference calibration.\n");	
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);	
	lpmsSensorUpdateParameters();
}

void lpmsSensorCheckMagReferenceCal(float T)
{
	float bInc;
	LpVector3f tR, tV, tV2;

	if (isRefCalibrationEnabled == 1) {
		refCalibrationDuration += T;

		cumulatedRefCounter++;
		
		convertArrayToLpVector3f(currentData.b, &tV);
		convertArrayToLpVector3f(currentData.a, &tV2);		
		
		printf("[LpmsSensor] b: %f, %f, %f\n", tV.data[0], tV.data[1], tV.data[2]);
		printf("[LpmsSensor] a: %f, %f, %f\n", tV2.data[0], tV2.data[1], tV2.data[2]);
		printf("[LpmsSensor] T: %f\n", refCalibrationDuration);
		
		getReferenceYZ(tV, tV2, &tR, &bInc);
		
		cumulatedRefData[0] = cumulatedRefData[0] + bInc;
		cumulatedRefData[1] = cumulatedRefData[1] + tR.data[1];
		cumulatedRefData[2] = cumulatedRefData[2] + tR.data[2];
	
		printf("[LpmsSensor] ref: %f, %f, %f\n", cumulatedRefData[0], cumulatedRefData[1], cumulatedRefData[2]);		
		
		printf("[LpmsSensor] Calibrating magnetometer reference..\n");		
		
		if (refCalibrationDuration >= LPMS_REF_CALIBRATION_DURATION_1S) {
			lpmsSensorCalcMagReferenceCal();
		}	
	}
}

void lpmsSensorCalcMagReferenceCal(void)
{
	float n;

  	if (isRefCalibrationEnabled == 0) return;
	
	lpmsSensorConfigData.magReference.data[0] = 0.0f;
	lpmsSensorConfigData.magReference.data[1] = cumulatedRefData[1] / (float) cumulatedRefCounter;
	lpmsSensorConfigData.magReference.data[2] = cumulatedRefData[2] / (float) cumulatedRefCounter;
	
	n = vect3x1Norm(lpmsSensorConfigData.magReference);
	scalarVectMult3x1(n, &lpmsSensorConfigData.magReference, &lpmsSensorConfigData.magReference);

	printf("[LpmsSensor] Magnetometer reference: %f, %f, %f\n", 0.0f, lpmsSensorConfigData.magReference.data[1], lpmsSensorConfigData.magReference.data[2]);
	printf("[LpmsSensor] Magnetometer reference length: %f\n", n);	
	
	isRefCalibrationEnabled = 0;
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);
	
	lpmsSensorUpdateParameters();	
}

void lpmsSensorinitMagMisalignCal(void)
{
	int p, i;

  	if (isMagMisalignCalEnabled == 1) return;
  	
	isMagMisalignCalEnabled = 0;
	misalignSetIndex = 0;
	misalignSamples = 0;
	misalignTime = 0.0f;
	
	for(i=0; i<N_MAG_ALIGNMENT_SETS; i++) {
		vectZero3x1(&misalignAData[i]);
		vectZero3x1(&misalignBData[i]);
		vectZero3x1(&misalignADataAcc);
	}
	
	lpmsGetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, &prevDataSelection);		
		
	p = SELECT_LPMS_QUAT_OUTPUT_ENABLED;	
	p |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, p);
	lpmsSensorUpdateParameters();
}

void lpmsSensorStartMagMisalignCal(int i) 
{
	if (i < N_MAG_ALIGNMENT_SETS) {
		isMagMisalignCalEnabled = 1;
		misalignSetIndex = i;
		misalignSamples = 0;
		misalignTime = 0.0f;
		
		vectZero3x1(&misalignADataAcc);
	}
}

void lpmsSensorCheckMagMisalignCal(float T)
{
	int i;

	if (isMagMisalignCalEnabled == 1) {		
		for(i=0; i<3; i++) {
			misalignADataAcc.data[i] += bRaw.data[i];
		}
		
		++misalignSamples;
				
		for(i=0; i<3; i++) {
			if (aRaw.data[i] > 50.0f) {
				misalignBData[misalignSetIndex].data[i] = 100.0f;
			} else if (aRaw.data[i] < -50.0f) {
				misalignBData[misalignSetIndex].data[i] = -100.0f;
			} else {
				misalignBData[misalignSetIndex].data[i] = 0.0f;
			}
		}

		misalignTime += T;
		if (misalignTime > 2000.0f) {
			isMagMisalignCalEnabled = 0;

			printf("Average mag. vector %d: ", misalignSetIndex);
			
			for(i=0; i<3; i++) {
				if (misalignSamples == 0) break;
				
				if ((misalignSetIndex % 2) == 0) {				
					misalignAData[misalignSetIndex].data[i] = misalignADataAcc.data[i] / misalignSamples;
				} else {
					misalignAData[misalignSetIndex].data[i] = (misalignADataAcc.data[i] / misalignSamples) - misalignAData[misalignSetIndex-1].data[i];
				}
				
				printf("%f ", misalignAData[misalignSetIndex].data[i]);
			}
			printf("\n");
			
			vectZero3x1(&misalignADataAcc);
			
			misalignSamples = 0;
			misalignTime = 0.0f;
		}
	}
}

void lpmsSensorCalcMagMisalignCal(void)
{
	int i;

	float *aTest[N_MAG_ALIGNMENT_SETS];
	float *bTest[N_MAG_ALIGNMENT_SETS];
	
	for(i=0; i<N_MAG_ALIGNMENT_SETS; i+=2) {
		aTest[i/2] = misalignAData[i+1].data;
		bTest[i/2] = misalignBData[i+1].data;
	}

	maCalCalcMagMisalignment(aTest, bTest, &lpmsSensorConfigData.misalignMatrix, &lpmsSensorConfigData.accBias, N_MAG_ALIGNMENT_SETS/2);
	
	lpmsSetParameterInt(&lpmsSensorConfigData, PRM_SELECT_DATA, prevDataSelection);
	lpmsSensorUpdateParameters();
	
	// printf("updated: %x\n", prevDataSelection);
}



/***********************************************************************
** DATA RECORDING
***********************************************************************/
void lpmsSensorStartSaveData(void /* stdofstream *saveDataHandle */)
{
	/* sensorMutex.lock();
	lpmsclearDataQueue();
	isSaveData = 1;
	this->saveDataHandle = saveDataHandle;
	frameNo = 0;
	saveDataPreroll = 10;
	sensorMutex.unlock(); */
}

void lpmsSensorCheckSaveData(void)
{
	/* float currentTimestamp;

	if (saveDataPreroll > 1) {
		--saveDataPreroll;
		return;
	} else if (saveDataPreroll == 1) {
		--saveDataPreroll;
		lpmsSensorTimestampOffset = currentData.timeStamp;
		frameCounterOffset = currentData.frameCount;	
	}
	
	currentTimestamp = (currentData.timeStamp-lpmsSensorTimestampOffset);

	sensorMutex.lock();
	if (isSaveData == 1 && saveDataHandle->is_open() == 1) {
		*saveDataHandle << currentData.openMatId << ", " << stdfixed << stdsetprecision(3) << currentTimestamp << ", " << (currentData.frameCount-frameCounterOffset) << ", " << currentData.aRaw[0] << ", " << currentData.aRaw[1] << ", " << currentData.aRaw[2] << ", " << currentData.gRaw[0] << ", " << currentData.gRaw[1] << ", " << currentData.gRaw[2] << ", " << currentData.bRaw[0] << ", " << currentData.bRaw[1] << ", " << currentData.bRaw[2] << ", " << currentData.r[0] << ", " << currentData.r[1] << ", " << currentData.r[2] << ", " << currentData.q[0] << ", " << currentData.q[1] << ", " << currentData.q[2] << ", " << currentData.q[3] << ", " << currentData.linAcc[0] << ", " << currentData.linAcc[1] << ", " << currentData.linAcc[2] << ", " << currentData.pressure << ", " << currentData.altitude << ", " << currentData.temperature << ", " << currentData.hm.yHeave << stdendl;
	}
	sensorMutex.unlock(); */
}

void lpmsSensorStopSaveData(void)
{
	isSaveData = 0;
}
