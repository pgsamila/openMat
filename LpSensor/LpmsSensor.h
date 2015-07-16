/***********************************************************************
** (c) LP-RESEARCH Inc.
** All rights reserved
** Contact: info@lp-research.com
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

#ifndef LPMS_SENSOR
#define LPMS_SENSOR

#include <iostream>
#include <fstream>
#include <mutex>
#include <iomanip>
#include <queue>

#ifdef _WIN32
	#include "windows.h"
#endif

#include "ImuData.h"
#include "MicroMeasure.h"
#include "CalibrationData.h"
#include "LpmsIoInterface.h"
#include "LpmsSensorI.h"
#include "LpMatrix.h"
#include "LpMagnetometerCalibration.h"
#include "CalcMisalignment.h"
#include "GaitTracking.h"
#include "LpMagnetometerMAlignment.h"

#ifdef _WIN32
	#include "LpmsCanIo.h"
	#include "LpmsU.h"
	#include "LpmsBBluetooth.h"	
	#include "LpmsRS232.h"
	#include "LpmsBle2.h"
#endif

#ifdef __GNUC__
	#include "LpmsCanIo.h"
	#include "LpmsU.h"
	#include "LpmsBBluetooth.h"	
	#include "LpmsRS232.h"
#endif

#define STATE_CONNECT 1
#define STATE_GET_SETTINGS 2
#define STATE_WAIT_CONNECT 3
#define STATE_MEASURE 4
#define STATE_NONE 5
#define STATE_CALIBRATE_GYRO 6
#define STATE_CALIBRATE_MAG 7
#define STATE_WAIT_AFTER_CONNECT 8
#define STATE_SET_ORIENTATION_OFFSET 9
#define STATE_SET_REFERENCE 10
#define STATE_ENABLE_THRESHOLD 11
#define STATE_SET_RANGE 12
#define STATE_SET_FILTER_MODE 13 
#define STATE_SET_PARAMETER_SET 14
#define STATE_SET_GYR_RANGE 15
#define STATE_SET_ACC_RANGE 16
#define STATE_SET_MAG_RANGE 17
#define STATE_CHECK_IAP_UPLOAD 18
#define STATE_WAIT_IAP_WRITE 19
#define STATE_UPLOAD_IAP 20
#define STATE_CHECK_FIRMWARE_UPLOAD 21
#define STATE_WAIT_FIRMWARE_WRITE 22
#define STATE_UPLOAD_FIRMWARE 23
#define STATE_SET_OPENMAT_ID 24
#define STATE_MAG_AUTOCALIBRATION 25
#define STATE_WRITE_PARAMETERS 26
#define PREPARE_PARAMETER_ADJUSTMENT 27
#define STATE_SET_CAN_PROTOCOL 28
#define STATE_SET_CAN_BAUDRATE 29
#define STATE_SET_SAMPLING_RATE 30
#define STATE_CALIBRATING 31
#define STATE_SET_SELF_TEST 32
#define STATE_GET_LATENCY 33
#define STATE_GET_FIELD_MAP 34
#define STATE_START_GET_FIELD_MAP 35
#define STATE_GET_HARD_IRON_OFFSET 36
#define STATE_NEW_FIELD_MAP 37
#define STATE_MAG_CALIBRATING 38
#define STATE_RESET_TO_FACTORY_DEFAULTS 39
#define STATE_GYR_AUTOCALIBRATION 40
#define STATE_SET_HARD_IRON_OFFSET 41
#define STATE_SET_SOFT_IRON_MATRIX 42
#define STATE_SET_FIELD_ESTIMATE 43
#define STATE_SET_ACC_ALIGNMENT 44
#define STATE_SET_ACC_BIAS 45
#define STATE_SELECT_DATA 46
#define STATE_SET_GYR_ALIGNMENT 47
#define STATE_SET_GYR_ALIGNMENT_BIAS 48
#define STATE_RESET_GYR_ALIGNMENT 49
#define STATE_RESET_GYR_ALIGNMENT_BIAS 50
#define STATE_RESET_ACC_ALIGNMENT 51
#define STATE_RESET_ACC_BIAS 52
#define STATE_SET_CONFIG 53
#define STATE_SET_GYR_TEMP_CAL_PRM_A 54
#define STATE_SET_GYR_TEMP_CAL_PRM_B 55
#define STATE_SET_GYR_TEMP_CAL_BASE_V 56
#define STATE_SET_GYR_TEMP_CAL_BASE_T 57
#define STATE_SET_LP_FILTER 58
#define STATE_SET_CAN_MAPPING 59
#define STATE_SET_CAN_HEARTBEAT 60
#define STATE_RESET_TIMESTAMP 61
#define STATE_SET_LIN_ACC_COMP_MODE 62
#define STATE_SET_CENTRI_COMP_MODE 63
#define STATE_SET_CAN_CHANNEL_MODE 64
#define STATE_SET_CAN_POINT_MODE 65
#define STATE_SET_CAN_START_ID 66
#define STATE_SET_LPBUS_DATA_MODE 67
#define STATE_SET_MAG_ALIGNMENT_MATRIX 68
#define STATE_SET_MAG_ALIGNMENT_BIAS 69
#define STATE_SET_MAG_REFERENCE 70
#define STATE_RESET_ORIENTATION_OFFSET 71
#define STATE_SELECT_UART_BAUDRATE 72
#define STATE_SELECT_UART_FORMAT 73
#define STATE_ARM_TIMESTAMP_RESET 74

#define C_STATE_GET_CONFIG 1
#define C_STATE_GYR_RANGE 2
#define C_STATE_ACC_RANGE 3
#define C_STATE_MAG_RANGE 4
#define C_STATE_SETTINGS_DONE 5
#define C_STATE_FILTER_MODE 6
#define C_STATE_FILTER_PRESET 7
#define C_STATE_IMU_ID 8
#define C_STATE_GOTO_COMMAND_MODE 9
#define C_STATE_GET_SOFT 10
#define C_STATE_GET_HARD 11
#define C_STATE_GET_ESTIMATE 12
#define C_STATE_GET_ACC_ALIGNMENT 13
#define C_STATE_GET_ACC_BIAS 14
#define C_STATE_SELECT_DATA 15
#define C_STATE_GET_FIRMWARE_VERSION 16
#define C_STATE_GET_GYR_ALIGNMENT 17
#define C_STATE_GET_GYR_ALIGNMENT_BIAS 18
#define C_STATE_GET_GYR_TEMP_CAL_PRM_A 19
#define C_STATE_GET_GYR_TEMP_CAL_PRM_B 20
#define C_STATE_GET_GYR_TEMP_CAL_BASE_V 21
#define C_STATE_GET_GYR_TEMP_CAL_BASE_T 22
#define C_STATE_GET_LOW_PASS 23
#define C_STATE_GET_CAN_MAPPING 24
#define C_STATE_GET_CAN_HEARTBEAT 25
#define C_STATE_GET_LIN_ACC_COMP_MODE 26
#define C_STATE_CENTRI_COMP_MODE 27
#define C_STATE_GET_CAN_CONFIGURATION 28
#define C_RESET_SENSOR_TIMESTAMP 29
#define C_STATE_GET_MAG_ALIGNMENT_MATRIX 30
#define C_STATE_GET_MAG_ALIGNMENT_BIAS 31
#define C_STATE_GET_MAG_REFERENCE 32
#define C_STATE_GET_UART_BAUDRATE 33

#define CAL_STATE_GET_STATUS 1
#define CAL_STATE_WAIT_FINISH 2
#define POLL_PERIOD 2
#define N_GYR_TEMPCAL_SETS 3
#define WAIT_CONNECT_ERROR 1000000
#define WAIT_FIRMWARE_WRITE_TIME 15000000
#define N_ALIGNMENT_SETS 6
#define N_MAG_ALIGNMENT_SETS 12
#define LPMS_MAG_CALIBRATION_DURATION_20S 20000
#define CALC_GYR_MA_DURATION 5000.0f
#define LPMS_REF_CALIBRATION_DURATION_1S 1000.0f
#define LPMS_REF_CALIBRATION_DURATION_20S 20000.0f
#define WAIT_AFTER_CONNECT 500000
#define STATUS_PERIOD 500000
#define WAIT_IAP_WRITE_TIME 3000000
#define STREAM_N_PREPARE 100
#define FIRMWARE_BACKUP_FILE "LpmsFirmwareBackupFile.txt"

class LpmsSensor : public LpmsSensorI
{
public:
/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
	LpmsSensor(int deviceType, const char *deviceId);	
	~LpmsSensor(void);

/***********************************************************************
** POLL / UPDATE DATA FROM SENSORS
***********************************************************************/
	void pollData(void);
	void assertConnected(void);
	void update(void);

/***********************************************************************
** DIRECT GET / SET DEVICE PARAMETERS
***********************************************************************/
	void getDeviceId(char *str);
	CalibrationData *getConfigurationData(void);
	bool assertFwVersion(int d0, int d1, int d2);
	bool hasNewFieldMap(void);
	void setFps(float f);
	float getFps(void);
	void setSensorStatus(int s);
	int getSensorStatus(void);
	void setConnectionStatus(int s);
	int getConnectionStatus(void);
	void setCurrentData(ImuData d);
	void setCallback(LpmsCallback cb);
	ImuData getCurrentData(void);
	int hasImuData(void);
	void getCalibratedSensorData(float g[3], float a[3], float b[3]);
	void getQuaternion(float q[4]); 
	void getEulerAngle(float r[3]); 
	void getRotationMatrix(float M[3][3]); 
	bool isRunning(void);
	void pause(void);
	void run(void);
	void close(void);
	void startCalibrateGyro(void);
	void setOrientationOffset(int v);
	void resetOrientationOffset(void);
	void setOpenMatId(int id);
	int getOpenMatId(void);
	bool updateParameters(void);
	bool setConfigurationPrm(int parameterIndex, int parameter);
	bool setConfigurationPrm(int parameterIndex, int *parameter);
	bool getConfigurationPrm(int parameterIndex, int* parameter);
	bool getConfigurationPrm(int parameterIndex, char* parameter);

/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
	bool uploadFirmware(const char *fn);
	bool uploadIap(const char *fn);
	int getUploadProgress(int *p); 
	void saveCalibrationData(void);	
	LpmsIoInterface *getIoInterface(void);
	void measureAvgLatency(void); 
	void acquireFieldMap(void); 
	bool getPressure(float *p);
	void getHardIronOffset(float v[3]); 
	void getSoftIronMatrix(float M[3][3], float *fieldRadius); 
	float getFieldNoise(void);
	void getFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]);
	void zeroFieldMap(void);
	void resetToFactorySettings(void);
	long getStreamFrequency(void);
	void armTimestampReset(void);

/***********************************************************************
** CALIBRATION
***********************************************************************/
	void startPlanarMagCalibration(void);
	void checkPlanarMagCal(float T);
	void stopPlanarMagCalibration(void);
	void startMagCalibration(void);
	void checkMagCal(float T);
	void stopMagCalibration(void);
	void initMisalignCal(void);
	void startGetMisalign(int i); 
	void checkMisalignCal(float T);
	void calcMisalignMatrix(void);
    void stopMisalignCal(void);
	void saveCalibrationData(const char* fn);
	void loadCalibrationData(const char* fn);
	void initGyrMisalignCal(void);
	void startGetGyrMisalign(int i); 
	void checkGyrMisalignCal(float T);
    void calcGyrMisalignMatrix(void);
    void stopGyrMisalignCal(void);
	void resetTimestamp(void);
	void setTimestamp(float t);
	void startAutoMagMisalignCal(void);
	void checkAutoMagMisalignCal(float T);
	void calcAutoMagMisalignCal(void);
	void startMagReferenceCal(void);
	void checkMagReferenceCal(float T);
	void calcMagReferenceCal(void);
	void initMagMisalignCal(void);
	void startMagMisalignCal(int i); 
	void checkMagMisalignCal(float T);
	void calcMagMisalignCal(void);

/***********************************************************************
** DATA RECORDING
***********************************************************************/
	void startSaveData(std::ofstream *saveDataHandle);
	void checkSaveData(void);
	void stopSaveData(void);

private:
	LpmsIoInterface *bt;	
	std::string deviceId;
	int state;
	MicroMeasure lpmsTimer;
	MicroMeasure statusTimer;
	int deviceType;
	CalibrationData configData;
	long configReg;
	int getConfigState;
	std::string iapFilename;
	std::string firmwareFilename;
	unsigned long frameNo;	
	float frameTime;
	float currentFps;
	int sensorStatus;
	int connectionStatus;
	ImuData currentData;
	bool stopped;
	bool paused;	
	std::mutex sensorMutex;
	float q0;
	float q1;
	float q2;
	float q3;
	float accLatency;
	float avgLatency;
	int latencyCounter;
	bool newFieldMap;
	bool isMagCalibrationEnabled;
	float magCalibrationDuration;
	LpVector3f aRaw;
	LpVector3f bRaw;
	LpVector3f gRaw;
	LpVector3f a;
	LpVector3f b;
	LpVector3f g;
	LpVector4f qOffset;
	LpVector4f currentQ;
	LpVector4f qAfterOffset;
	LpVector3f misalignAData[N_ALIGNMENT_SETS];
	LpVector3f misalignBData[N_ALIGNMENT_SETS];
	LpVector3f gyrMisalignAData[N_ALIGNMENT_SETS];
	LpVector3f gyrMisalignBData[N_ALIGNMENT_SETS];
	LpVector3f magMisalignAData[N_MAG_ALIGNMENT_SETS];
	LpVector3f magmisalignBData[N_MAG_ALIGNMENT_SETS];
	bool isGetMisalign;
	bool isGetGyrMisalign;
	int misalignSetIndex;
	bool isSelectQuaternion;
	bool isSelectEuler;
	bool isSelectLinAcc;
	int retrialsCommandMode;
	int retrialsConnect;
	int misalignSamples;
	float misalignTime;
	LpVector3f misalignADataAcc;
	LpVector3f gyrMisalignADataAcc;
	LpVector3f gyrTempCalData[N_GYR_TEMPCAL_SETS];
	float gyrAvgTemp[N_GYR_TEMPCAL_SETS];
	float gTemp;
	bool isGetGyrTempCal;
	int prevDataSelection;
	int prepareStream;
	bool isFirmwareUpdated;
	bool isSaveData;
	LpVector3f bMax;
	LpVector3f bMin;
	std::ofstream *saveDataHandle;
	LpmsCallback lpmsCallback;
	bool callbackSet;
	GaitTracking gm;
	bool isMagMisalignCalEnabled;
	bool isAutoMagMisalignCalEnabled;
	float cumulatedRefData[3];
	int cumulatedRefCounter;
	float refCalibrationDuration;
	bool isRefCalibrationEnabled;
	bool isPlanarMagCalibrationEnabled;
	int saveDataPreroll;
	float timestampOffset;
	int frameCounterOffset;
	int currentOffsetResetMethod;
	LpVector3f magAvg;
	bool runOnce;
	std::queue<ImuData> dataQueue;
}; 
	
#endif
