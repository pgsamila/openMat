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

#ifndef LPMS_SENSOR_C
#define LPMS_SENSOR_C

#include <math.h>

#include "../LpSensor/ImuData.h"
#include "../LpSensor/LpMatrix.h"
#include "../LpSensor/LpMagnetometerCalibration.h"
#include "../LpSensor/CalcMisalignment.h"
#include "../LpSensor/LpMagnetometerMAlignment.h"

#include "MicroMeasureC.h"
#include "CalibrationDataC.h"
#include "LpmsIoInterfaceC.h"

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

#define CAL_STATE_GET_STATUS 1
#define CAL_STATE_WAIT_FINISH 2

#define POLL_PERIOD 2

#define N_GYR_TEMPCAL_SETS 3

#define WAIT_CONNECT_ERROR 1000000

#define WAIT_FIRMWARE_WRITE_TIME 15000000

#define N_ALIGNMENT_SETS 6
#define N_MAG_ALIGNMENT_SETS 12

typedef void (*LpmsCallback)(ImuData d, const char* id);

void initLpmsSensor(int deviceType, const char *deviceId);
void lpmsSensorRun(void);
void lpmsSensorPause(void);
void lpmsSensorClose(void);
void lpmsSensorUpdate(void);
int lpmsSensorIsRunning(void);
int lpmsSensorGetCurrentData(ImuData *d);
void lpmsSensorGetDeviceId(char *str);
void lpmsSensorSetOpenMatId(int id);
int lpmsSensorGetOpenMatId(void);
int lpmsSensorGetSensorStatus(void);
int lpmsSensorGetConnectionStatus(void);
float lpmsSensorGetFps(void);
void lpmsSensorStartCalibrateGyro(void);
void lpmsSensorResetTimestamp(void);
CalibrationData* lpmsSensorGetConfigurationData(void);
int lpmsSensorSetConfigurationPrmInt(int parameterIndex, int parameter);
int lpmsSensorSetConfigurationPrmIntArray(int parameterIndex, int *parameter);	
int lpmsSensorGetConfigurationPrmInt(int parameterIndex, int *parameter);	
int lpmsSensorGetConfigurationPrmString(int parameterIndex, char *parameter);		
void lpmsSensorPollData(void);
int lpmsSensorUploadFirmware(const char *fn);
int lpmsSensorUploadIap(const char *fn);
void lpmsSensorSaveParametersToSensor(void);
void lpmsSensorGetCalibratedSensorData(float g[3], float a[3], float b[3]);
void lpmsSensorGetQuaternion(float q[4]);
void lpmsSensorGetEulerAngle(float r[3]);
void lpmsSensorGetRotationMatrix(float M[3][3]);
int lpmsSensorGetUploadProgress(int *p);
void lpmsSensorMeasureAvgLatency(void); 
void lpmsSensorAcquireFieldMap(void);
int lpmsSensorGetPressure(float *p);
void lpmsSensorGetFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]);
void lpmsSensorGetHardIronOffset(float v[3]);
void lpmsSensorGetSoftIronMatrix(float M[3][3], float *fieldRadius);
int lpmsSensorHasNewFieldMap(void);
void lpmsSensorResetToFactorySettings(void);
float lpmsSensorGetFieldNoise(void);
void lpmsSensorsaveCalibrationData(const char *fn);
void lpmsSensorLoadCalibrationData(const char *fn);
void lpmsSensorStartSaveData(void /* std::ofstream *saveDataHandle */);
void lpmsSensorCheckSaveData(void);
void lpmsSensorStopSaveData(void);
void lpmsSensorSetCallback(LpmsCallback cb);
int lpmsSensorAssertFwVersion(int d0, int d1, int d2);
void lpmsSensorSetOrientationOffset(void);
void lpmsSensorResetOrientationOffset(void);
void lpmsSensorStartMagCalibration(void);
void lpmsSensorSyncTimestamp(float t);
void lpmsSensorSetCurrentSyncOffset(float t);
float lpmsSensorGetCurrentSyncOffset(void);
void lpmsSensorCheckResetReference(void);
void lpmsSensorCheckMagCalibration(ImuData d);
void lpmsSensorCheckGyroCalibration(ImuData d);
void lpmsSensorSetSensorStatus(int s);
void lpmsSensorSetConnectionStatus(int s);
void lpmsSensorSetCurrentData(ImuData d);
void lpmsSensorSetFps(float f);
void lpmsSensorBackupGyroThresholdSetting(void);
void lpmsSensorRestoreGyroThresholdSetting(void);
long lpmsSensorGetStreamFrequency(void);
void lpmsSensorZeroFieldMap(void);
void lpmsSensorCheckMagCal(float T);
void lpmsSensorStopMagCalibration(void);
void lpmsSensorInitMisalignCal(void);
void lpmsSensorStartGetMisalign(int i);
void lpmsSensorCheckMisalignCal(float T);
void lpmsSensorCalcMisalignMatrix(void);
void lpmsSensorInitGyrMisalignCal(void);
void lpmsSensorStartGetGyrMisalign(int i);
void lpmsSensorCheckGyrMisalignCal(float T);
void lpmsSensorCalcGyrMisalignMatrix(void);
void lpmsSensorInitGyrTempCal(void);
void lpmsSensorStartGetGyrTempCal(int i);
void lpmsSensorCheckGyrTempCal(float T);
void lpmsSensorCalcGyrTempCal(void);
int lpmsSensorUpdateParameters(void);
void lpmsSensorStartPlanarMagCalibration(void);
void lpmsSensorCheckPlanarMagCal(float T);
void lpmsSensorStopPlanarMagCalibration(void);
void lpmsSensorInitMagMisalignCal(void);
void lpmsSensorStartMagMisalignCal(int i);
void lpmsSensorCheckMagMisalignCal(float T);
void lpmsSensorCalcMagMisalignCal(void);
void lpmsSensorStartMagReferenceCal(void);
void lpmsSensorCheckMagReferenceCal(float T);
void lpmsSensorCalcMagReferenceCal(void);
void lpmsSensorStartAutoMagMisalignCal(void);
void lpmsSensorCheckAutoMagMisalignCal(float T);
void lpmsSensorCalcAutoMagMisalignCal(void);
void lpmsSensorAssertConnected(void);
	
#endif
