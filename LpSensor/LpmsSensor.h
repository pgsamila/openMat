/***********************************************************************
** Copyright (C) 2012 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_SENSOR
#define LPMS_SENSOR

#include <boost/thread/thread.hpp>

#ifdef _WIN32
	#include "windows.h"
#endif

#include "ImuData.h"
#include "MicroMeasure.h"
#include "CalibrationData.h"
#include "LpmsIoInterface.h"
#include "LpmsSensorI.h"
#include "LpMatrix.h"
#include "LpFilterCVersion.h"
#include "LpMagnetometerCalibration.h"
#include "CalcMisalignment.h"
#include "GaitTracking.h"

#ifdef _WIN32
	#include "LpmsCanIo.h"
	#include "LpmsU.h"
	#include "LpmsBBluetooth.h"	
	#include "LpmsRS232.h"
	
	#ifdef USE_ZEROC_ICE
		#include "IceImuDriver.h"
	#endif
#endif

#ifdef __GNUC__
	#include "LpmsCanIo.h"
	#include "LpmsU.h"
	#include "LpmsBBluetooth.h"	
	#include "LpmsRS232.h"
#endif
	
#define LOGV(...) std::cout<<__VA_ARGS__<<std::endl
#define LOGD(...) std::cout<<__VA_ARGS__<<std::endl
#define LOGI(...) std::cout<<__VA_ARGS__<<std::endl
#define LOGW(...) std::cout<<__VA_ARGS__<<std::endl
#define LOGE(...) std::cout<<__VA_ARGS__<<std::endl

#ifdef ANDROID
	using namespace std;
	
	#include <jni.h>
	#include <android/log.h>
	
	#include "AndroidBluetooth.h"
#endif

#define STATE_CONNECT 1
#define STATE_GET_SETTINGS 2
#define STATE_WAIT_CONNECT 3
#define STATE_MEASURE 4
#define STATE_NONE 5
#define STATE_CALIBRATE_GYRO 6
#define STATE_CALIBRATE_MAG 7
#define STATE_WAIT_AFTER_CONNECT 8
#define STATE_RESET_ORIENTATION 9
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

#define CAL_STATE_GET_STATUS 1
#define CAL_STATE_WAIT_FINISH 2

#define POLL_PERIOD 2

#define N_GYR_TEMPCAL_SETS 3

#define WAIT_CONNECT_ERROR 1000000

#define WAIT_FIRMWARE_WRITE_TIME 15000000

/* See LpmsSensorI for comments on this class. */
class LpmsSensor : public LpmsSensorI
{
public:
#ifndef ANDROID			
	LpmsSensor(int deviceType, const char *deviceId);
#else
	LpmsSensor(int deviceType, const char *deviceId, JavaVM *thisVm, jobject bluetoothAdapter);
#endif	

	~LpmsSensor(void);
	void run(void);
	void pause(void);
	void close(void);
	void update(void);
	bool isRunning(void);
	ImuData getCurrentData(void);
	void getDeviceId(char *str);
	void setOpenMatId(int id);
	int getOpenMatId(void);
	int getSensorStatus(void);
	int getConnectionStatus(void);
	float getFps(void);
	void startResetReference(void);
	void startCalibrateGyro(void);
	void startCalibrateMag(void);
	void stopCalibrateMag(void);
	void resetTimestamp(void);
	CalibrationData* getConfigurationData(void);
	bool setConfigurationPrm(int parameterIndex, int parameter);
	bool setConfigurationPrm(int parameterIndex, int *parameter);	
	bool getConfigurationPrm(int parameterIndex, int *parameter);	
	bool getConfigurationPrm(int parameterIndex, char *parameter);		
	void pollData(void);
	bool uploadFirmware(const char *fn);
	bool uploadIap(const char *fn);
	void resetOrientation(void);
	void saveCalibrationData(void);
	LpmsIoInterface *getIoInterface(void);
	void getCalibratedSensorData(float g[3], float a[3], float b[3]);
	void getQuaternion(float q[4]);
	void getEulerAngle(float r[3]);
	void getRotationMatrix(float M[3][3]);
	bool getUploadProgress(int *p);
	void measureAvgLatency(void); 
	void acquireFieldMap(void);
	bool getPressure(float *p);
	void getFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]);
	void getHardIronOffset(float v[3]);
	void getSoftIronMatrix(float M[3][3], float *fieldRadius);
	bool hasNewFieldMap(void);
	void resetToFactorySettings(void);
	float getFieldNoise(void);
	void saveCalibrationData(const char *fn);
	void loadCalibrationData(const char *fn);
	void startSaveData(FILE *saveDataHandle);
	void checkSaveData(void);
	void stopSaveData(void);
	void setCallback(LpmsCallback cb);
	
private:
	void checkResetReference(void);
	void checkMagCalibration(ImuData d);
	void checkGyroCalibration(ImuData d);
	void setSensorStatus(int s);
	void setConnectionStatus(int s);
	void setCurrentData(ImuData d);
	void setFps(float f);
	void backupGyroThresholdSetting(void);
	void restoreGyroThresholdSetting(void);
	long getStreamFrequency(void);
	void zeroFieldMap(void);
	void startMagCalibration(void);
	void checkMagCal(float T);
	void stopMagCalibration(void);
	void initMisalignCal(void);
	void startGetMisalign(int i);
	void checkMisalignCal(float T);
	void calcMisalignMatrix(void);
	void initGyrMisalignCal(void);
	void startGetGyrMisalign(int i);
	void checkGyrMisalignCal(float T);
	void calcGyrMisalignMatrix(void);
	void initGyrTempCal(void);
	void startGetGyrTempCal(int i);
	void checkGyrTempCal(float T);
	void calcGyrTempCal(void);
	bool updateParameters(void);
	void startPlanarMagCalibration(void);
	void checkPlanarMagCal(float T);
	void stopPlanarMagCalibration(void);

	LpmsIoInterface *bt;	
	string deviceId;
	int state;
	MicroMeasure lpmsTimer;
	MicroMeasure statusTimer;
	int deviceType;
	CalibrationData configData;
	long configReg;
	int getConfigState;
	string iapFilename;
	string firmwareFilename;
	unsigned long frameNo;	
	float frameTime;
	float currentFps;
	int sensorStatus;
	int connectionStatus;
	ImuData currentData;
	bool stopped;
	bool paused;	
	boost::mutex sensorMutex;
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
	LpVector3f misalignAData[6];
	LpVector3f misalignBData[6];
	LpVector3f gyrMisalignAData[6];
	LpVector3f gyrMisalignBData[6];
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
	FILE* saveDataHandle;
	LpmsCallback lpmsCallback;
	bool callbackSet;
	GaitTracking gm;
}; 
	
#endif	
