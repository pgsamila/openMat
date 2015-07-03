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

#ifndef LPEMG_SENSOR
#define LPEMG_SENSOR

#include <iostream>
#include <fstream>
#include <mutex>
#include <iomanip>
#include <queue>

#ifdef _WIN32
	#include "windows.h"
#endif

#include "EmgData.h"
#include "MicroMeasure.h"
#include "CalibrationData.h"
#include "LpmsIoInterface.h"
#include "LpemgSensorI.h"
#include "LpMatrix.h"

#ifdef _WIN32
	#include "LpemgBluetooth.h"	
#endif

#ifdef __GNUC__
	#include "LpemgBluetooth.h"	
#endif

#define STATE_CONNECT 1
#define STATE_GET_SETTINGS 2
#define STATE_WAIT_CONNECT 3
#define STATE_MEASURE 4
#define STATE_NONE 5
#define STATE_WAIT_AFTER_CONNECT 8
#define PREPARE_PARAMETER_ADJUSTMENT 27
#define STATE_SET_CONFIG 53
#define STATE_SET_OPENMAT_ID 24
#define STATE_SET_SAMPLING_RATE 30
#define STATE_WRITE_PARAMETERS 26
#define STATE_CHECK_IAP_UPLOAD 18
#define STATE_WAIT_IAP_WRITE 19
#define STATE_UPLOAD_IAP 20
#define STATE_CHECK_FIRMWARE_UPLOAD 21
#define STATE_WAIT_FIRMWARE_WRITE 22
#define STATE_UPLOAD_FIRMWARE 23
#define STATE_SELECT_DATA 46
#define STATE_SET_SELF_TEST 32
#define STATE_GET_LATENCY 33
#define STATE_RESET_TO_FACTORY_DEFAULTS 39
#define STATE_RESET_TIMESTAMP 61
#define STATE_ARM_TIMESTAMP_RESET 74

#define C_STATE_GET_CONFIG 1
#define C_STATE_GET_FIRMWARE_VERSION 16
#define C_STATE_GOTO_COMMAND_MODE 9
#define C_STATE_SETTINGS_DONE 5

#define CAL_STATE_GET_STATUS 1
#define CAL_STATE_WAIT_FINISH 2
#define POLL_PERIOD 2

#define WAIT_CONNECT_ERROR 1000000
#define WAIT_FIRMWARE_WRITE_TIME 15000000

#define WAIT_AFTER_CONNECT 500000
#define STATUS_PERIOD 500000

#define WAIT_IAP_WRITE_TIME 3000000

#define STREAM_N_PREPARE 100

#define LPEMG_FIRMWARE_BACKUP_FILE "LpemgFirmwareBackupFile.txt"

class LpemgSensor : public LpemgSensorI
{
public:
/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
	LpemgSensor(int deviceType, const char *deviceId);	
	~LpemgSensor(void);

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
	void setSensorStatus(int s);
	int getSensorStatus(void);
	void setConnectionStatus(int s);
	int getConnectionStatus(void);
	void setCurrentData(EmgData d);
	void setCallback(LpemgCallback cb);
	void getSensorData(SensorData *d);
	int hasData(void);
	bool isRunning(void);
	void pause(void);
	void run(void);
	void close(void);
	void setOpenMatId(int id);
	int getOpenMatId(void);
	bool updateParameters(void);
	bool setConfigurationPrm(int parameterIndex, int parameter);
	bool setConfigurationPrm(int parameterIndex, int *parameter);
	bool getConfigurationPrm(int parameterIndex, int* parameter);
	bool getConfigurationPrm(int parameterIndex, char* parameter);
	LpmsIoInterface *getIoInterface(void);
	void measureAvgLatency(void); 
	void resetToFactorySettings(void);
	long getStreamFrequency(void);
	void armTimestampReset(void);
	void setFps(float f);
	float getFps(void);	

/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
	bool uploadFirmware(const char *fn);
	bool uploadIap(const char *fn);
	int getUploadProgress(int *p); 

/***********************************************************************
** CALIBRATION
***********************************************************************/
	void saveCalibrationData(void);
	void loadCalibrationData(const char* fn);
	void saveCalibrationData(const char* fn);
	void resetTimestamp(void);
	void setTimestamp(float t);	

/***********************************************************************
** DATA RECORDING
***********************************************************************/
	void startSaveData(std::ofstream *saveDataHandle);
	void checkSaveData(void);
	void stopSaveData(void);

private:
	LpemgIoInterface *bt;	
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
	EmgData currentData;
	bool stopped;
	bool paused;	
	std::mutex sensorMutex;
	float avgLatency;
	int latencyCounter;
	int retrialsCommandMode;
	int retrialsConnect;
	int prevDataSelection;
	int prepareStream;
	bool isFirmwareUpdated;
	bool isSaveData;
	std::ofstream *saveDataHandle;
	LpemgCallback lpemgCallback;
	bool callbackSet;
	int saveDataPreroll;
	float timestampOffset;
	int frameCounterOffset;
	int currentOffsetResetMethod;
	bool runOnce;
	std::queue<EmgData> emgDataQueue;
	float accLatency;
}; 
	
#endif
