/***********************************************************************
** Copyright (C) 2012 LP-Research
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

#ifndef LPEMG_SENSOR_I
#define LPEMG_SENSOR_I

#include <stdio.h>
#include <iostream>
#include <fstream>

#include "LpSensorBaseI.h"
#include "EmgData.h"
#include "LpmsDefinitions.h"

typedef void (*LpemgCallback)(EmgData d, const char* id);

class LpemgSensorI : LpSensorBaseI
{
public:
/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
	~LpemgSensorI(void) { };

/***********************************************************************
** POLL / UPDATE DATA FROM SENSORS
***********************************************************************/
	virtual void pollData(void) = 0;
	virtual void assertConnected(void) = 0;
	virtual void update(void) = 0;

/***********************************************************************
** DIRECT GET / SET DEVICE PARAMETERS
***********************************************************************/
	virtual void getDeviceId(char *str) = 0;
	// virtual CalibrationData *getConfigurationData(void) = 0;
	virtual bool assertFwVersion(int d0, int d1, int d2) = 0;
	virtual void setSensorStatus(int s) = 0;
	virtual int getSensorStatus(void) = 0;
	virtual void setConnectionStatus(int s) = 0;
	virtual int getConnectionStatus(void) = 0;
	virtual void setCurrentData(EmgData d) = 0;
	virtual void setCallback(LpemgCallback cb) = 0;
	virtual void getSensorData(SensorData *d) = 0;	
	virtual int hasData(void) = 0;
	virtual bool isRunning(void) = 0;
	virtual void pause(void) = 0;
	virtual void run(void) = 0;
	virtual void close(void) = 0;
	virtual void setOpenMatId(int id) = 0;
	virtual int getOpenMatId(void) = 0;
	virtual bool updateParameters(void) = 0;
	virtual bool setConfigurationPrm(int parameterIndex, int parameter) = 0;
	virtual bool setConfigurationPrm(int parameterIndex, int *parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex, int* parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex, char* parameter) = 0;
	virtual void measureAvgLatency(void) = 0;
	virtual void resetToFactorySettings(void) = 0;
	virtual long getStreamFrequency(void) = 0;
	virtual void armTimestampReset(void) = 0;

/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
	virtual bool uploadFirmware(const char *fn) = 0;
	virtual bool uploadIap(const char *fn) = 0;
	virtual int getUploadProgress(int *p) = 0;

/***********************************************************************
** CALIBRATION
***********************************************************************/
	virtual void saveCalibrationData(void) = 0;
	virtual void loadCalibrationData(const char* fn) = 0;
	virtual void saveCalibrationData(const char* fn) = 0;
	virtual void resetTimestamp(void) = 0;
	virtual void setTimestamp(float t) = 0;

/***********************************************************************
** DATA RECORDING
***********************************************************************/
	virtual void startSaveData(std::ofstream *saveDataHandle) = 0;
	virtual void checkSaveData(void) = 0;
	virtual void stopSaveData(void) = 0;
};
	
#ifdef _WIN32
	#ifdef DLL_EXPORT
		#define LPMS_API __declspec(dllexport)
	#else
		#define LPMS_API __declspec(dllimport)
	#endif
#else
	#define LPMS_API
#endif
	
#endif
