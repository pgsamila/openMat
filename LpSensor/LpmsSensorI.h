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

#ifndef LPMS_SENSOR_I
#define LPMS_SENSOR_I

#include <stdio.h>
#include <iostream>
#include <fstream>

#include "ImuData.h"
#include "LpmsDefinitions.h"

typedef void (*LpmsCallback)(ImuData d, const char* id);  

/* Class for accessing an LPMS device. */
class LpmsSensorI
{
public:
/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
	virtual ~LpmsSensorI(void) { };
	
/***********************************************************************
** POLL / UPDATE DATA FROM SENSOR
***********************************************************************/	
	virtual void pollData(void) = 0;
	virtual void update(void) = 0;
	
/***********************************************************************
** SENSOR CONTROL
***********************************************************************/
	virtual void pause(void) = 0;
	virtual void run(void) = 0;
	virtual bool isRunning(void) = 0;
	virtual void assertConnected(void) = 0;
	virtual void close(void) = 0;
	virtual void measureAvgLatency(void) = 0;
	virtual void acquireFieldMap(void) = 0;
	virtual void resetToFactorySettings(void) = 0;
	virtual void setConnectionStatus(int s) = 0;
	virtual int getConnectionStatus(void) = 0;
	virtual void setCallback(LpmsCallback cb) = 0;
	virtual bool startSelfTest(void) = 0;

/***********************************************************************
** SENSOR DATA, CONFIGURATION AND COMMANDS
***********************************************************************/
	virtual void setCurrentData(ImuData d) = 0;
	virtual ImuData getCurrentData(void) = 0;
	virtual void setSensorStatus(int s) = 0;
	virtual int getSensorStatus(void) = 0;
	virtual void setOpenMatId(int id) = 0;
	virtual int getOpenMatId(void) = 0;
	virtual void getDeviceId(char *str) = 0;
	virtual bool assertFwVersion(int d0, int d1, int d2) = 0;
	// virtual CalibrationData *getConfigurationData(void) = 0;
	virtual bool updateParameters(void) = 0;
	virtual bool setSensorCommand(int commandIndex, int parameter) = 0;
	
	/* virtual bool setConfigurationPrm(int parameterIndex, int parameter) = 0;
	virtual bool setConfigurationPrm(int parameterIndex, int *parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex, int* parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex, char* parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex0, int parameterIndex1, int parameterIndex2, int parameterIndex3, float* parameter) = 0; */
	
	virtual bool setConfigurationPrm(int parameterIndex, void* parameter) = 0;
	virtual bool getConfigurationPrm(int parameterIndex, void* parameter) = 0;
	
	// virtual LpmsIoInterface *getIoInterface(void) = 0;
	virtual void saveConfigurationToSensor(void) = 0;
	virtual void saveConfiguration(const char* fn) = 0;	
	virtual void loadConfiguration(const char* fn) = 0;	
	
/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
	virtual bool uploadFirmware(const char *fn) = 0;
	virtual bool uploadIap(const char *fn) = 0;
	virtual int getUploadProgress(int *p) = 0;
	
/***********************************************************************
** CALIBRATION
***********************************************************************/
	virtual void setOrientationOffset(void) = 0;
	virtual void resetOrientationOffset(void) = 0;
	virtual void startCalibrateGyro(void) = 0;
	virtual void startPlanarMagCalibration() = 0;
	virtual void checkPlanarMagCal(float T) = 0;
	virtual void stopPlanarMagCalibration(void) = 0;
	virtual void startMagCalibration(void) = 0;
	virtual void checkMagCal(float T) = 0;
	virtual void stopMagCalibration(void) = 0;
	virtual void initMisalignCal(void) = 0;
	virtual void startGetMisalign(int i) = 0;
	virtual void checkMisalignCal(float T) = 0;
	virtual void calcMisalignMatrix(void) = 0;
	virtual void initGyrMisalignCal(void) = 0;
	virtual void startGetGyrMisalign(int i) = 0; 
	virtual void checkGyrMisalignCal(float T) = 0;
	virtual void calcGyrMisalignMatrix(void) = 0;
	virtual void resetTimestamp(void) = 0;
	virtual void syncTimestamp(float t) = 0;
	virtual void startAutoMagMisalignCal(void) = 0;
	virtual void checkAutoMagMisalignCal(float T) = 0;
	virtual void calcAutoMagMisalignCal(void) = 0;
	virtual void startMagReferenceCal(void) = 0;
	virtual void checkMagReferenceCal(float T) = 0;
	virtual void calcMagReferenceCal(void) = 0;
	virtual void initMagMisalignCal(void) = 0;
	virtual void startMagMisalignCal(int i) = 0;
	virtual void checkMagMisalignCal(float T) = 0;
	virtual void calcMagMisalignCal(void) = 0;

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