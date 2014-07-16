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
	/* Destructor. */
	virtual ~LpmsSensorI(void) { };

	/* Starts the sensor. */
	virtual void run(void) = 0;

	/* Pauses the sensor. */
	virtual void pause(void) = 0;

	/* Closes the connection to the sensor. */
	virtual void close(void) = 0;

	/* Returns if the sensor is currently acquring data. */ 
	virtual bool isRunning(void) = 0;

	/* Retrieves currently sampled data as an ImuData structure. */
	virtual ImuData getCurrentData(void) = 0;
	
	/* Gets the ID of the connected device. */
	virtual void getDeviceId(char *str) = 0;

	/* Sets the OpenMAT ID of the device. */
	virtual void setOpenMatId(int id) = 0;

	/* Gets the OpenMAT ID of the device. */
	virtual int getOpenMatId(void) = 0;

	/* Gets current sensor status. States are defined in LpmsDefinitions.h. */
	virtual int getSensorStatus(void) = 0;

	/* Gets current connection status. States are defined in LpmsDefinitions.h. */
	virtual int getConnectionStatus(void) = 0;

	/* Retrieves the current framerate. */
	virtual float getFps(void) = 0;

	/* Starts the calibration of the gyroscope. */
	virtual void startCalibrateGyro(void) = 0;

	/* Resets the current sensor timestamp. */ 
	virtual void resetTimestamp(void) = 0;
	
	/* Sets a single integer configuration parameter. */
	virtual bool setConfigurationPrm(int parameterIndex, int parameter) = 0;

	/* Sets an integer array configuration parameter. */
	virtual bool setConfigurationPrm(int parameterIndex, int *parameter) = 0;	
	
	/* Retrieves a configuration parameter. */
	virtual bool getConfigurationPrm(int parameterIndex, int *parameter) = 0;	

	/* Retrieves a configuration parameter. */
	virtual bool getConfigurationPrm(int parameterIndex, char *parameter) = 0;	
	
	/* Starts uploading the firmware. */
	virtual bool uploadFirmware(const char *fn) = 0;

	/* Starts uploading the in-application programmer. */
	virtual bool uploadIap(const char *fn) = 0;

	/* Starts saving the current parameter settings to the sensor flash memory. */
	virtual void saveCalibrationData(void) = 0;

	/* Retrieves calibrated sensor data (gyroscope, accelerometer, magnetometer). */
	virtual void getCalibratedSensorData(float g[3], float a[3], float b[3]) = 0;

	/* Retrieves the 3d orientation quaternion. */
	virtual void getQuaternion(float q[4]) = 0;

	/* Retrieves the currently measured 3d Euler angles. */
	virtual void getEulerAngle(float r[3]) = 0;

	/* Retrievs the current rotation matrix. */
	virtual void getRotationMatrix(float M[3][3]) = 0;
	
	/* Gets the current upload progress. */
	virtual int getUploadProgress(int *p) = 0;	
	
	/* Measures the average data transfer latency. */
	virtual void measureAvgLatency(void) = 0; 
	
	/* Acquires the current field map. */
	virtual void acquireFieldMap(void) = 0;
	
	/* Retrieves the current pressure measurement. */
	virtual bool getPressure(float *p) = 0;

	/* Retrieves the magnetic field map. */
	virtual void getFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]) = 0;
	
	/* Gets the current hard iron offset. */
	virtual void getHardIronOffset(float v[3]) = 0;
	
	/* Gets the current soft iron matrix. */
	virtual void getSoftIronMatrix(float M[3][3], float *fieldRadius) = 0;
	
	/* Checks, if a new field map has been acquired. */
	virtual bool hasNewFieldMap(void) = 0;
	
	/* Resets sensor to factory settings. */
	virtual void resetToFactorySettings(void) = 0;
	
	/* Initializes accelerometer misalignment calibration. */
	virtual void initMisalignCal(void) = 0;

	/* Starts accalerometer misalignment calibration. */
	virtual void startGetMisalign(int i) = 0;

	/* Calculates accelerometer misalignment matrix. */
	virtual void calcMisalignMatrix(void) = 0;
	
	/* Initialize gyroscope misalignment calibration. */
	virtual void initGyrMisalignCal(void) = 0;

	/* Starts acquiring gyroscope misalignment data. */
	virtual void startGetGyrMisalign(int i) = 0;

	/* Calculates gyroscope misalignment matrix. */
	virtual void calcGyrMisalignMatrix(void) = 0;
	
	/* Gets the current magnetic field noise indicator. */
	virtual float getFieldNoise(void) = 0;
	
	/* Saves the current calibration data. */
	virtual void saveCalibrationData(const char *fn) = 0;

	/* Loads current calibration data. */
	virtual void loadCalibrationData(const char *fn) = 0;
	
	/* Starts saving data to file handle. */
	virtual void startSaveData(std::ofstream *saveDataHandle) = 0;

	/* Updates file saving. */
	virtual void checkSaveData(void) = 0;

	/* Stops file saving. */
	virtual void stopSaveData(void) = 0;
	
	/* Sets callback for data data acquisition. */
	virtual void setCallback(LpmsCallback cb) = 0;

	virtual void initMagMisalignCal(void) = 0;
	
	virtual void startMagMisalignCal(int i) = 0;
	
	virtual void startAutoMagMisalignCal(void) = 0;	
	
	virtual void startPlanarMagCalibration(void) = 0;
	
	virtual void startMagReferenceCal(void) = 0;
	
	virtual void setOrientationOffset(int v) = 0;
	
	virtual void resetOrientationOffset(void) = 0;
	
	virtual void startMagCalibration(void) = 0;	
	
	virtual void setTimestamp(float t) = 0;
	virtual void armTimestampReset(void) = 0;
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
