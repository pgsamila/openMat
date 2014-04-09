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

#ifndef LPMS_SENSOR_MANAGER
#define LPMS_SENSOR_MANAGER

#include "LpmsSensorManagerI.h"
#include "LpmsSensor.h"
#include "CalibrationData.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"

#ifdef _WIN32
	#include "LpmsBBluetooth.h"
	#include "LpmsU.h"
	#include "CanEngine.h"
	#include "LpmsRS232.h"
	#include "BleEngine.h"
#endif

#ifdef __GNUC__
	#include "LpmsBBluetooth.h"
	#include "LpmsU.h"
	#include "CanEngine.h"
	#include "LpmsU.h"
#endif

#ifdef ANDROID
	using namespace std;
	
	#include <jni.h>
	#include <android/log.h>
	
	#include "AndroidBluetooth.h"	
#endif

// #include <boost/thread/thread.hpp> 
// #include <boost/foreach.hpp>

#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>

#ifdef _WIN32
	#ifdef DLL_EXPORT
		#define DLL_MACRO __declspec(dllexport)
	#else
		#define DLL_MACRO __declspec(dllimport)
	#endif
#else
	#define DLL_MACRO
#endif

#pragma warning( disable: 4251 )

#include "pugixml.hpp"

/* See LpmsSensorManagerI for comments on this class. */
class LpmsSensorManager : public LpmsSensorManagerI
{
public:
#ifndef ANDROID		
	LpmsSensorManager(void);
#else
	LpmsSensorManager(JavaVM *thisVm, jobject bluetoothAdapter);
#endif

	~LpmsSensorManager(void);
	void start(void);
	void run(void);		
	LpmsSensorI* addSensor(int mode, const char *deviceId);
	void removeSensor(LpmsSensorI *sensor);	
	void startListDevices(void);
	bool listDevicesBusy(void);
	void stopListDevices(void);
	LpmsDeviceList getDeviceList(void);
	bool saveSensorData(const char* fn);
	void stopSaveSensorData(void);
	bool isRecordingActive(void);
	void setThreadTiming(int delay);
	bool isCanPresent(void);
	void setCanBaudrate(int i);

private:	
	list<LpmsSensor*> sensorList;	
	bool stopped;
	std::string configurationFile;
	LpmsDeviceList deviceList;
	int managerState;
	std::ofstream saveDataHandle;
	bool isRecording;
	std::mutex lm;
	int threadDelay;
	char writeBuffer[65536];
	
#ifdef _WIN32	
	CanEngine ce;
	BleEngine be;
#endif

#ifdef ANDROID
	JavaVM *thisVm;
	jobject bluetoothAdapter;
#endif
};

#endif
