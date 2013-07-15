/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
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

#include <boost/thread/thread.hpp> 
#include <boost/foreach.hpp>

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
	string configurationFile;
	LpmsDeviceList deviceList;
	int managerState;
	std::ofstream saveDataHandle;
	bool isRecording;
	boost::mutex lm;
	int threadDelay;
	char writeBuffer[65536];
	
#ifdef _WIN32	
	CanEngine ce;
#endif

#ifdef ANDROID
	JavaVM *thisVm;
	jobject bluetoothAdapter;
#endif
};

#endif
