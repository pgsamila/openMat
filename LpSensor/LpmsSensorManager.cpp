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

#include "LpmsSensorManager.h"

#define SENSOR_UPDATE_PERIOD 200

#define SMANAGER_LIST 0
#define SMANAGER_MEASURE 1

#ifdef _WIN32
	LpmsSensorManagerI* APIENTRY LpmsSensorManagerFactory(void) 
	{
		return (LpmsSensorManagerI*) new LpmsSensorManager();
	}

	LpmsSensorManager::LpmsSensorManager(void)
#endif

#ifdef __GNUC__
	LpmsSensorManagerI* LpmsSensorManagerFactory(void) 
	{
		return (LpmsSensorManagerI*) new LpmsSensorManager();
	}

	LpmsSensorManager::LpmsSensorManager(void)
#endif

#ifdef ANDROID
	LpmsSensorManagerI* APIENTRY LpmsSensorManagerFactory(JavaVM *thisVm, jobject bluetoothAdapter) 
	{
		return (LpmsSensorManagerI*) new LpmsSensorManager(JavaVM *thisVm, jobject bluetoothAdapter);
	}

	LpmsSensorManager::LpmsSensorManager(JavaVM *thisVm, jobject bluetoothAdapter) :
	thisVm(thisVm),
	bluetoothAdapter(bluetoothAdapter)
#endif
{
	stopped = false;
	isRecording = false;
	threadDelay = 750;
	currentUartBaudrate = 2;

	managerState = SMANAGER_MEASURE;

	std::thread t(&LpmsSensorManager::run, this);
	
#ifdef _WIN32	
	#ifdef THREAD_HIGH_PRIORITY
		HANDLE th = t.native_handle();
		SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
	#endif
#endif
	
	t.detach();
#ifdef ANDROID
	LOGV("[LpmsSensorManager] Started");		
#endif
}

LpmsSensorManager::~LpmsSensorManager(void)
{
	stopped = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	deviceList.clear();
}

void LpmsSensorManager::startListDevices(bool scan_serial_ports)
{
	if (listDevicesBusy() == true) return;
	
	scan_serial_ports_ = scan_serial_ports;
	
	deviceList.clear();
	managerState = SMANAGER_LIST;
	
	return;
}

bool LpmsSensorManager::listDevicesBusy(void) {
	if (managerState == SMANAGER_LIST) return true;

	return false;
}

void LpmsSensorManager::stopListDevices(void) {
	if (managerState != SMANAGER_LIST) return;

	LpmsBBluetooth::stopDiscovery();
	
	managerState = SMANAGER_MEASURE;
	std::cout << "[LpmsSensorManager] Cancelling discovery." << std::endl;
	
	return;
}

void LpmsSensorManager::start(void)
{
	stopped = false;
	std::thread t(&LpmsSensorManager::run, this);

#ifdef _WIN32
	#ifdef THREAD_HIGH_PRIORITY
		HANDLE th = t.native_handle();
		SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
	#endif
#endif
	
	t.detach();	
}

#define LPMS_B_LATENCY_ESTIMATE 0.030f
#define LPMS_OTHER_LATENCY_ESTIMATE 0.005f
	
void LpmsSensorManager::run(void)
{
	MicroMeasure mm;

	list<LpmsSensor*>::iterator i;	
    bool bIsWindows7orLater = true;
	float prevTimestamp = 0.0f;
	int deviceType = 0;

#ifdef _WIN32	
	ce.connect();
	be.connect();
#endif	

#ifdef ANDROID
	LOGV("[LpmsSensorManager] Thread running\n");
#endif

	mm.reset();
	int sleepFlag = 0;
	while (stopped == false) {	
		switch (managerState) {
		case SMANAGER_MEASURE:
			lm.lock();
			for (i = sensorList.begin(); i != sensorList.end(); i++) {
				(*i)->pollData();
			}	
			
#ifdef _WIN32
			ce.poll();
#endif
			
			lm.unlock();

			if (mm.measure() > SENSOR_UPDATE_PERIOD) {			
				mm.reset();				
				
				lm.lock();
				for (i = sensorList.begin(); i != sensorList.end(); i++) {
					(*i)->update();				
				}
				lm.unlock();
			}
#ifdef _WIN32
			/*
			else {
				// Busy Wait hack
				if (sleepFlag++ > 100)
				{
					std::this_thread::sleep_for(std::chrono::nanoseconds(10));
					sleepFlag = 0;
				}
			}
			*/
#endif
		break;

		case SMANAGER_LIST:
			deviceList.clear();

#ifdef _WIN32
			ce.listDevices(&deviceList);
			be.listDevices(&deviceList);			
#endif

			if (scan_serial_ports_ == true) {
				printf("[LpmsSensorManager] List RS232 devices\n");
				LpmsRS232::listDevices(&deviceList);
			}
			LpmsU::listDevices(&deviceList);
			LpmsBBluetooth::listDevices(&deviceList);			

			managerState = SMANAGER_MEASURE;
		break;
		}
#ifdef __GNUC__
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
	}

#ifdef _WIN32		
	ce.close();
	be.close();
#endif
}

bool compareOpenMatId(LpmsSensor *first, LpmsSensor *second)
{
	if (first->getOpenMatId() < second->getOpenMatId()) return true;
	
	return false;
}

LpmsSensorI* LpmsSensorManager::addSensor(int mode, const char *deviceId)
{
	LpmsSensor* sensor;
		
	lm.lock();
	switch (mode) {
	case DEVICE_LPMS_B:	
	
#ifndef ANDROID
		sensor = new LpmsSensor(DEVICE_LPMS_B, deviceId);
#else
		sensor = new LpmsSensor(DEVICE_LPMS_B, deviceId, thisVm, bluetoothAdapter);
		
		LOGV("[LpmsSensorManager] Sensor added\n");
#endif

		sensorList.push_back(sensor);
	break;
		
	case DEVICE_LPMS_BLE:	
		sensor = new LpmsSensor(DEVICE_LPMS_BLE, deviceId);
		sensorList.push_back(sensor);
		
#ifdef _WIN32	
		((LpmsBle *)sensor->getIoInterface())->deviceId = deviceId;
		be.addSensor((LpmsBle *)sensor->getIoInterface());
#endif
	break;		
		
	case DEVICE_LPMS_C:		
		sensor = new LpmsSensor(DEVICE_LPMS_C, deviceId);
		
#ifdef _WIN32		
		ce.addSensor((LpmsCanIo *)sensor->getIoInterface());
#endif

		sensorList.push_back(sensor);
	break;		
		
	case DEVICE_LPMS_U:	
		sensor = new LpmsSensor(DEVICE_LPMS_U, deviceId);
		sensorList.push_back(sensor);
	break;
	
	case DEVICE_LPMS_RS232:	
		sensor = new LpmsSensor(DEVICE_LPMS_RS232, deviceId);
		((LpmsRS232 *)sensor->getIoInterface())->setRs232Baudrate(currentUartBaudrate);
		sensorList.push_back(sensor);
	break;
	}
	
	lm.unlock();
	
	return (LpmsSensorI*) sensor;
}

void LpmsSensorManager::removeSensor(LpmsSensorI *sensor)
{
	sensor->close();

	lm.lock();	
	sensorList.remove((LpmsSensor*) sensor);
		
#ifdef _WIN32
	ce.removeSensor((LpmsCanIo*) ((LpmsSensor*) sensor)->getIoInterface());
	be.removeSensor((LpmsBle*) ((LpmsSensor*) sensor)->getIoInterface());	
#endif
	
	delete sensor;
	lm.unlock();
}

LpmsDeviceList LpmsSensorManager::getDeviceList(void)
{
	return deviceList;
}

bool LpmsSensorManager::isRecordingActive(void)
{
	return isRecording;
}

bool LpmsSensorManager::saveSensorData(const char* fn)
{
	list<LpmsSensor*>::iterator i;

	if (isRecording == true) return false;	
	
	saveDataHandle.open(fn, ios_base::out);
	saveDataHandle.rdbuf()->pubsetbuf(writeBuffer, 65536);
	if (saveDataHandle.is_open() == true) {	
		saveDataHandle << "SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg), EulerY (deg), EulerZ (deg), QuatW, QuatX, QuatY, QuatZ, LinAccX (g), LinAccY (g), LinAccZ (g), Pressure (kPa), Altitude (m), Temperature (degC), HeaveMotion (m)\n";

		cout << "[LpmsSensorManager] Writing LPMS data to " << fn << endl;	
						
		lm.lock();
		for (i = sensorList.begin(); i != sensorList.end(); ++i) {
			(*i)->startSaveData(&saveDataHandle);
		}
		lm.unlock();		
		
		isRecording = true;
		
		return true;
	}
	
	cout << "[LpmsSensorManager] Failed to open " << fn << endl;	
	
	return false;
}

void LpmsSensorManager::stopSaveSensorData(void)
{
	list<LpmsSensor*>::iterator i;

	if (isRecording == false) return;
	
	lm.lock();
	for (i = sensorList.begin(); i != sensorList.end(); ++i) {
		(*i)->stopSaveData();
	}
	lm.unlock();	

	isRecording = false;
	
	//if (saveDataHandle != NULL) saveDataHandle.close();
	if (saveDataHandle.is_open() ) saveDataHandle.close();
}

void LpmsSensorManager::setThreadTiming(int delay)
{
	if (delay >= 0) {
		threadDelay = delay;
	}
}

bool LpmsSensorManager::isCanPresent(void) 
{
#ifdef _WIN32
	return ce.isInterfacePresent();
#else
	return false;
#endif
}

void LpmsSensorManager::setCanBaudrate(int i)
{
#ifdef _WIN32	
	ce.setBaudrate(i);
#endif
}

void LpmsSensorManager::setRs232Baudrate(int i)
{
	currentUartBaudrate = i;
}
