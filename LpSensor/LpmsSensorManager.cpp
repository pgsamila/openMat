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

#define SENSOR_UPDATE_PERIOD 500

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
	isSensorSyncOn = false;

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
#define SYNC_PERIOD 5000000
	
void LpmsSensorManager::run(void)
{
	MicroMeasure mm;
	MicroMeasure syncPeriodTimer;	
	list<LpmsSensor*>::iterator i;	
    bool bIsWindows7orLater = true;
	float prevTimestamp = 0.0f;
	int deviceType = 0;

#ifdef _WIN32	
    OSVERSIONINFO osvi;

    ZeroMemory(&osvi, sizeof(OSVERSIONINFO));
    osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);

    GetVersionEx(&osvi);

	if ((osvi.dwMajorVersion > 6) || ((osvi.dwMajorVersion == 6) && (osvi.dwMinorVersion >= 1))) {
		bIsWindows7orLater = true;
		LOGV("[LpmsSensorManager] Windows 7 and above mode\n");		
	} else {
		bIsWindows7orLater = false;
		LOGV("[LpmsSensorManager] Windows XP mode\n");
	}
	
	ce.connect();
	be.connect();
#endif	

#ifdef ANDROID
	LOGV("[LpmsSensorManager] Thread running\n");
#endif

	mm.reset();
	syncTimer.reset();
	syncPeriodTimer.reset();	
	
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
			
			if (syncPeriodTimer.measure() > SYNC_PERIOD) {
				syncPeriodTimer.reset();

				lm.lock();				
				for (i = sensorList.begin(); i != sensorList.end(); i++) {
					(*i)->setCurrentSyncOffset((*i)->getCurrentData().timeStamp - (*(sensorList.begin()))->getCurrentData().timeStamp);

					if (isSensorSyncOn == true) {
						(*i)->getConfigurationPrm(PRM_DEVICE_TYPE, &deviceType);
						if (deviceType == DEVICE_LPMS_B) {
							(*i)->syncTimestamp((*(sensorList.begin()))->getCurrentData().timeStamp + LPMS_B_LATENCY_ESTIMATE);
						} else {
							(*i)->syncTimestamp((*(sensorList.begin()))->getCurrentData().timeStamp + LPMS_OTHER_LATENCY_ESTIMATE);
						}
						printf("[LpmsSensorManager] ID=%d, timestamp offset=%f\n", (*i)->getOpenMatId(), (*i)->getCurrentSyncOffset());
					}
				}
				lm.unlock();
			}

			if (mm.measure() > SENSOR_UPDATE_PERIOD) {			
				mm.reset();				
				
				lm.lock();
				for (i = sensorList.begin(); i != sensorList.end(); i++) {
					(*i)->update();				
				}
				lm.unlock();
			}
			
#ifdef _WIN32
			if (bIsWindows7orLater == false) {
				std::this_thread::sleep_for(std::chrono::microseconds(500));
			}			
#endif		
		break;

		case SMANAGER_LIST:
			deviceList.clear();

#ifdef _WIN32
			ce.listDevices(&deviceList);
			be.listDevices(&deviceList);			
#endif

			if (scan_serial_ports_ == true) {
				LpmsRS232::listDevices(&deviceList);
			}
			LpmsU::listDevices(&deviceList);
			LpmsBBluetooth::listDevices(&deviceList);			

			managerState = SMANAGER_MEASURE;
		break;
		}
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

void LpmsSensorManager::setSensorSync(bool s)
{
	list<LpmsSensor*>::iterator i;

	if (s == true) {
		printf("[LpmsSensorManager] Set sensor sync ON\n");
	} else {
		printf("[LpmsSensorManager] Set sensor sync OFF\n");
	}
	
	lm.lock();
	sensorList.sort(compareOpenMatId);
	lm.unlock();
	
	isSensorSyncOn = s;
	
	if (isSensorSyncOn == true) {
		lm.lock();				
		for (i = sensorList.begin(); i != sensorList.end(); i++) {
			(*i)->setCurrentSyncOffset((*i)->getCurrentData().timeStamp - (*(sensorList.begin()))->getCurrentData().timeStamp);

			(*i)->syncTimestamp((*(sensorList.begin()))->getCurrentData().timeStamp);
		}
		lm.unlock();
	}	
}

bool LpmsSensorManager::getSensorSync(void)
{
	return isSensorSyncOn;
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
		
		((LpmsBle *)sensor->getIoInterface())->deviceId = deviceId;
		be.addSensor((LpmsBle *)sensor->getIoInterface());
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
		sensorList.push_back(sensor);
	break;
	}
	
	sensorList.sort(compareOpenMatId);
	sensor->setCurrentSyncOffset(0.0f);
	
	lm.unlock();
	
	return (LpmsSensorI*) sensor;
}

void LpmsSensorManager::removeSensor(LpmsSensorI *sensor)
{
	lm.lock();
	sensorList.remove((LpmsSensor*) sensor);
	
#ifdef _WIN32	
	ce.removeSensor((LpmsCanIo*) ((LpmsSensor*) sensor)->getIoInterface());
	be.removeSensor((LpmsBle*) ((LpmsSensor*) sensor)->getIoInterface());	
#endif
	
	sensor->close();
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
		saveDataHandle << "SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg), EulerY (deg), EulerZ (deg), QuatW, QuatX, QuatY, QuatZ, LinAccX (m/s^2), LinAccY (m/s^2), LinAccZ (m/s^2), Pressure (hPa), Altitude (m), Temperature (degC), HeaveMotion (m)\n";

		cout << "[LpmsSensorManager] Writing LPMS data to " << fn << endl;	
		
		syncTimer.reset();
		
		lm.lock();				
		for (i = sensorList.begin(); i != sensorList.end(); i++) {
			(*i)->syncTimestamp(0.0f);
			(*i)->setCurrentSyncOffset(0);
		}
		lm.unlock();		
		
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
	
	if (saveDataHandle != NULL) saveDataHandle.close();
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