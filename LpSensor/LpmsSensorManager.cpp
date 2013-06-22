/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsSensorManager.h"

#define SENSOR_UPDATE_PERIOD 750

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

	managerState = SMANAGER_MEASURE;

	boost::thread t(&LpmsSensorManager::run, this);
	
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
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	deviceList.clear();
}

void LpmsSensorManager::startListDevices(void)
{
	if (listDevicesBusy() == true) return;
	
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
	boost::thread t(&LpmsSensorManager::run, this);

#ifdef _WIN32
	#ifdef THREAD_HIGH_PRIORITY
		HANDLE th = t.native_handle();
		SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
	#endif
#endif
	
	t.detach();	
}
	
void LpmsSensorManager::run(void)
{
	MicroMeasure mm;
	list<LpmsSensor*>::iterator i;	
    bool bIsWindows7orLater = true;

#ifdef _WIN32	
    OSVERSIONINFO osvi;

    ZeroMemory(&osvi, sizeof(OSVERSIONINFO));
    osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);

    GetVersionEx(&osvi);

	if ((osvi.dwMajorVersion > 6) || ((osvi.dwMajorVersion == 6) && (osvi.dwMinorVersion >= 1))) {
		bIsWindows7orLater = true;
		LOGV("[LpmsSensorManager] Win7 mode");		
	} else {
		bIsWindows7orLater = false;
		LOGV("[LpmsSensorManager] WinXP mode");
	}
	
	ce.connect();
#endif	

#ifdef ANDROID
	LOGV("[LpmsSensorManager] Thread running");
#endif

	mm.reset();
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
			/* if (bIsWindows7orLater == false && threadDelay < 500) {
				boost::this_thread::sleep(boost::posix_time::microseconds(500));
			} else if (threadDelay > 0) {
				boost::this_thread::sleep(boost::posix_time::microseconds(threadDelay));
			} */
			
			if (bIsWindows7orLater == false) {
				boost::this_thread::sleep(boost::posix_time::microseconds(500));
			}			
#endif		
		break;

		case SMANAGER_LIST:
			deviceList.clear();

#ifdef _WIN32
			ce.listDevices(&deviceList);
#endif

			// LpmsRS232::listDevices(deviceList);
			
			LpmsU::listDevices(&deviceList);
			LpmsBBluetooth::listDevices(&deviceList);

			managerState = SMANAGER_MEASURE;
		break;
		}
	}

#ifdef _WIN32		
	ce.close();
#endif
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
		
		LOGV("[LpmsSensorManager] Sensor added");
#endif

		sensorList.push_back(sensor);
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
	}
	lm.unlock();
	
	return (LpmsSensorI*) sensor;
}

void LpmsSensorManager::removeSensor(LpmsSensorI *sensor)
{
	lm.lock();
	sensorList.remove((LpmsSensor*) sensor);
	
#ifdef _WIN32	
	ce.removeSensor((LpmsCanIo*) ((LpmsSensor*) sensor)->getIoInterface());
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
	
	saveDataHandle = fopen(fn, "w");
	if (saveDataHandle != NULL) {	
		fprintf(saveDataHandle, "SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg), EulerY (deg), EulerZ (deg), QuatX, QuatY, QuatZ, QuatW, LinAccX (m/s^2), LinAccY (m/s^2), LinAccZ (m/s^2), Pressure (hPa), Altitude (m), Temperature (degC), HeaveMotion (m)\n");

		cout << "[LpmsSensorManager] Writing LPMS data to " << fn << endl;	
		
		lm.lock();
		for (i = sensorList.begin(); i != sensorList.end(); ++i) {
			(*i)->startSaveData(saveDataHandle);
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
	
	if (saveDataHandle != NULL) fclose(saveDataHandle);
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
