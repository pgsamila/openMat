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

#include "LpemgSensor.h"

const float pi = 3.141592f;

/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/
#ifdef WIN32
	LpemgSensorI* APIENTRY LpemgSensorFactory(int deviceType, const char *deviceId) 
	{
		return (LpemgSensorI*) new LpemgSensor(deviceType, deviceId);
	}

	LpemgSensor::LpemgSensor(int deviceType, const char *deviceId) :
#endif

#ifdef __GNUC__
	LpemgSensorI* LpemgSensorFactory(int deviceType, const char *deviceId) 
	{
		return (LpemgSensorI*) new LpemgSensor(deviceType, deviceId);
	}

	LpemgSensor::LpemgSensor(int deviceType, const char *deviceId) :
#endif
	
#ifdef ANDROID
	LpemgSensorI* LpemgSensorFactory(int deviceType, const char *deviceId) 
	{
		return (LpemgSensorI*) new LpemgSensor(deviceType, deviceId);
	}

	LpemgSensor::LpemgSensor(int deviceType, const char *deviceId, JavaVM *thisVm, jobject bluetoothAdapter) :
#endif

	deviceId(deviceId),
	deviceType(deviceType) {	

	this->configData.setParameter(PRM_DEVICE_ID, deviceId);
	this->configData.setParameter(PRM_DEVICE_TYPE, deviceType);
	this->configData.setParameter(PRM_OPENMAT_ID, 1);

	switch (deviceType) {
	
#ifdef ANDROID
	case DEVICE_LPEMG_B:	
		bt = new AndroidBluetooth(&(this->configData), thisVm, bluetoothAdapter);
		LOGV("[LpemgSensor] Sensor initialized\n");
		break;
#endif
	
#ifdef _WIN32
	case DEVICE_LPEMG_B:
		LOGV("[LpemgSensor] Initializing sensor\n");	
		bt = new LpemgBluetooth(&(this->configData));
		LOGV("[LpemgSensor] Sensor initialized\n");		
	break;
#endif

#ifdef __GNUC__
	case DEVICE_LPEMG_B:
		bt = new LpemgBluetooth(&(this->configData));
	break;
#endif
	
	default:
		bt = new LpemgIoInterface(&(this->configData));
		break;
	}	
	
	paused = false;
	stopped = false;
	
	setSensorStatus(SENSOR_STATUS_PAUSED);
	setConnectionStatus(SENSOR_CONNECTION_CONNECTING);
	
	frameNo = 0;
	configReg = 0;
	
	state = STATE_CONNECT;
	retrialsConnect = 0;	

	isFirmwareUpdated = false;
	isSaveData = false;
	callbackSet = false;
	timestampOffset = 0.0f;
	frameCounterOffset = 0;
	currentOffsetResetMethod = 2;
	
	bt->zeroData(&currentData);
}

LpemgSensor::~LpemgSensor(void)
{	
	close();

	delete bt;
}

/***********************************************************************
** POLL / UPDATE DATA FROM SENSORS
***********************************************************************/
void LpemgSensor::pollData(void)
{
	if (bt->deviceStarted() == true) {		
		bt->pollData();
		bt->checkState();
	}
}

void LpemgSensor::assertConnected(void)
{
	if (bt->deviceStarted() == false) {	
		bt->close();
		state = STATE_CONNECT;
		LOGV("[LpemgSensor] Re-connecting..\n");	
	}
}

void LpemgSensor::update(void)
{
	EmgData emgData;
	int p;
		
	if (stopped == true) return;	
		
	switch (state) {
	// Initiates the connection to the sensor using the hardware interface
	case STATE_CONNECT:
		setConnectionStatus(SENSOR_CONNECTION_CONNECTING);
		setSensorStatus(SENSOR_STATUS_PAUSED);
		lpmsTimer.reset();		
		bt->connect(deviceId);
		LOGV("[LpemgSensor] Trying to connect..\n");
		retrialsCommandMode = 0;
		prepareStream = 0;
		state = STATE_WAIT_CONNECT;		
	break;
		
	// Waits for a successful connect for a certain time period
	case STATE_WAIT_CONNECT:
		if (lpmsTimer.measure() < bt->getConnectWait() && bt->deviceStarted() == false) {
			state = STATE_WAIT_CONNECT;
		} else {
			if (bt->deviceStarted() == false) {		
				setConnectionStatus(SENSOR_CONNECTION_FAILED);
				state = STATE_CONNECT;		
				LOGV("[LpemgSensor] Connection failed (timeout)..\n");			
			} else {			
				state = STATE_WAIT_AFTER_CONNECT;
			}
		}
	break;
	
	// Waits for a certains period after the connect 
	case STATE_WAIT_AFTER_CONNECT:
		if (lpmsTimer.measure() > WAIT_AFTER_CONNECT) {
			lpmsTimer.reset();	
			LOGV("[LpemgSensor] Waiting after connect..\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_SETTINGS_DONE; // C_STATE_GOTO_COMMAND_MODE;
		}
	break;

	// Retrieves the current parameter settings of the sensor
	case STATE_GET_SETTINGS:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			lpmsTimer.reset();
			
			switch (getConfigState) {
			// Switches to command mode.
			case C_STATE_GOTO_COMMAND_MODE:
				LOGV("[LpemgSensor] Switch to command mode\n");
				bt->setCommandMode();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_FIRMWARE_VERSION;
			break;
			
			// Retrieves firmware version
			case C_STATE_GET_FIRMWARE_VERSION:
				LOGV("[LpemgSensor] Get firmware version\n");
				bt->getFirmwareVersion();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_GET_CONFIG;			
			break;			
			
			// Gets the current configuration word
			case C_STATE_GET_CONFIG:
				LOGV("[LpemgSensor] Get configuration data\n");
				bt->getConfig();
				state = STATE_GET_SETTINGS;
				getConfigState = C_STATE_SETTINGS_DONE;
			break;
			
			// Resets the timer and retrieves the field map (soft/hard iron calibration parameters)
			case C_STATE_SETTINGS_DONE:	
				LOGV("[LpemgSensor] Done reading configuration\n");			
			
				lpmsTimer.reset();
				statusTimer.reset();
					
				bt->startStreaming();
				
				setSensorStatus(SENSOR_STATUS_RUNNING);	
				
				retrialsConnect = 0;
				retrialsCommandMode = 0;
				
				state = STATE_MEASURE;
				
				configData.print();
				
				if (isFirmwareUpdated == true) {
					loadCalibrationData(LPEMG_FIRMWARE_BACKUP_FILE);
					isFirmwareUpdated = false;
				}
			break;
			} 
		}
		
		assertConnected();
	break;	
		
	// Main measurement state
	case STATE_MEASURE:
		assertConnected();		
		
		// Start next measurement step only if program is not waiting for data or ACK
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			if (bt->getMode() != SELECT_LPMS_MODE_STREAM) {
				bt->setStreamMode();
				prepareStream = 0;
			}
		}
		
		// TODO: Insert error handling for sensor.
		/* if (bt->isError() == true) {
			setSensorStatus(SENSOR_STATUS_ERROR);
		} */

		if (paused == true) {
			break;
		}
		
		if (prepareStream < STREAM_N_PREPARE) {
			++prepareStream;
			break;
		}
				
		// Load current data from hardware and calculate rotation matrix and Euler angle
		if (bt->getLatestData(&emgData) == false) break;
		
		frameTime = lpmsTimer.measure() / 1000.0f;	
		lpmsTimer.reset();	
		setFps(frameTime);		
		
		// Add frame number timestamp and IMU ID to current EmgData
		++frameNo;
		emgData.frameCount = frameNo;	
		emgData.openMatId = configData.openMatId;				

		setConnectionStatus(SENSOR_CONNECTION_CONNECTED);
		if (paused == false) {
			setSensorStatus(SENSOR_STATUS_RUNNING);
		} else {
			setSensorStatus(SENSOR_STATUS_PAUSED);
		}
		
		// Sets current data
		setCurrentData(emgData);
		
		// Checks, if data saving is active
		checkSaveData();
	break;
	
	// Prepares parameter adjustment by switching to command mode
	case PREPARE_PARAMETER_ADJUSTMENT:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			bt->setCommandMode();
			state = getConfigState;
		}
	break;
	
	// Enables / disables gyroscope threshold
	case STATE_SET_CONFIG:

	// Sets OpenMAT ID
	case STATE_SET_OPENMAT_ID:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			configData.getParameter(PRM_OPENMAT_ID, &p);
			bt->setImuId(p);
			LOGV("[LpemgSensor] Set OpenMAT ID\n");
			state = STATE_SET_SAMPLING_RATE;
		}
	break;

	// Sets sampling rate
	case STATE_SET_SAMPLING_RATE:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			configData.getParameter(PRM_SAMPLING_RATE, &p);
			bt->setStreamFrequency(p);
			LOGV("[LpemgSensor] Set sampling rate\n");
			state = STATE_SELECT_DATA;
		}
	break;	
	
	// Selects transmission data
	case STATE_SELECT_DATA:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			configData.getParameter(PRM_SELECT_DATA, &p);
			bt->selectData(p);
			printf("[LpemgSensor] Select data 0x%x\n", p);
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
		
	// Writes parameters to the sensor flash memory
	case STATE_WRITE_PARAMETERS:	
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			bt->writeRegisters();
			setSensorStatus(SENSOR_STATUS_CALIBRATING);
			
			state = STATE_MEASURE;
		}
	break;
	
	// Starts uploading firmware
	case STATE_UPLOAD_FIRMWARE:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			bt->startUploadFirmware(firmwareFilename);		
			state = STATE_CHECK_FIRMWARE_UPLOAD;
			setSensorStatus(SENSOR_STATUS_UPLOADING);
		}
	break;
	
	// Starts checking state of firmware upload
	case STATE_CHECK_FIRMWARE_UPLOAD:
		if (bt->getCurrentState() == UPDATE_FIRMWARE) {
		} else {
			state = STATE_WAIT_FIRMWARE_WRITE;
			lpmsTimer.reset();
		}
	break;
	
	// Waits for firmware writing to be finished (10s)
	case STATE_WAIT_FIRMWARE_WRITE:
		if (lpmsTimer.measure() > WAIT_FIRMWARE_WRITE_TIME) {
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GOTO_COMMAND_MODE;
			if (paused == false) {
				isFirmwareUpdated = true;
				setSensorStatus(SENSOR_STATUS_RUNNING);
			} else {
				setSensorStatus(SENSOR_STATUS_PAUSED);
			}
		}
	break;	

	// Starts uploading the IAP
	case STATE_UPLOAD_IAP:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			bt->startUploadIap(iapFilename);		
			state = STATE_CHECK_IAP_UPLOAD;
			setSensorStatus(SENSOR_STATUS_UPLOADING);
		}
	break;
	
	// Starts checking the status of the IAP upload
	case STATE_CHECK_IAP_UPLOAD:
		if (bt->getCurrentState() == UPDATE_IAP) {
		} else {
			state = STATE_WAIT_IAP_WRITE;
			lpmsTimer.reset();
		}
	break;
	
	// Waits for IAP writing to be finished
	case STATE_WAIT_IAP_WRITE:
		if (lpmsTimer.measure() > WAIT_IAP_WRITE_TIME) {
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GOTO_COMMAND_MODE;
			if (paused == false) {
				setSensorStatus(SENSOR_STATUS_RUNNING);
			} else {
				setSensorStatus(SENSOR_STATUS_PAUSED);
			}
		}
	break;
	
	// Initiates self-test
	case STATE_SET_SELF_TEST:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			configData.getParameter(PRM_SELF_TEST, &p);
			bt->setSelfTest(p);
			state = STATE_MEASURE;			
		}
	break;
	
	// Retrieves communication latency
	case STATE_GET_LATENCY:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {		
			setSensorStatus(SENSOR_STATUS_CALIBRATING);
		
			bt->getConfig();
			
			if (latencyCounter == 0) {
				accLatency = 0.0f;
			} else if (latencyCounter > 0 && latencyCounter < 50) {
				accLatency += bt->getLatestLatency();
			} else {
				avgLatency = accLatency / (float)(latencyCounter-1) / 2.0f;
				std::cout << "[LpemgSensor] Average latency: " << avgLatency << "ms" << std::endl;
				state = STATE_MEASURE;	
			}
			
			latencyCounter++;
		}
	break;
	
	// Restores factory defaults
	case STATE_RESET_TO_FACTORY_DEFAULTS:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			bt->restoreFactoryValue();
			LOGV("[LpemgSensor] Reset factory defaults\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
	
	// Resets sensor timestamp
	case STATE_RESET_TIMESTAMP:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
			bt->setTimestamp(0.0f);
			LOGV("[LpemgSensor] Reset sensor timestamp\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
	
	case STATE_ARM_TIMESTAMP_RESET:
		if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {	
			bt->armTimestampReset(LPMS_ARM_TIMESTAMP_RESET);
			LOGV("[LpemgSensor] Arm hardware timestamp reset\n");
			
			state = STATE_GET_SETTINGS;
			getConfigState = C_STATE_GET_CONFIG;
		}
	break;
	
	// Error state
	case STATE_NONE:
		state = STATE_NONE;
	break;
	
	default:
	break;
	}
}

/***********************************************************************
** DIRECT GET / SET DEVICE PARAMETERS
***********************************************************************/
void LpemgSensor::getDeviceId(char *str)
{
	std::string deviceId;

	configData.getParameter(PRM_DEVICE_ID, &deviceId);
	
	strcpy(str, deviceId.c_str());
}

CalibrationData *LpemgSensor::getConfigurationData(void)
{
	return &configData;
}

bool LpemgSensor::assertFwVersion(int d0, int d1, int d2)
{
	if (configData.firmwareVersionDig0 > d0) return true;
	if (configData.firmwareVersionDig0 == d0 && configData.firmwareVersionDig1 > d1) return true;
	if (configData.firmwareVersionDig0 == d0 && configData.firmwareVersionDig1 == d1 && configData.firmwareVersionDig2 >= d2) return true;
	
	return false;
}

void LpemgSensor::setSensorStatus(int s) 
{	
	sensorStatus = s;
}

int LpemgSensor::getSensorStatus(void) 
{	
	return sensorStatus;
}

void LpemgSensor::setConnectionStatus(int s)
{
	connectionStatus = s;
}

int LpemgSensor::getConnectionStatus(void)
{
	return connectionStatus;
}

void LpemgSensor::setCurrentData(EmgData d)
{
	sensorMutex.lock();
	
	currentData = d;
	
	if (emgDataQueue.size() < 64) { 
		emgDataQueue.push(d);
	} else {		
		emgDataQueue.pop();
		emgDataQueue.push(d);
	}
	
	if (callbackSet == true) {
		lpemgCallback(d, deviceId.c_str());
	}
	
	sensorMutex.unlock();	
}

void LpemgSensor::setCallback(LpemgCallback cb)
{
	lpemgCallback = cb;
	callbackSet = true;
}

int LpemgSensor::hasData(void)
{
	return emgDataQueue.size();
}

void LpemgSensor::getSensorData(SensorData *d)
{
	EmgData* ed = (EmgData*) d;
	
	bt->zeroData(ed);
	
	sensorMutex.lock();
	
	if (emgDataQueue.size() > 0) {
		*ed = emgDataQueue.front();
		emgDataQueue.pop();
	} else {
		*ed = currentData;
	}
	
	sensorMutex.unlock();
}

bool LpemgSensor::isRunning(void)
{
	return !paused;
}

void LpemgSensor::pause(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	setSensorStatus(SENSOR_STATUS_PAUSED);
	paused = true;
}

void LpemgSensor::run(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	setSensorStatus(SENSOR_STATUS_RUNNING);	
	paused = false;
}

void LpemgSensor::close(void)
{
	stopped = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	bt->close();
}
	
void LpemgSensor::setOpenMatId(int id)
{
	configData.openMatId = id;
}

int LpemgSensor::getOpenMatId(void)
{
	return configData.openMatId;
}

bool LpemgSensor::updateParameters(void)
{
	bool r = true;

	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;
	
	if (state != STATE_MEASURE) {
		return false;
	}
		
	state = PREPARE_PARAMETER_ADJUSTMENT;
	getConfigState = STATE_SET_CONFIG;
	
	return r;
}

bool LpemgSensor::setConfigurationPrm(int parameterIndex, int parameter)
{	
	bool f = true;
	
	configData.setParameter(parameterIndex, parameter);

	switch (parameterIndex) {
	case PRM_SELF_TEST:
		state = PREPARE_PARAMETER_ADJUSTMENT;	
		getConfigState = STATE_SET_SELF_TEST;
	break;

	default:
		f = updateParameters();
	break;
	} 

	return f;
}

bool LpemgSensor::setConfigurationPrm(int parameterIndex, int *parameter)
{	
	bool f = true;
		
	configData.setParameter(parameterIndex, parameter);	
	
	f = updateParameters();
	
	return f;
}

bool LpemgSensor::getConfigurationPrm(int parameterIndex, int* parameter)
{
	return configData.getParameter(parameterIndex, parameter);
}

bool LpemgSensor::getConfigurationPrm(int parameterIndex, char* parameter)
{
	std::string cppStr;
	
	const bool ret = configData.getParameter(parameterIndex, &cppStr);
	
	strcpy(parameter, cppStr.c_str());
	
	return ret;
}

LpmsIoInterface *LpemgSensor::getIoInterface(void)
{
	return (LpmsIoInterface *)bt;
}

void LpemgSensor::measureAvgLatency(void) 
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	latencyCounter = 0;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_GET_LATENCY;
}

void LpemgSensor::resetToFactorySettings(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_RESET_TO_FACTORY_DEFAULTS;
}

long LpemgSensor::getStreamFrequency(void)
{
	int i;

	getConfigurationPrm(PRM_SAMPLING_RATE, &i);
	return i;
}

void LpemgSensor::setFps(float f) 
{	
	currentFps = f;
}

float LpemgSensor::getFps(void) 
{	
	return currentFps;
}

/***********************************************************************
** FIRMWARE / IAP
***********************************************************************/
bool LpemgSensor::uploadFirmware(const char *fn)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;
	
	saveCalibrationData(LPEMG_FIRMWARE_BACKUP_FILE);
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_UPLOAD_FIRMWARE;
	firmwareFilename = fn;
	
	return true;
}

bool LpemgSensor::uploadIap(const char *fn)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_UPLOAD_IAP;
	iapFilename = fn;
	
	return true;
}

int LpemgSensor::getUploadProgress(int *p) 
{
	if (state != STATE_CHECK_IAP_UPLOAD &&
		state != STATE_WAIT_IAP_WRITE &&
		state != STATE_UPLOAD_IAP &&
		state != STATE_CHECK_FIRMWARE_UPLOAD &&
		state != STATE_WAIT_FIRMWARE_WRITE &&
		state != STATE_UPLOAD_FIRMWARE) {
		return 0;
	}
	
	if (bt->getUploadProgress(p) == false) return 2;
	
	if (state == STATE_WAIT_FIRMWARE_WRITE) {
		*p = *p + (int) (lpmsTimer.measure() * 100 / WAIT_FIRMWARE_WRITE_TIME);
	} 

	if (state == STATE_WAIT_IAP_WRITE) {
		*p = *p + (int) (lpmsTimer.measure() * 100 / WAIT_IAP_WRITE_TIME);
	} 

	return 1;
}

/***********************************************************************
** CALIBRATION
***********************************************************************/
void LpemgSensor::saveCalibrationData(void)	
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_WRITE_PARAMETERS;	
}

void LpemgSensor::saveCalibrationData(const char* fn)
{
	printf("[LpemgSensor] Saving calibration data to %s\n", fn);
	configData.save(fn);
}

void LpemgSensor::loadCalibrationData(const char* fn)
{
	printf("[LpemgSensor] Loading calibration data from %s\n", fn);
	configData.load(fn);
	updateParameters();
}

void LpemgSensor::resetTimestamp(void)
{
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
	
	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_RESET_TIMESTAMP;
	
	frameNo = 0;
}

void LpemgSensor::setTimestamp(float t)
{
	bt->setTimestamp(t);
}

void LpemgSensor::armTimestampReset(void)
{	
	if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

	state = PREPARE_PARAMETER_ADJUSTMENT;	
	getConfigState = STATE_ARM_TIMESTAMP_RESET;
}


/***********************************************************************
** DATA RECORDING
***********************************************************************/
void LpemgSensor::startSaveData(std::ofstream *saveDataHandle)
{
	sensorMutex.lock();
	bt->clearDataQueue();
	isSaveData = true;
	this->saveDataHandle = saveDataHandle;
	frameNo = 0;
	saveDataPreroll = 10;
	sensorMutex.unlock();
}

void LpemgSensor::checkSaveData(void)
{
	float currentTimestamp;

	if (saveDataPreroll > 1) {
		--saveDataPreroll;
		return;
	} else if (saveDataPreroll == 1) {
		--saveDataPreroll;
		timestampOffset = currentData.t;
		frameCounterOffset = currentData.frameCount;	
	}
	
	// currentTimestamp = (currentData.timeStamp-timestampOffset);
	currentTimestamp = currentData.t;

	sensorMutex.lock();
	if (isSaveData == true && saveDataHandle->is_open() == true) {
		*saveDataHandle << currentData.openMatId << ", " << std::fixed << std::setprecision(6) << currentTimestamp << ", " << (currentData.frameCount-frameCounterOffset) << ", " << currentData.u << std::endl;
	}
	sensorMutex.unlock();
}

void LpemgSensor::stopSaveData(void)
{
	isSaveData = false;
}