#include "LpmsBle2.h"

LpmsBle::LpmsBle(CalibrationData *configData) :
	LpmsIoInterface(configData)
{	
	mm.reset();
	oneTx.clear();

	rxState = PACKET_END;
	currentState = IDLE_STATE;
	waitForAck = false;
	waitForData = false;
	ackReceived = false;
	ackTimeout = 0;	
	lpmsStatus = 0;
	configReg = 0;
	dataReceived = false;
	dataTimeout = 0;
	pCount = 0;
	isOpen = false;	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;
	
	started = true;
		
	setConfiguration();
}

LpmsBle::~LpmsBle(void)
{
	close();
}

long long LpmsBle::getConnectWait(void) 
{ 
	return 10000000; 
}

void LpmsBle::listDevices(LpmsDeviceList *deviceList)
{
}

bool LpmsBle::connect(std::string deviceId)
{	
	this->deviceId = deviceId;

	printf("[LpmsBle] Connecting to device %s\n", deviceId);

	return true;
}

void LpmsBle::close(void)
{
	started = false;
	isOpen = false;
}

bool LpmsBle::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1014) return false;

	txData[0] = 0x3a;
	
	txData[1] = function & 0xff;
	txData[2] = length & 0xff;
	
	for (unsigned int i=0; i < length; ++i) {
		txData[3+i] = data[i];
	}
		
	txLrcCheck = function;
	txLrcCheck += length;
	
	for (unsigned int i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[3 + length] = txLrcCheck & 0xff;
	txData[4 + length] = (txLrcCheck >> 8) & 0xff;
		
	for (unsigned int i=0; i<5+length; ++i) txQueue.push(txData[i]);
	
	return false;
}

bool LpmsBle::parseModbusByte(unsigned char b)
{
	switch (rxState) {
	case PACKET_END:
		if (b == 0x3a) {
			rxState = PACKET_FUNCTION0;
			oneTx.clear();
		}
		break;
		
	case PACKET_FUNCTION0:
		currentFunction = b;
		rxState = PACKET_LENGTH0;				
		break;
		
	case PACKET_LENGTH0:
		currentLength = b;
		rxState = PACKET_RAW_DATA;
		rawDataIndex = currentLength;
	break;
							
	case PACKET_RAW_DATA:
		if (rawDataIndex == 0) {
			lrcCheck = currentFunction + currentLength;
			for (unsigned i=0; i<oneTx.size(); i++) lrcCheck += oneTx[i];
			lrcReceived = b;
			rxState = PACKET_LRC_CHECK1;
		} else {
			oneTx.push_back(b);
			--rawDataIndex;
		}
		break;
		
	case PACKET_LRC_CHECK1:
		lrcReceived = lrcReceived + ((unsigned) b * 256);	
		
		if (lrcReceived == lrcCheck) {
			if (currentFunction == GET_SENSOR_DATA) {
				/* long long dt;					
				dt = dataTimer.measure();
				dataTimer.reset();
				avgDt = 0.1f * ((float) dt / 1000.0f) + 0.9f * avgDt; */
			}
		
			parseFunction();
		} else {
			std::cout << "[LPMS-BLE] Checksum fail in data packet" << std::endl;
		}
		
		rxState = PACKET_END;
		break;
	
	default:
		rxState = PACKET_END;		
		return false;
		break;
	}
		
	return true;
}

bool LpmsBle::pollData(void) 
{	
	return true;
}

bool LpmsBle::deviceStarted(void)
{
	return isOpen;
}

bool LpmsBle::parseSensorData(void)
{
	unsigned o=0;
	const float r2d = 57.2958f;
	short iTimestamp;
	short iQuat;
	short iHeave;

	zeroImuData(&imuData); 
	
	fromBufferInt16(oneTx, o, &iTimestamp);
	o = o + 2;
	currentTimestamp = (float) iTimestamp;
	
	if (timestampOffset > currentTimestamp) timestampOffset = currentTimestamp;
	imuData.timestamp = currentTimestamp - timestampOffset;
	
	fromBufferInt16(oneTx, o, &iQuat);
	o = o + 2;
	imuData.q[0] = (float) iQuat / (float) 0x7fff;
	
	fromBufferInt16(oneTx, o, &iQuat);
	o = o + 2;
	imuData.q[1] = (float) iQuat / (float) 0x7fff;

	fromBufferInt16(oneTx, o, &iQuat);
	o = o + 2;
	imuData.q[2] = (float) iQuat / (float) 0x7fff;

	fromBufferInt16(oneTx, o, &iQuat);
	o = o + 2;
	imuData.q[3] = (float) iQuat / (float) 0x7fff;
	
	fromBufferInt16(oneTx, o, &iHeave);
	o = o + 2;
	imuData.displacement[2] = (float) iHeave / (float) 0x0fff;
	
	if (imuDataQueue.size() < 64) {
		imuDataQueue.push(imuData);
	}

	return true;
}

void LpmsBle::setConfiguration(void)
{	
	int selectedData = 0;

	configData->setParameter(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
	configData->setParameter(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
	configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_30HZ);	
	configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
	
	selectedData &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;	
	selectedData |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
	selectedData |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
	selectedData &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;	
	selectedData &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
	selectedData |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
	
	configData->setParameter(PRM_SELECT_DATA, selectedData);
	configData->setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_ENABLED);
}

bool LpmsBle::getTxMessage(BleBlock *b)
{
	BleBlock ble_block;
	int l = 0;

	if (txQueue.size() == 0) return false;
	
	l = txQueue.size();
	
	if (l > 20) {
		l = 20;
	
		for (int i=0; i<l; ++i) {
			ble_block.txData[i] = txQueue.front();
			txQueue.pop();
		}
	} else {				
		int nF = l % 20;
		
		for (int i=0; i<l; ++i) {
			ble_block.txData[i] = txQueue.front();
			txQueue.pop();
		}
		
		if (nF > 0) {
			for (int i=l; i < (l + 20 - nF); ++i) ble_block.txData[i] = 0x0;
			l += 20 - nF;
		}					
	}
		
	ble_block.bleConnectionHandle = bleConnectionHandle;
	ble_block.bleMeasurementHandle = bleMeasurementHandle;
	
	// printf("Sending data from %s with CH: 0x%x, MH: 0x%x\n", deviceId, bleConnectionHandle, bleMeasurementHandle);
			
	*b = ble_block;
	
	return true;
}

void LpmsBle::setConnectionHandle(int v)
{
	bleConnectionHandle = v;
}

int LpmsBle::getConnectionHandle(void)
{
	return bleConnectionHandle;
}

void LpmsBle::setMeasurementHandle(int v)
{
	bleMeasurementHandle = v;
}

int LpmsBle::getMeasurementHandle(void)
{
	return bleMeasurementHandle;
}

void LpmsBle::setConnectionStatus(bool s) 
{
	isOpen = s;
}

bool LpmsBle::isTimeToSend(void)
{
	if (sendTimer.measure() > 50000) {
		return true;
	}
	
	return false;
}

void LpmsBle::resetSendTimer(void)
{
	sendTimer.reset();
}

bool LpmsBle::isReadyToSend(void)
{
	return readyToSend;
}

void LpmsBle::setReadyToSend(bool s)
{
	readyToSend = s;
}

bool LpmsBle::startUploadFirmware(std::string fn)
{
	bool f = false;
	long long l;
	unsigned long r;
	
	if (ifs.is_open() == true) ifs.close();

	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpmsIoInterface] Firmware file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open firmware file %s\n", fn.c_str());
		f = false;
	
		return f;
	}
	
	ifs.seekg(0, ios::end);
	l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] Firmware filesize: %d\n", l);
	
	firmwarePages = l / BLE_FIRMWARE_PACKET_LENGTH;
	r = l % BLE_FIRMWARE_PACKET_LENGTH;
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] Firmware pages: %d\n", firmwarePages);	
	LOGV("[LpmsIoInterface] Firmware remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] Firmware packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_FIRMWARE, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;	
	uploadTimer.reset();
	
	return f;
}

bool LpmsBle::handleFirmwareFrame(void)
{
	uploadTimer.reset();
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
		
		ifs.close();		
		
		return false;
	}

	if (ifs.eof() == true || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;	

		ifs.close();
		
		LOGV("[LpmsIoInterface] Firmware upload finished. Now writing to flash. Please DO NOT detach the power from the device for 15s.\n");
		
		return true;
	}

	LOGV("[LpmsIoInterface] Firmware sending packet %d\n", pCount);
	++pCount;

	for (unsigned i=0; i < BLE_FIRMWARE_PACKET_LENGTH; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, BLE_FIRMWARE_PACKET_LENGTH);
	cLength = BLE_FIRMWARE_PACKET_LENGTH;
	sendModbusData(imuId, UPDATE_FIRMWARE, BLE_FIRMWARE_PACKET_LENGTH, (unsigned char *)cBuffer);

	ackTimeout = 0;
	ackTimer.reset();
	dataTimeout = 0;
	resendI = 0;

	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;	
	
	return true;
}

bool LpmsBle::startUploadIap(std::string fn)
{
	bool f = false;

	if (ifs.is_open() == true) ifs.close();
	
	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpmsIoInterface] IAP file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpmsIoInterface] Could not open IAP file %s\n", fn.c_str());	
		f = false;
	}
	
	ifs.seekg(0, ios::end);
	long long l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpmsIoInterface] IAP filesize: %d\n", l);
	
	firmwarePages = l / BLE_FIRMWARE_PACKET_LENGTH;
	unsigned long r = (long) (l % BLE_FIRMWARE_PACKET_LENGTH);
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpmsIoInterface] IAP pages: %d\n", firmwarePages);
	LOGV("[LpmsIoInterface] IAP remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpmsIoInterface] IAP packets to be sent: %d\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_IAP, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;
	uploadTimer.reset();
	
	return f;
}	

bool LpmsBle::handleIAPFrame(void)
{
	uploadTimer.reset();	
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
			
		ifs.close();	
		
		return false;
	}

	if (ifs.eof() == true || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;

		ifs.close();
		
		LOGV("[LpmsIoInterface] IAP upload finished\n");
		
		return true;		
	}
	
	LOGV("[LpmsIoInterface] Sending IAP packet %d\n", pCount);
	++pCount;		
	
	for (unsigned i=0; i < BLE_FIRMWARE_PACKET_LENGTH; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, BLE_FIRMWARE_PACKET_LENGTH);
	cLength = BLE_FIRMWARE_PACKET_LENGTH;
	sendModbusData(imuId, UPDATE_IAP, BLE_FIRMWARE_PACKET_LENGTH, (unsigned char *)cBuffer);
	
	ackTimeout = 0;
	dataTimeout = 0;
	resendI = 0;
	ackTimer.reset();
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;		
		
	return true;
}
