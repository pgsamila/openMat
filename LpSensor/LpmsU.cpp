/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsU.h"

#define GYRO1X_START 2
#define ACC_START 8
#define MAG_START 14

#define RAW_DATA_LENGTH 20

#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7

#define LPMS_GOTO_BROADCAST 0
#define LPMS_GOTO_SLEEP 1
#define LPMS_RAW_DATA 61

#define LPMS_FACTORY_IMU_ID 1

LpmsU::LpmsU(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
	isOpen = false;
}
	
long long LpmsU::getConnectWait(void) { 
	return 3000000; 
}	
	
void LpmsU::listDevices(LpmsDeviceList *deviceList) 
{
	FT_STATUS ftStatus; 
	FT_DEVICE_LIST_INFO_NODE *devInfo;
	
#ifdef _WIN32
	unsigned long numDevs;
#else
	unsigned int numDevs;
#endif

	ftStatus = FT_CreateDeviceInfoList(&numDevs);
	
	if (ftStatus == FT_OK) { 
		printf("[LpmsU] Number of devices is %d\n", numDevs); 
	} 
	
	if (numDevs > 0) { 
		devInfo = (FT_DEVICE_LIST_INFO_NODE*) malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numDevs);
		ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs); 

		if (ftStatus == FT_OK) { 
			for (unsigned i = 0; i < numDevs; i++) {
				std::cout << "[LpmsU] Discovered device: " << devInfo[i].SerialNumber << std::endl;
				deviceList->push_back(DeviceListItem(devInfo[i].SerialNumber, DEVICE_LPMS_U));
			} 
		} 
	}
}

bool LpmsU::connect(string deviceId)
{
	FT_STATUS ftStatus; 
	bool f;
	unsigned long bytesReceived;
	unsigned char rxBuffer[4096];
	
	this->idNumber = deviceId;
	
	f = false;
	
	ftStatus = FT_OpenEx((PVOID)idNumber.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
	ftStatus = FT_SetBaudRate(ftHandle, 921600);
	ftStatus = FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
	ftStatus = FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x11, 0x13);	
	ftStatus = FT_SetLatencyTimer(ftHandle, 2);
	ftStatus = FT_SetUSBParameters(ftHandle, 64, 0);
				
	if (ftStatus == FT_OK) { 
		setCommandMode();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		read(rxBuffer, &bytesReceived);
		FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
	}

	if (ftStatus == FT_OK) { 
		cout << "[LpmsU] Connection to " << idNumber << " successful." << endl;
		isOpen = true;
	} else {
		cout << "[LpmsU] Connection to " << idNumber << " failed." << endl;
		isOpen = false;
	}
		
	oneTx.clear();
	
	rxState = PACKET_END;
	currentState = GET_CONFIG;
	
	waitForAck = false;
	ackReceived = false;
	waitForData = false;
	dataReceived = false;
	
	pCount = 0;
	ackTimeout = 0;
	dataTimeout = 0;	

	lpmsStatus = 0;
	configReg = 0;
	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;
	
	return f;
}

void LpmsU::close(void)
{
	if (isOpen == false) return;
	
	stopStreaming();	
	isOpen = false;
	FT_Close(ftHandle); 
	
	cout << "[LpmsU] Connection to " << idNumber << " closed." << endl;
}	

bool LpmsU::read(unsigned char *rxBuffer, unsigned long *bytesReceived) 
{
	FT_STATUS ftStatus; 

#ifdef _WIN32
	unsigned long eventDWord; 
	unsigned long txBytes; 
	unsigned long rxBytes; 
#else
	unsigned int eventDWord; 
	unsigned int txBytes; 
	unsigned int rxBytes; 	
#endif

	bool f = true;

	*bytesReceived = 0;

	if (isOpen == false) return false;
	
	if (FT_GetStatus(ftHandle, &rxBytes, &txBytes, &eventDWord) != FT_OK) {
		std::cout << "[LpmsU] Read failed!" << std::endl;
		return false;
	}

	if (rxBytes > 4096) {
		std::cout << "[LpmsU] Buffer overflow!" << std::endl;
		rxBytes = 4096;
	}
	
	if (rxBytes > 0) {
#ifdef _WIN32
		ftStatus = FT_Read(ftHandle, rxBuffer, rxBytes, bytesReceived);
#else
		ftStatus = FT_Read(ftHandle, rxBuffer, rxBytes, (unsigned int *) bytesReceived);
#endif
		
		if (ftStatus == FT_OK) { 
			f = true;
		} else {
			f = false;
		}
	}
	
	return f;
}

bool LpmsU::write(unsigned char *txBuffer, unsigned bufferLength)
{
	FT_STATUS ftStatus; 
	
#ifdef _WIN32
	unsigned long bytesWritten;
#else
	unsigned int bytesWritten;
#endif

	bool f = false;

	if (isOpen == false) return false;
	
	ftStatus = FT_Write(ftHandle, txBuffer, bufferLength, &bytesWritten);
		
	if (ftStatus == FT_OK) {	
		f = true;
	}
	
	return f;
}

bool LpmsU::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	unsigned char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1014) return false;

	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	txData[2] = (address >> 8) & 0xff;
	txData[3] = function & 0xff;
	txData[4] = (function >> 8) & 0xff;
	txData[5] = length & 0xff;
	txData[6] = (length >> 8) & 0xff;
	
	for (unsigned int i=0; i < length; ++i) {
		txData[7+i] = data[i];
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	for (unsigned int i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[7 + length] = txLrcCheck & 0xff;
	txData[8 + length] = (txLrcCheck >> 8) & 0xff;
	txData[9 + length] = 0x0d;
	txData[10 + length] = 0x0a;
	
	/* printf("Sent: ");
	for (int i=0; i<10+length; ++i) {
		printf("%x ", txData[i]);
	}
	printf("\n"); */
	
	if (write(txData, length+11) == true) {
		return true;
	}

	return false;
}

bool LpmsU::parseModbusByte(void)
{	
	unsigned char b;

	while (dataQueue.size() > 0) {	
		b = dataQueue.front();
		dataQueue.pop();

		switch (rxState) {
		case PACKET_END:
			if (b == 0x3a) {
				rxState = PACKET_ADDRESS0;
				oneTx.clear();
			}
			break;
			
		case PACKET_ADDRESS0:
			currentAddress = b;
			rxState = PACKET_ADDRESS1;
			break;

		case PACKET_ADDRESS1:
			currentAddress = currentAddress + ((unsigned) b * 256);
			rxState = PACKET_FUNCTION0;
			break;

		case PACKET_FUNCTION0:
			currentFunction = b;
			rxState = PACKET_FUNCTION1;				
			break;

		case PACKET_FUNCTION1:
			currentFunction = currentFunction + ((unsigned) b * 256);
			rxState = PACKET_LENGTH0;			
		break;

		case PACKET_LENGTH0:
			currentLength = b;
			rxState = PACKET_LENGTH1;
		break;
				
		case PACKET_LENGTH1:
			currentLength = currentLength + ((unsigned) b * 256);
			rxState = PACKET_RAW_DATA;
			rawDataIndex = currentLength;
		break;
				
		case PACKET_RAW_DATA:
			if (rawDataIndex == 0) {
				lrcCheck = currentAddress + currentFunction + currentLength;
				for (unsigned i=0; i<oneTx.size(); i++) {
					lrcCheck += oneTx[i];
				}
				
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
				parseFunction();
			} else {
				cout << "[LpmsU] Checksum fail in data packet" << endl;
			}
			
			rxState = PACKET_END;
			break;
		
		default:
			rxState = PACKET_END;		
			return false;
			break;
		}
	}
	
	return true;
}

bool LpmsU::pollData(void) 
{
	unsigned long bytesReceived;
	unsigned char rxBuffer[4096];
	bool packetOk = false;
	
	if (deviceStarted() == false) return false;

	if (read(rxBuffer, &bytesReceived) == false) {
		isOpen = false;
		return false;
	}	

	if (bytesReceived > 0) {
		// printf("Received: ");
		for (unsigned int i=0; i < bytesReceived; i++) {
			dataQueue.push((unsigned char) rxBuffer[i]);
			// printf("%x ", rxBuffer[i]);		
		}
		// printf("\n");
	}
		
	return true;
}

bool LpmsU::deviceStarted(void)
{
	return isOpen;
}
