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

#include "LpmsBBluetooth.h"

LpmsBBluetooth::LpmsBBluetooth(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
}

LpmsBBluetooth::~LpmsBBluetooth(void)
{
	close();
}

long long LpmsBBluetooth::getConnectWait(void) 
{ 
	return 6000000;
}

void LpmsBBluetooth::listDevices(LpmsDeviceList *deviceList)
{
	inquiry_info *ii = NULL;
	int max_rsp, num_rsp;
	int dev_id, sock, len, flags;
	int i;
	char addr[19] = { 0 };
	char name[248] = { 0 };

	dev_id = hci_get_route(NULL);
	sock = hci_open_dev( dev_id );
	
	if (dev_id < 0 || sock < 0) {
		std::cout << "[LpmsBBluetooth] Error opening socket" << std::endl;
		exit(1);
	}

	len  = 8;
	max_rsp = 255;
	flags = IREQ_CACHE_FLUSH;
	ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));

	num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
	if (num_rsp < 0) {
		std::cout << "[LpmsBBluetooth] Error with hci_inquiry" << std::endl;
	}

	for (i = 0; i < num_rsp; i++) {
		ba2str(&(ii+i)->bdaddr, addr);
		memset(name, 0, sizeof(name));
		
		if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), name, 0) < 0) strcpy(name, "[unknown]");
		
		if (std::string(name) == std::string("LPMS-B")) {
			deviceList->push_back(DeviceListItem(addr, DEVICE_LPMS_B));
		}

		std::cout << "[LpmsBBluetooth] Address: " << addr << " name: " << name << std::endl;
	}

	(void) deviceList;

	free(ii);
	::close(sock);
}

void LpmsBBluetooth::stopDiscovery(void)
{
}

bool LpmsBBluetooth::connect(string deviceId)
{	
	this->bluetoothAddress = deviceId;

	int status;
	int on;
	int rc;
	struct sockaddr_rc addr = { 0 };

	bzSocket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba(deviceId.c_str(), &addr.rc_bdaddr);

	std::cout << "[LpmsBBluetooth] Connecting to: " << deviceId.c_str() << std::endl;
	status = ::connect(bzSocket, (struct sockaddr *)&addr, sizeof(addr));
	
	if (status < 0) {
		std::cout << "[LpmsBBluetooth] Couldn't create socket." << std::endl;
		::close(bzSocket);
		return false;
	}
	std::cout << "[LpmsBBluetooth] Connected!" << std::endl;
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
	isOpen = true;
	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;

	std::thread t(&LpmsBBluetooth::runRead, this);	
	t.detach();
	
	return true;
}

void LpmsBBluetooth::close(void)
{
	isOpen = false;
	
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	::close(bzSocket);
}

void LpmsBBluetooth::runRead(void)
{
	unsigned long bytesReceived;
	char rxBuffer[1024];

	while (isOpen == true) {
		bytesReceived = ::read(bzSocket, rxBuffer, 8);

		for (unsigned int i=0; i < bytesReceived; i++) {
			dataQueue.push((unsigned char) rxBuffer[i]);
		}
	}
}

bool LpmsBBluetooth::read(char *rxBuffer, unsigned long *bytesReceived) 
{
	return true;
}

bool LpmsBBluetooth::write(char *txBuffer, unsigned bufferLength)
{	
	int s;

	s = ::write(bzSocket, txBuffer, bufferLength);

	return true;
}

bool LpmsBBluetooth::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	char txData[1024];
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
	
	if (write(txData, length+11) == true) {
		return true;
	}
	
	return false;
}

bool LpmsBBluetooth::parseModbusByte(void)
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
				parseFunction();
			} else {
				std::cout << "[LpmsBBluetooth] Checksum fail in data packet" << std::endl;
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

bool LpmsBBluetooth::pollData(void) 
{
	return true;
}

bool LpmsBBluetooth::deviceStarted(void)
{
	return isOpen;
}
