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

#ifndef LPMS_BLE
#define LPMS_BLE

#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <thread>
#include <mutex>

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"

#ifdef _WIN32
	#include "windows.h"
#endif

class LpmsBle : public LpmsIoInterface {
public:
	LpmsBle(CalibrationData *configData);
	~LpmsBle(void);
	
	bool connect(std::string deviceId);
	void close(void);
	bool pollData(void);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	bool parseModbusByte(void);
	bool deviceStarted(void);
	long long getConnectWait(void);
	static void listDevices(LpmsDeviceList *deviceList);	
	static void stopDiscovery(void);
	static int read_message(int timeout_ms);
	void run(void);
	void runTx(void);
	bool parseSensorData(void);
	void setConfiguration(void);
	int convertHexStringToNumber(const char *s);
	bool getTxMessage(std::queue<unsigned char> *topTxQ);

private:
	std::string portname;
	bool isOpen;
	std::string idNumber;
	std::string bluetoothAddress;
	MicroMeasure mm;
	std::queue<unsigned char> txQueue;
	bool started;
	std::string comStr;
	std::queue<unsigned char> txQ;	
};

#endif
