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

#ifndef LPEMG_BLUETOOTH
#define LPEMG_BLUETOOTH

#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpemgIoInterface.h"
#include "DeviceListItem.h"

#ifdef _WIN32
	#include <winsock2.h>
	#include <ws2bth.h>
	#include <windows.h>

	#include "BluetoothAPIs.h"
#endif

#ifdef __GNUC__
	// install libbluetooth-dev

	#include <stdio.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <sys/socket.h>
	#include <sys/ioctl.h>

	#include <bluetooth/bluetooth.h>
	#include <bluetooth/rfcomm.h>
	#include <bluetooth/hci.h>
	#include <bluetooth/hci_lib.h>
	#include <mutex>
#endif

#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7
#define PACKET_LENGTH0 8
#define PACKET_LENGTH1 9

class LpemgBluetooth : public LpemgIoInterface {
public:
	LpemgBluetooth(CalibrationData *configData);
	~LpemgBluetooth(void);
	bool connect(string deviceId);
	void close(void);
	bool pollData(void);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	bool parseModbusByte(void);
	bool deviceStarted(void);
	long long getConnectWait(void);
	static void listDevices(LpmsDeviceList *deviceList);	
	static void stopDiscovery(void);

private:
	bool read(char *rxBuffer, unsigned long *bytesReceived); 
	bool write(char *txBuffer, unsigned bufferLength);

#ifdef __GNUC__
	std::mutex mutexDataQueue;
	void runRead(void);
#endif

#ifdef _WIN32
	SOCKET sock;
#endif

#ifdef __GNUC__
	int bzSocket;
#endif

	bool isOpen;
	MicroMeasure mm;
	std::string idNumber;
	std::string bluetoothAddress;
};

#endif
