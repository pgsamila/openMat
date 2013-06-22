/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_B_BLUETOOTH
#define LPMS_B_BLUETOOTH

#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"
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

	/* #include <stdio.h>
	#include <signal.h>
	#include <stdlib.h>
	#include <fcntl.h>
	#include <errno.h>
	#include <unistd.h>
	#include <sys/socket.h>
	#include <sys/time.h>
	#include <bluetooth/bluetooth.h>
	#include <bluetooth/hci.h>
	#include <bluetooth/hci_lib.h>
	#include <bluetooth/l2cap.h> */
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

class LpmsBBluetooth : public LpmsIoInterface {
public:
	LpmsBBluetooth(CalibrationData *configData);
	~LpmsBBluetooth(void);
	bool connect(string deviceId);
	void close(void);
	bool pollData(void);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	bool parseModbusByte(unsigned char b);
	bool deviceStarted(void);
	long long getConnectWait(void);
	static void listDevices(LpmsDeviceList *deviceList);	
	static void stopDiscovery(void);

private:
	bool read(char *rxBuffer, unsigned long *bytesReceived); 
	bool write(char *txBuffer, unsigned bufferLength);

#ifdef __GNUC__
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
