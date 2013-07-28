/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef LPMS_U
#define LPMS_U

#include <iostream>
#include <string>
#include <queue>
#include <vector>
using namespace std;

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"

#ifdef _WIN32
	#include "windows.h"
#endif

#include "ftd2xx.h"

class LpmsU : public LpmsIoInterface {
public:
	LpmsU(CalibrationData *configData);
	static void listDevices(LpmsDeviceList *deviceList);
	bool connect(string deviceId);
	void close(void);
	bool pollData(void);
	bool deviceStarted(void);
	bool parseModbusByte(void);

private:
//#ifdef WIN32
	bool read(unsigned char *rxBuffer, unsigned long *bytesReceived); 
	bool write(unsigned char *txBuffer, unsigned bufferLength);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
//#endif

/*#ifdef __GNUC__
	bool read(unsigned char *rxBuffer, unsigned int *bytesReceived); 
	bool write(unsigned char *txBuffer, unsigned bufferLength);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
#endif*/

	long long getConnectWait(void);

	FT_HANDLE ftHandle;
	bool isOpen;
	string idNumber;
};

#endif
