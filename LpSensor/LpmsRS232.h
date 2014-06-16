#ifndef LPMS_RS232
#define LPMS_RS232

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

/*! \brief	Hardware driver class for LPMS-U sensor. */
class LpmsRS232 : public LpmsIoInterface {
public:
	/* Constructor. */
	LpmsRS232(CalibrationData *configData);

	/* Lists devices connected to the system. */
	static void listDevices(LpmsDeviceList *deviceList);

	/* Connects to LPMS. */
	bool connect(string deviceId);

	/* Closes sensor connection. */
	void close(void);

	/* Polls data from sensor. */
	bool pollData(void);

	/* Returns true if device has been successfully connected. */
	bool deviceStarted(void);

	/* Parses one byte of a LPBUS message. */
	bool parseModbusByte(void);
	
	void setRs232Baudrate(int i);

private:
	bool read(unsigned char *rxBuffer, unsigned long *bytesReceived); 
	bool write(unsigned char *txBuffer, unsigned bufferLength);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	long long getConnectWait(void);

#ifdef _WIN32
	HANDLE rs232Handle;
	DCB rs232Config;
#endif

	string portname;
	bool isOpen;
	string idNumber;
	int currentUartBaudrate;
};

#endif
