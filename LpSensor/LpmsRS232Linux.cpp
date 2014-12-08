#include "LpmsRS232.h"

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

LpmsRS232::LpmsRS232(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
}
	
long long LpmsRS232::getConnectWait(void) 
{ 
	return 0; 
}	
	
void LpmsRS232::listDevices(LpmsDeviceList* deviceList) 
{
	(void) deviceList;
}

bool LpmsRS232::connect(string deviceId)
{	
	(void) deviceId;

	return true;
}

bool LpmsRS232::write(unsigned char *txBuffer, unsigned bufferLength)
{
	(void) txBuffer;
	(void) bufferLength;

	return true;
}

bool LpmsRS232::read(unsigned char *rxBuffer, unsigned long *bytesReceived) {
	(void) rxBuffer;
	(void) bytesReceived;

	return true;
}

void LpmsRS232::close(void) {	
	return;
}

bool LpmsRS232::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	(void) address;
	(void) function;
	(void) length;
	(void) *data;

	return false;
}

bool LpmsRS232::parseModbusByte(void)
{
	return true;
}

bool LpmsRS232::pollData(void) 
{
	return true;
}

bool LpmsRS232::deviceStarted(void)
{
	return isOpen;
}

void LpmsRS232::setRs232Baudrate(int i)
{
	currentUartBaudrate = i;
}
