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

#define LPMS_GOTO_BROADCAST	0
#define LPMS_GOTO_SLEEP 1
#define LPMS_RAW_DATA 61

#define LPMS_FACTORY_IMU_ID 1

LpmsRS232::LpmsRS232(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
	currentUartBaudrate = SELECT_LPMS_UART_BAUDRATE_115200;
}
	
long long LpmsRS232::getConnectWait(void) { 
	return 1000000; 
}	
	
void LpmsRS232::listDevices(LpmsDeviceList *deviceList) 
{
	int i;
	HANDLE h;
	
	for (i=0; i<255; ++i) {
		std::ostringstream pS;
		pS << "\\\\.\\COM" << i;
		h = CreateFile(pS.str().c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
		if (h != INVALID_HANDLE_VALUE) {
			std::ostringstream pS;
			pS << "COM" << i;
			deviceList->push_back(DeviceListItem(pS.str().c_str(), DEVICE_LPMS_RS232));
			CloseHandle(h);
		}
	}
}

bool LpmsRS232::connect(string deviceId)
{
	std::ostringstream pS;
	pS << "\\\\.\\" << deviceId;

	this->idNumber = deviceId;
	
	isOpen = false;
	
	rs232Handle = CreateFile(pS.str().c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);	

	if (GetCommState(rs232Handle, &rs232Config) == 0) {
		return false;
    }

	switch (currentUartBaudrate) {
	case SELECT_LPMS_UART_BAUDRATE_19200:
		rs232Config.BaudRate = 19200;
	break;	
	
	case SELECT_LPMS_UART_BAUDRATE_57600:
		rs232Config.BaudRate = 57600;
	break;
	
	case SELECT_LPMS_UART_BAUDRATE_115200:
		rs232Config.BaudRate = 115200;
	break;

	case SELECT_LPMS_UART_BAUDRATE_921600:
		rs232Config.BaudRate = 921600;
	break;
	
	default:
		rs232Config.BaudRate = 115200;
	break;
	}
	
	rs232Config.StopBits = ONESTOPBIT;  
	rs232Config.Parity = NOPARITY;     
	rs232Config.ByteSize = 8;  

	if (SetCommState(rs232Handle, &rs232Config) == 0) {
		return false;
	}

    COMMTIMEOUTS comTimeOut;                   
    comTimeOut.ReadIntervalTimeout = 5;
    comTimeOut.ReadTotalTimeoutMultiplier = 5;
    comTimeOut.ReadTotalTimeoutConstant = 5;
    comTimeOut.WriteTotalTimeoutMultiplier = 5;
    comTimeOut.WriteTotalTimeoutConstant = 5;
	
    SetCommTimeouts(rs232Handle, &comTimeOut);

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
	isOpen = true;
	
	return true;
}

bool LpmsRS232::write(unsigned char *txBuffer, unsigned bufferLength)
{
	unsigned long l;

	if (WriteFile(rs232Handle, txBuffer, bufferLength, &l, NULL) == 0) {
		cout << "[LpmsRS232] Writing serial port failed." << endl;
		return false;
    }
	
	return true;
}

bool LpmsRS232::read(unsigned char *rxBuffer, unsigned long *bytesReceived) {
	if (ReadFile(rs232Handle, rxBuffer, 64, bytesReceived, NULL) == 0) {
		cout << "[LpmsRS232] Reading serial port failed." << endl;
		return false;
	}

	return true;
}

void LpmsRS232::close(void) {	
	if (isOpen == false) return;

	isOpen = false;
	
	CloseHandle(rs232Handle);

	return;
}

bool LpmsRS232::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	unsigned char txData[1024];
	unsigned int txLrcCheck;
	// int i;
	
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
		// for (i=0; i<length+11; ++i) printf("%x ", txData[i]);
		return true;
	}
	
	return false;
}

int c2 = 0;

bool LpmsRS232::parseModbusByte(void)
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
				cout << "[LPMS-U] Checksum fail in data packet" << endl;
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

bool LpmsRS232::pollData(void) 
{
	unsigned long bytesReceived;
	unsigned char rxBuffer[4096];
	bool packetOk = false;
	
	if (deviceStarted() == false) return false;

	if (read(rxBuffer, &bytesReceived) == false) {
		isOpen = false;
		return false;
	}	

	for (unsigned int i=0; i < bytesReceived; i++) {
		dataQueue.push((unsigned char) rxBuffer[i]);
	}	
	
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