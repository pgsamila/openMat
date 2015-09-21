#include "LpmsTcp.h"

#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7

LpmsTcp::LpmsTcp(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
}
	
long long LpmsTcp::getConnectWait(void) { 
	return 1000000; 
}	
	
void LpmsTcp::listDevices(LpmsDeviceList *deviceList) 
{
	int i;
	TcpClientCom t;
	std::string a = "192.168.161.5";
	int p = 7000;
	int s = 0;
	std::string aS;

	for (i=0; i<16; ++i) {	
		if (t.CreateSocket(s, kFBTCPIP_Stream)) {
			if (t.Connect(s, a.c_str(), p+i) == true) {
				std::ostringstream oss;
				oss << a << ":" << p+i;
				std::cout << "[LpmsTcp] Discovered TCP sensor: " << oss.str() << std::endl;
				deviceList->push_back(DeviceListItem(oss.str().c_str(), DEVICE_LPMS_TCP));
			}
			t.CloseSocket(s);	
		}
	}
}

bool LpmsTcp::connect(string deviceId)
{
	std::ostringstream oss;
	this->idNumber = deviceId;
	isOpen = false;
	mNetworkAddress = "192.168.161.5";
	mNetworkPort = 7000;
	mSocket = 0;
	
	size_t f = (int)deviceId.find(":");
	mNetworkPort = std::stoi(deviceId.substr(f+1, deviceId.length()-f));	
	mNetworkAddress = deviceId.substr(0, f);
	
	if (mNetworkPort < 0 || mNetworkPort > 99999) return false;
	
	oss << mNetworkAddress << ":" << mNetworkPort;
	std::cout << "[LpmsTcp] Trying to connect to " << oss.str() << std::endl;
	
	if (isOpen == false) {
		if (mTCP.CreateSocket(mSocket, kFBTCPIP_Stream)) {
			if (mTCP.Connect(mSocket, mNetworkAddress.c_str(), mNetworkPort) == false) {
				std::cout << "[LpmsTcp] Connection to server failed." << std::endl;
				if (mSocket) mTCP.CloseSocket(mSocket);
				mSocket = 0; 
			} else {
				std::cout << "[LpmsTcp] Connected to server." << std::endl;
				
				unsigned long iMode = 1;
				ioctlsocket(mSocket, FIONBIO, &iMode);
			}
		}
	} else {
		std::cout << "Another connection has been established." << std::endl;
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
	isOpen = true;
	
	mm.reset();
	
	return true;
}

bool LpmsTcp::write(char *txBuffer, unsigned bufferLength)
{
	if (send(mSocket, txBuffer, bufferLength, 0) == SOCKET_ERROR) {
		close();

		return false;
	}
	
	return true;
}

bool LpmsTcp::read(char *rxBuffer, unsigned long *bytesReceived) {
	float timeoutT;
	int n;
	
	n = recv(mSocket, rxBuffer, sizeof(rxBuffer), 0);
	
	if (n == SOCKET_ERROR) {
		timeoutT = (float) mm.measure() / 1e6f;
		*bytesReceived = 0;
	} else {
		*bytesReceived = (unsigned long) n;
		
		mm.reset();
		timeoutT = 0;
	}
	
	if (timeoutT > 10.0f) {
		std::cout << "[LpmsBBluetooth] Bluetooth timeout" << std::endl;
		
		shutdown(mSocket, SD_SEND);
		closesocket(mSocket);
		
		while (!dataQueue.empty()) dataQueue.pop();
		
		return false;
	}

	return true;
}

void LpmsTcp::close(void) {	
	if (isOpen == false) return;

	isOpen = false;
	
	mTCP.CloseSocket(mSocket);
	mSocket = 0;

	cout << "[LpmsTcp] Disconnected from server." << std::endl;

	return;
}

bool LpmsTcp::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
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

bool LpmsTcp::parseModbusByte(void)
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

bool LpmsTcp::pollData(void) 
{
	unsigned long bytesReceived;
	char rxBuffer[4096];
	bool packetOk = false;
	
	if (deviceStarted() == false) return false;

	if (read(rxBuffer, &bytesReceived) == false) {
		isOpen = false;
		return false;
	}	

	for (unsigned int i=0; i < bytesReceived; i++) {
		dataQueue.push((char) rxBuffer[i]);
	}	
	
	return true;
}

bool LpmsTcp::deviceStarted(void)
{
	return isOpen;
}