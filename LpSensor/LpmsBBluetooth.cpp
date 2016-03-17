/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsBBluetooth.h"

static volatile bool isStopDiscovery = false;
static HBLUETOOTH_RADIO_FIND m_bt = NULL;
static HBLUETOOTH_DEVICE_FIND m_bt_dev = NULL;

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
	BLUETOOTH_RADIO_INFO m_bt_info = { sizeof(BLUETOOTH_RADIO_INFO), 0, };
	BLUETOOTH_DEVICE_SEARCH_PARAMS m_search_params = {
		sizeof(BLUETOOTH_DEVICE_SEARCH_PARAMS),
		1,
		1,
		1,
		1,
		1,
		5,		
		NULL
	};

	BLUETOOTH_DEVICE_INFO m_device_info = { sizeof(BLUETOOTH_DEVICE_INFO), 0, };

	HANDLE m_radio = NULL;
	HBLUETOOTH_RADIO_FIND m_bt = NULL;
	HBLUETOOTH_DEVICE_FIND m_bt_dev = NULL;
	int m_radio_id;
	int m_device_id;
	DWORD mbtinfo_ret;
	
	isStopDiscovery = false;

	BLUETOOTH_FIND_RADIO_PARAMS m_bt_find_radio = {sizeof(BLUETOOTH_FIND_RADIO_PARAMS)};

	std::cout << "[LpmsBBluetooth] Searching for Bluetooth LPMS devices" << std::endl;

    WORD wVersionRequested;
    WSADATA wsaData;
    wVersionRequested = MAKEWORD(2, 0);

	for (int i=0; i<5; ++i) {
		WSAStartup(wVersionRequested, &wsaData);
	
		while (isStopDiscovery == false) {
			m_bt = BluetoothFindFirstRadio(&m_bt_find_radio, &m_radio);

			if (m_bt == NULL) break;
			
			m_radio_id = 0;

			do {
				mbtinfo_ret = BluetoothGetRadioInfo(m_radio, &m_bt_info);

				m_search_params.hRadio = m_radio;
				ZeroMemory(&m_device_info, sizeof(BLUETOOTH_DEVICE_INFO));
				m_device_info.dwSize = sizeof(BLUETOOTH_DEVICE_INFO);
				m_bt_dev = BluetoothFindFirstDevice(&m_search_params, &m_device_info);

				m_radio_id++;
				m_device_id = 0;

				do {
					char addressString[64];
					
					wprintf(L"[LpmsBBluetooth] Device: %d\r\n", m_device_id);
					wprintf(L"[LpmsBBluetooth] Instance Name: %s\r\n", m_device_info.szName);
					wprintf(L"[LpmsBBluetooth] Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
						m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4], 
						m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2],
						m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);
					wprintf(L"[LpmsBBluetooth] Class: 0x%08x\r\n", m_device_info.ulClassofDevice);
					wprintf(L"[LpmsBBluetooth] Connected: %s\r\n", m_device_info.fConnected ? L"true" : L"false");
					wprintf(L"[LpmsBBluetooth] Authenticated: %s\r\n", m_device_info.fAuthenticated ? L"true" : L"false");
					wprintf(L"[LpmsBBluetooth] Remembered: %s\r\n", m_device_info.fRemembered ? L"true" : L"false");				
					
					sprintf(addressString, "%02X:%02X:%02X:%02X:%02X:%02X", 
						m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4], 
						m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2], 
						m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);

					if (m_device_info.szName[0] == 'L' && 
						m_device_info.szName[1] == 'P' && 
						m_device_info.szName[2] == 'M' && 
						m_device_info.szName[3] == 'S') {
						string s(addressString);
						
						int fD = 0;
						
						for (int i=0; i<deviceList->nDevices; ++i) {
							string s2(deviceList->device[i].deviceId);
							if (s2 == s) {
								fD = 1;
							}
						}
						
						if (fD == 0) {
							deviceList->push_back(DeviceListItem(s.c_str(), DEVICE_LPMS_B));

							std::cout << "[LpmsBBluetooth] Discovered device: " << s << endl;
						}
					}

					m_device_id++;
					
					if (isStopDiscovery == true) break;
					
				} while(BluetoothFindNextDevice(m_bt_dev, &m_device_info) && isStopDiscovery == false);

				if (isStopDiscovery == true) {
					BluetoothFindDeviceClose(m_bt_dev);
					break;
				}

			} while(BluetoothFindNextRadio(&m_bt_find_radio, &m_radio) && isStopDiscovery == false);

			if (isStopDiscovery == true) {
				BluetoothFindRadioClose(m_bt);
				break;			
			}

			if(!BluetoothFindNextRadio(&m_bt_find_radio, &m_radio)) break;

			// Sleep(1000);
		}
	}
}

void LpmsBBluetooth::stopDiscovery(void)
{
	isStopDiscovery = true;	
}

bool LpmsBBluetooth::connect(string deviceId)
{
	this->bluetoothAddress = deviceId;
    SOCKADDR_BTH sa = { 0 };
	
    int sa_len = sizeof(sa);

    WORD wVersionRequested;
    WSADATA wsaData;
    wVersionRequested = MAKEWORD(2, 0);

	if (WSAStartup(wVersionRequested, &wsaData) != 0) {
		isOpen = false;

		std::cout << "[LpmsBBluetooth] Error during WSA Startup." << std::endl;

		return false;
    }

	if (SOCKET_ERROR == WSAStringToAddress(const_cast<char*>(bluetoothAddress.c_str()), AF_BTH, 
		NULL, (LPSOCKADDR) &sa, &sa_len )) {
		WSACleanup();
		isOpen = false;

		std::cout << "[LpmsBBluetooth] Couldn't get Bluetooth address." << std::endl;

		return false;
    }

    sock = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
    if( SOCKET_ERROR == sock ) {
		close();

		std::cout << "[LpmsBBluetooth] Couldn't create socket." << std::endl;

		return false;
    }
	
	sa.port = 1;
	
	int e = ::connect(sock, (LPSOCKADDR) &sa, sa_len);
    if (SOCKET_ERROR == e) {
		close();	

		std::cout << "[LpmsBBluetooth] Couldn't connect." << std::endl;

		return false;
    }
	
	unsigned long iMode = 1;
	ioctlsocket(sock, FIONBIO, &iMode);
	
	//std::cout << "[LpmsBBluetooth] Connected!" << std::endl;	
	
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
	
	imuId = 1;
	
	dataReceived = false;
	dataTimeout = 0;
	
	pCount = 0;
	isOpen = true;
	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;	
	
	return true;
}

void LpmsBBluetooth::close(void)
{
	// if (isOpen == false) return;

	isOpen = false;
	
	// std::cout << "[LPMS-B] Closing connection" << std::endl;	
	
	shutdown(sock, SD_SEND);
	closesocket(sock);
}	

bool LpmsBBluetooth::read(char *rxBuffer, unsigned long *bytesReceived) 
{
	float timeoutT;
	int n;
	
	n = recv(sock, rxBuffer, sizeof(rxBuffer), 0);
	
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
		
		shutdown(sock, SD_SEND);
		closesocket(sock);
		
		while (!dataQueue.empty()) dataQueue.pop();
		
		return false;
	}
	
	return true;
}

bool LpmsBBluetooth::write(char *txBuffer, unsigned bufferLength)
{
	// for lpms b2
	/*
	int bufL = 16;
	int l = bufferLength / bufL;
	int res = bufferLength % bufL;
	int p = 0;
	for (int i = 0; i < l; ++i)
	{
		if (send(sock, txBuffer + p, bufL, 0) == SOCKET_ERROR) {
			close();
			return false;
		}
		p += bufL;
	}
	if (send(sock, txBuffer + p, res, 0) == SOCKET_ERROR) {
		close();
		return false;
	}
	*/

	if (send(sock, txBuffer, bufferLength, 0) == SOCKET_ERROR) {
		close();
		return false;
	}
	return true;
}

bool LpmsBBluetooth::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1013) return false;

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
	unsigned long bytesReceived;
	char rxBuffer[1024];
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

bool LpmsBBluetooth::deviceStarted(void)
{
	return isOpen;
}
