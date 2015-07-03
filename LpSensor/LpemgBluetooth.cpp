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

#include "LpemgBluetooth.h"

static volatile bool isStopDiscovery = false;
static HBLUETOOTH_RADIO_FIND m_bt = NULL;
static HBLUETOOTH_DEVICE_FIND m_bt_dev = NULL;

LpemgBluetooth::LpemgBluetooth(CalibrationData *configData) :
	LpemgIoInterface(configData)
{
}

LpemgBluetooth::~LpemgBluetooth(void)
{
	close();
}
	
long long LpemgBluetooth::getConnectWait(void) 
{ 
	return 6000000; 
}

void LpemgBluetooth::listDevices(LpmsDeviceList *deviceList)
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

	std::cout << "[LpemgBluetooth] Searching for Bluetooth LPMS devices" << std::endl;

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
					
					wprintf(L"[LpemgBluetooth] Device: %d\r\n", m_device_id);
					wprintf(L"[LpemgBluetooth] Instance Name: %s\r\n", m_device_info.szName);
					wprintf(L"[LpemgBluetooth] Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
						m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4], 
						m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2],
						m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);
					wprintf(L"[LpemgBluetooth] Class: 0x%08x\r\n", m_device_info.ulClassofDevice);
					wprintf(L"[LpemgBluetooth] Connected: %s\r\n", m_device_info.fConnected ? L"true" : L"false");
					wprintf(L"[LpemgBluetooth] Authenticated: %s\r\n", m_device_info.fAuthenticated ? L"true" : L"false");
					wprintf(L"[LpemgBluetooth] Remembered: %s\r\n", m_device_info.fRemembered ? L"true" : L"false");				
					
					sprintf(addressString, "%02X:%02X:%02X:%02X:%02X:%02X", 
						m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4], 
						m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2], 
						m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);

					if (m_device_info.szName[0] == 'L' && 
						m_device_info.szName[1] == 'P' && 
						m_device_info.szName[2] == 'E' && 
						m_device_info.szName[3] == 'M' &&
						m_device_info.szName[4] == 'G') {
						string s(addressString);
						
						int fD = 0;
						
						for (int i=0; i<deviceList->nDevices; ++i) {
							string s2(deviceList->device[i].deviceId);
							if (s2 == s) {
								fD = 1;
							}
						}
						
						if (fD == 0) {
							deviceList->push_back(DeviceListItem(s.c_str(), DEVICE_LPEMG_B));

							std::cout << "[LpemgBluetooth] Discovered device: " << s << endl;
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

void LpemgBluetooth::stopDiscovery(void)
{
	isStopDiscovery = true;	
}

bool LpemgBluetooth::connect(string deviceId)
{
	this->bluetoothAddress = deviceId;
    SOCKADDR_BTH sa = { 0 };
	unsigned long iMode = 1;
    int sa_len = sizeof(sa);
    WORD wVersionRequested;
    WSADATA wsaData;
    wVersionRequested = MAKEWORD(2, 0);

	if (WSAStartup(wVersionRequested, &wsaData) != 0) {
		isOpen = false;

		std::cout << "[LpemgBluetooth] Error during WSA Startup." << std::endl;

		return false;
    }

	if (SOCKET_ERROR == WSAStringToAddress(const_cast<char*>(bluetoothAddress.c_str()), AF_BTH, 
		NULL, (LPSOCKADDR) &sa, &sa_len )) {
		WSACleanup();
		isOpen = false;

		std::cout << "[LpemgBluetooth] Couldn't get Bluetooth address." << std::endl;

		return false;
    }

    sock = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
    if( SOCKET_ERROR == sock ) {
		close();

		std::cout << "[LpemgBluetooth] Couldn't create socket." << std::endl;

		return false;
    }
	
	sa.port = 1;
	
	int e = ::connect(sock, (LPSOCKADDR) &sa, sa_len);
    if (SOCKET_ERROR == e) {
		close();	

		std::cout << "[LpemgBluetooth] Couldn't connect." << std::endl;

		return false;
    }
	
	ioctlsocket(sock, FIONBIO, &iMode);
	
	std::cout << "[LpemgBluetooth] Connected!" << std::endl;	
	
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

void LpemgBluetooth::close(void)
{
	// if (isOpen == false) return;

	isOpen = false;
	
	// std::cout << "[LPMS-B] Closing connection" << std::endl;	
	
	shutdown(sock, SD_SEND);
	closesocket(sock);
}	

bool LpemgBluetooth::read(char *rxBuffer, unsigned long *bytesReceived) 
{
	float timeoutT;
	int n;
	
	n = recv(sock, rxBuffer, sizeof(rxBuffer), 0);
	
	if (n == SOCKET_ERROR) {
		timeoutT = (float) mm.measure() / 1e6f;
		*bytesReceived = 0;
	} else {
		*bytesReceived = (unsigned long) n;
		
		/* for (int i=0; i<(int)*bytesReceived; i++) {
			printf("%d, ", i);
			printf("%x ", rxBuffer[i]);
		}
		printf("\n "); */
		
		mm.reset();
		timeoutT = 0;
	}
	
	if (timeoutT > 10.0f) {
		std::cout << "[LpemgBluetooth] Bluetooth timeout" << std::endl;
		
		shutdown(sock, SD_SEND);
		closesocket(sock);
		
		while (!dataQueue.empty()) dataQueue.pop();
		
		return false;
	}
	
	return true;
}

bool LpemgBluetooth::write(char *txBuffer, unsigned bufferLength)
{
	if (send(sock, txBuffer, bufferLength, 0) == SOCKET_ERROR) {
		close();

		return false;
	}
	
	return true;
}

bool LpemgBluetooth::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
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

bool LpemgBluetooth::parseModbusByte(void)
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
				std::cout << "[LpemgBluetooth] Checksum fail in data packet" << std::endl;
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

bool LpemgBluetooth::pollData(void) 
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
		// printf("%x ", (unsigned char)rxBuffer[i]);
	}
	// if (bytesReceived > 0) printf("\n ");
	
	return true;
}

bool LpemgBluetooth::deviceStarted(void)
{
	return isOpen;
}
