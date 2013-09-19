/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CanEngine.h"

#define MAX_RX_BYTES 4096

CanEngine::CanEngine(void) 
{
	peakCanInitialized = false;
	peakCanDetected = false;
	
	ixxatCanDetected = false;
	ixxatCanInitialized = false;

	messageBytes = 0;
	headerBytes = 0;
	messageLen = 0;
	ixxatState = 0;
	
	// initIxxat();
}

CanEngine::~CanEngine(void) 
{
	close();
}

void CanEngine::listDevices(LpmsDeviceList *v)
{	
	if (peakCanInitialized == false && ixxatCanInitialized == false) return;

	for (int i=0; i<18; i++) {
		printf("[CanEngine] Scanning for device %d\n", i);
	
		CalibrationData c;
		std::stringstream ss;
		bool f = false;

		ss << i;

		BOOST_FOREACH(LpmsCanIo *cio, sensorList) {
			int p;
			cio->getConfigData()->getParameter(PRM_OPENMAT_ID, &p);
			if (p == i) f = true;
		}
		
		if (f == true) {
			v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C));
			break;
		}

		c.setParameter(PRM_DEVICE_ID, ss.str());

		LpmsCanIo sensor(&c);
		sensor.connect(ss.str());
		addSensor(&sensor);

		sensor.setCommandMode();
		sensor.setCommandMode();

		MicroMeasure mm;
		mm.reset();
		
		while (mm.measure() < 500000) {
			poll();
			sensor.checkState();
		}
		
		sensor.setCommandMode();		
		
		mm.reset();
		
		while (mm.measure() < 500000) {
			poll();
			sensor.checkState();

			if (sensor.isWaitForAck() == false) {
				v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C));
				std::cout << "[LpmsCanIo] Discovered device: " << i << std::endl;
				break;
			}
		}

		removeSensor(&sensor);
	}
}

bool CanEngine::initIxxat(void)
{
	char hostAddr[64] = "192.168.0.10";
	char replyStr[MAX_RX_BYTES];
	int n;
	
	connectIxxat(hostAddr, 19227);
	
	printf("[CanEngine] IXXAT Device initialization\n");

	sendCmdIxxat("d ver\r\n");
	getReplyIxxat(replyStr, &n);

	sendCmdIxxat("c init 20\r\n");
	getReplyIxxat(replyStr, &n);
	if (strncmp(replyStr, "I OK", 4) != 0) {
		printf("[CanEngine] IXXAT init error!\n");
		return false;
	}

	sendCmdIxxat("c start\r\n");
	getReplyIxxat(replyStr, &n);
	if (strncmp(replyStr, "I OK", 4) != 0) {
		printf("[CanEngine] CAN start error!\n");
		return false;
	}

	sendCmdIxxat("c filter disable\r\n");
	getReplyIxxat(replyStr, &n);

	printf("[CanEngine] IXXAT initialization OK\n");
	
	ixxatCanDetected = true;
	ixxatCanInitialized = true;
	
	unsigned long iMode = 1;
	ioctlsocket(connectionSocket, FIONBIO, &iMode);
	
	return true;
}

bool CanEngine::connectIxxat(char *hostName, int port) 
{
	WSADATA wsda;
	struct hostent *host;
	SOCKADDR_IN addr;
	int ret;

	WSAStartup(MAKEWORD(1,1), &wsda);

	printf("[CanEngine] Creating socket for IXXAT interface\n");

	connectionSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	
	if (connectionSocket == SOCKET_ERROR) {
		printf("[CanEngine] Socket creation error %d!\n", WSAGetLastError());
		
		return false;
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(hostName);

	if (addr.sin_addr.s_addr == INADDR_NONE) {
		host = NULL;
		printf("[CanEngine] Resolving host\n");
		host = gethostbyname(hostName);  

		if (host == NULL) {
			printf("[CanEngine] Unknown host: %s!\n", hostName);
			return false;
		}

		memcpy(&addr.sin_addr, host->h_addr_list[0], host->h_length);
	}

	printf("[CanEngine] Connecting to %s:%d\n", hostName, port);

	ret = ::connect(connectionSocket, (struct sockaddr *) &addr, sizeof(addr));

	if (ret == SOCKET_ERROR) {
		printf("[CanEngine] Socket error %d!\n", WSAGetLastError());
		return false;
	}
	
	printf("[CanEngine] Connection with IXXAT interface OK\n"); 
	
	ixxatCanDetected = true;
	
	return true;
}

bool CanEngine::sendCmdIxxat(char *commandPtr)
{
	int bytesSent;
	char command[CMD_LEN];
	
	memset(command, 0, CMD_LEN);
	
	strcpy(command, commandPtr);

	bytesSent = send(connectionSocket, command, strlen(command), 0);
 
	if (bytesSent <= 0) {
		printf("[CanEngine] IXXAT send failed!\n");
		return false;
	}
	
	return true;
}

bool CanEngine::getReplyIxxat(char *replyPtr, int *nBytes)
{
	int bytesRecv;

	bytesRecv = recv(connectionSocket, replyPtr, MAX_RX_BYTES, 0);

	if (bytesRecv == SOCKET_ERROR) {		
		return false;
	}

	replyPtr[bytesRecv] = 0;
	*nBytes = bytesRecv;
	
	// printf("[CanEngine] IXXAT receive reply: %s\n", replyPtr);  

	return 0;
}

void CanEngine::connect(void) 
{
	TPCANStatus r;

	canChannel = PCAN_USBBUS1;

	if (peakCanInitialized == false) {
		r = pcan.Initialize(canChannel, PCAN_BAUD_1M);
		if (r != PCAN_ERROR_OK) {
			std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;
			return;
		}
		
		boost::uint8_t v = PCAN_PARAMETER_ON;
		r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);
	}

	peakCanDetected = true;
	peakCanInitialized = true;
}

void CanEngine::addSensor(LpmsCanIo *s)
{
	sensorList.push_back(s);
}

void CanEngine::removeSensor(LpmsCanIo *s)
{
	sensorList.remove(s);
}

void CanEngine::poll() {
	TPCANStatus r;
	TPCANMsg m;
	TPCANTimestamp t;
	char replyStr[MAX_RX_BYTES+10];
	int n=0;
	char messageIdStr[16];
	int messageId = 0;
	char messageByteStr[16];
	int messageByte[16];
	int v = 0;
	int nB = 0;
	char lengthStr[16];

	updateSendQueue();	
	
	if (peakCanInitialized == true) {
		r = pcan.Read(canChannel, &m, &t);

		if (r != PCAN_ERROR_OK) {
			return;
		}
		
		BOOST_FOREACH(LpmsCanIo *c, sensorList) {
			c->parseCanMsg(m);
		}
	}

	if (ixxatCanInitialized == true) {
		getReplyIxxat(replyStr, &n);
		
		for (int i=0; i<n; ++i) {
			if (replyStr[i] == 'M' && ixxatState == 0) {
				ixxatState = 1;
				headerBytes = 0;
			} else if (replyStr[i] == 'E' && ixxatState == 0) {
			}
						
			if (ixxatState == 1) {
				headerData[headerBytes] = replyStr[i];
				++headerBytes;
								
				if (headerBytes == 11) {
					lengthStr[0] = headerData[4];
					lengthStr[1] = 0;
					messageLen = atoi(lengthStr);
					
					for (int j=6; j<10; ++j) messageIdStr[j-6] = headerData[j];
					messageIdStr[4] = 0;
					
					messageId = 0;
					for (int k=0; k<4; ++k) {
						switch (messageIdStr[k]) {
						case 'A': v = 10; break;		
						case 'B': v = 11; break;
						case 'C': v = 12; break;
						case 'D': v = 13; break;
						case 'E': v = 14; break;
						case 'F': v = 15; break;
						default:
							messageByteStr[0] = messageIdStr[k];
							messageByteStr[1] = 0;
							v = atoi(messageByteStr);
						break;
						}
						switch (k) {
						case 0: messageId += v * 4096; break;
						case 1: messageId += v * 256; break;
						case 2: messageId += v * 16; break;
						case 3: messageId += v; break;
						}
					}					
					
					ixxatState = 2;
					messageBytes = 0;
				}
			} else if (ixxatState == 2) {
				messageData[messageBytes] = replyStr[i];
				++messageBytes;
								
				if (messageBytes == (messageLen * 3 - 1)) {
					for (int j=0; j<messageBytes; j+=3) {
						for (int k=0; k<2; ++k) {
							switch (messageData[j+k]) {
							case 'A': v = 10; break;		
							case 'B': v = 11; break;
							case 'C': v = 12; break;
							case 'D': v = 13; break;
							case 'E': v = 14; break;
							case 'F': v = 15; break;
							default:
								messageByteStr[0] = messageData[j+k];
								messageByteStr[1] = 0;
								v = atoi(messageByteStr);
							break;
							}
							if (k==1) messageByte[j/3] += v; else messageByte[j/3] = v * 16;
						}
					}
					
					/* printf("Parsed message ID %d LEN %d: %x %x %x %x %x %x %x %x\n",
						messageId,
						messageLen,
						messageByte[0],
						messageByte[1],
						messageByte[2],
						messageByte[3],
						messageByte[4],
						messageByte[5],
						messageByte[6],
						messageByte[7]); */
						
					m.ID = messageId;
					m.MSGTYPE = PCAN_MESSAGE_STANDARD;
					m.LEN = messageLen;
					for (int j=0; j<8; ++j) m.DATA[j] = messageByte[j];
					
					BOOST_FOREACH(LpmsCanIo *c, sensorList) c->parseCanMsg(m);	

					ixxatState = 0;
				}		
			}
		}
	}
}

void CanEngine::close() {
	if (peakCanInitialized == true) {
		pcan.Uninitialize(canChannel);
	}

	peakCanInitialized = false;
}

bool CanEngine::updateSendQueue(void)
{
	TPCANStatus r;		
	TPCANMsg m;
	char commandStr[64];

	BOOST_FOREACH(LpmsCanIo *c, sensorList) {
		int rT = 0;
		while (c->getTxMessage(&txQ) == true && rT < 1024) ++rT;
	}
	
	long timeout = 0;
	
	if (txQ.size() > 0) {
		m = txQ.front();

		// printf("Msg. write %x %x %x %x %x %x %x %x\n", m.DATA[0], m.DATA[1], m.DATA[2], m.DATA[3], m.DATA[4], m.DATA[5], m.DATA[6], m.DATA[7]);
		
		if (peakCanInitialized == true) {
			r = pcan.Write(canChannel, &m);
		}

		if (ixxatCanInitialized == true) {		
			sprintf(commandStr, "M SD%x %x", m.LEN, m.ID);
			sendCmdIxxat(commandStr);
			for (int i=0; i<m.LEN; ++i) {
				sprintf(commandStr, " %x", m.DATA[i]);
				sendCmdIxxat(commandStr);
			}	
			sprintf(commandStr, "\r\n");
			sendCmdIxxat(commandStr);
		}
		
		std::this_thread::sleep_for(std::chrono::microseconds(7500));
		
		if (r == PCAN_ERROR_QXMTFULL) {
		}
		
		timeout = 0;
		txQ.pop();
	}
	
	return true;
}

bool CanEngine::isInterfacePresent(void)
{
	TPCANStatus r;
	canChannel = PCAN_USBBUS1;

	if (peakCanInitialized == false) {
		r = pcan.Initialize(canChannel, PCAN_BAUD_500K);
		
		if (r != PCAN_ERROR_OK) {
			std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;
			
			return false;	
		} 
		
		pcan.Uninitialize(canChannel);
	}
	
	return true;
}

void CanEngine::setBaudrate(int i)
{
	TPCANStatus r;
	canChannel = PCAN_USBBUS1;

	if (peakCanInitialized == true) pcan.Uninitialize(canChannel);
	
	switch (i) {
	case 0:
		r = pcan.Initialize(canChannel, PCAN_BAUD_20K);
	break;	
	
	case 1:
		r = pcan.Initialize(canChannel, PCAN_BAUD_50K);
	break;

	case 2:
		r = pcan.Initialize(canChannel, PCAN_BAUD_125K);
	break;
	
	case 3:
		r = pcan.Initialize(canChannel, PCAN_BAUD_250K);
	break;

	case 4:
		r = pcan.Initialize(canChannel, PCAN_BAUD_500K);
	break;

	case 5:	
		r = pcan.Initialize(canChannel, PCAN_BAUD_1M);
	break;
	}
		
	if (r != PCAN_ERROR_OK) {
		std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;

		peakCanInitialized = false;
	} 
	
	boost::uint8_t v = PCAN_PARAMETER_ON;
	r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);

	peakCanInitialized = true;
}