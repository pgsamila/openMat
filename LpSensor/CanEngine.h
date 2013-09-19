/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#ifndef CAN_ENGINE
#define CAN_ENGINE

#include <iostream>
#include <string>
#include <queue>
#include <list>

#ifdef _WIN32
	#include "windows.h"

	#include "PCANBasic.h"
	#include "PCANBasicClass.h"
	
	#include <stdio.h>
	#include <stdlib.h>
	#include <conio.h>
	#include <time.h>
	#include <string.h>

	#include <winsock.h>
#endif

#include "LpmsCanIo.h"
#include "DeviceListItem.h"
#include "LpmsDefinitions.h"

#define CMD_LEN 2048
#define CAN_GATEWAY_TCP_PORT 19227

class CanEngine {
private:
#ifdef _WIN32
	std::queue<TPCANMsg> txQ;
	TPCANHandle canChannel;
	PCANBasicClass pcan;
	
	UINT32 port;
	SOCKET connectionSocket;
	char commandStr[CMD_LEN], replyStr[CMD_LEN];	
#endif

	std::list<LpmsCanIo *> sensorList;

	bool peakCanInitialized;
	bool ixxatCanInitialized;
	
	bool peakCanDetected;
	bool ixxatCanDetected;
	
	int messageBytes;
	int headerBytes;
	int messageLen;
	int ixxatState;

	char headerData[64];
	char messageData[64];

public:
	/*! Constructor. */
	CanEngine(void);

	/*! Destructor. */
	~CanEngine(void);

	/*! Connect to CAN interface. */
	void connect(void);

	/*! Add one sensor to the list of sensors that are maintained by the CAN engine. */
	void addSensor(LpmsCanIo *s);

	/*! Poll data from CAN bus. */
	void poll(void);

	/*! Close connection to CAN bus interface. */
	void close(void);

	/*! Poll data from transmission queue. */
	bool updateSendQueue(void);

	/*! List CAN devices connected to the system. */
	void listDevices(LpmsDeviceList *v);

	/*! Remove sensor from list of connected sensors. */
	void removeSensor(LpmsCanIo *s);
	
	bool isInterfacePresent(void);
	
	void setBaudrate(int i);
	
	/* IXXAT related */
	bool sendCmdIxxat(char *commandPtr);
	bool getReplyIxxat(char *replyPtr, int *nBytes);
	bool connectIxxat(char *hostName, int port);
	bool initIxxat(void);
};

#endif
